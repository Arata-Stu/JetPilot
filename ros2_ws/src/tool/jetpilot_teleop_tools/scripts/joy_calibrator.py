#!/usr/bin/env python3
"""Interactive joystick profile calibrator and terminal tester."""

from __future__ import annotations

import argparse
import array
import fcntl
import glob
import os
import select
import struct
import sys
import termios
import time
from dataclasses import dataclass, field
from typing import Any


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12
JSIOCGNAME = lambda length: 0x80006A13 + (length << 16)

AXIS_MIN = -32767
AXIS_MAX = 32767
AXIS_DETECTION_THRESHOLD = 10000
TRIGGER_MOVEMENT_THRESHOLD = 30000
STICK_MOVEMENT_THRESHOLD = 22000
BUTTON_NAMES = [
    ("cross", "x / cross"),
    ("circle", "circle"),
    ("triangle", "triangle"),
    ("square", "square"),
    ("l1", "L1"),
    ("r1", "R1"),
    ("share", "share"),
    ("options", "options"),
    ("ps", "PS"),
    ("l3", "L3"),
    ("r3", "R3"),
]
DPAD_DIRECTIONS = [
    ("up", "d-pad up"),
    ("right", "d-pad right"),
    ("down", "d-pad down"),
    ("left", "d-pad left"),
]


@dataclass
class JsEvent:
    time_ms: int
    value: int
    event_type: int
    number: int

    @property
    def is_button(self) -> bool:
        return (self.event_type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON

    @property
    def is_axis(self) -> bool:
        return (self.event_type & ~JS_EVENT_INIT) == JS_EVENT_AXIS

    @property
    def is_init(self) -> bool:
        return bool(self.event_type & JS_EVENT_INIT)


@dataclass
class DeviceInfo:
    path: str
    name: str
    axes: int
    buttons: int
    vendor_id: str = ""
    product_id: str = ""


@dataclass
class JoyState:
    axes: dict[int, int] = field(default_factory=dict)
    buttons: dict[int, int] = field(default_factory=dict)


class JoystickDevice:
    def __init__(self, path: str):
        self.path = path
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.info = inspect_device(path, self.fd)
        self.state = JoyState(
            axes={index: 0 for index in range(self.info.axes)},
            buttons={index: 0 for index in range(self.info.buttons)},
        )
        self.drain_initial_events()

    def close(self) -> None:
        os.close(self.fd)

    def drain_initial_events(self) -> None:
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            for event in self.read_events(timeout=0.02):
                self.apply(event)

    def apply(self, event: JsEvent) -> None:
        if event.is_axis:
            self.state.axes[event.number] = event.value
        elif event.is_button:
            self.state.buttons[event.number] = event.value

    def read_events(self, timeout: float = 0.0) -> list[JsEvent]:
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return []
        events = []
        while True:
            try:
                data = os.read(self.fd, 8)
            except BlockingIOError:
                break
            if len(data) != 8:
                break
            event = JsEvent(*struct.unpack("IhBB", data))
            self.apply(event)
            events.append(event)
        return events

    def snapshot(self) -> JoyState:
        return JoyState(axes=dict(self.state.axes), buttons=dict(self.state.buttons))


def inspect_device(path: str, fd: int | None = None) -> DeviceInfo:
    own_fd = fd is None
    if fd is None:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    try:
        axes = array.array("B", [0])
        buttons = array.array("B", [0])
        name = array.array("B", [0] * 128)
        fcntl.ioctl(fd, JSIOCGAXES, axes, True)
        fcntl.ioctl(fd, JSIOCGBUTTONS, buttons, True)
        try:
            fcntl.ioctl(fd, JSIOCGNAME(len(name)), name, True)
            decoded_name = name.tobytes().split(b"\0", 1)[0].decode(errors="replace")
        except OSError:
            decoded_name = os.path.basename(path)
        vendor_id, product_id = read_usb_ids(path)
        return DeviceInfo(path, decoded_name, axes[0], buttons[0], vendor_id, product_id)
    finally:
        if own_fd:
            os.close(fd)


def read_usb_ids(path: str) -> tuple[str, str]:
    event_dir = os.path.realpath(f"/sys/class/input/{os.path.basename(path)}/device")
    current = event_dir
    for _ in range(8):
        vendor_path = os.path.join(current, "id", "vendor")
        product_path = os.path.join(current, "id", "product")
        if os.path.exists(vendor_path) and os.path.exists(product_path):
            with open(vendor_path, encoding="utf-8") as vendor_file:
                vendor = vendor_file.read().strip()
            with open(product_path, encoding="utf-8") as product_file:
                product = product_file.read().strip()
            return vendor, product
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    return "", ""


def list_devices() -> list[DeviceInfo]:
    devices = []
    for path in sorted(glob.glob("/dev/input/js*")):
        try:
            devices.append(inspect_device(path))
        except OSError:
            continue
    return devices


def select_device(path: str | None) -> JoystickDevice:
    if path:
        return JoystickDevice(path)
    devices = list_devices()
    if not devices:
        raise RuntimeError("No /dev/input/js* devices were found.")
    print("\nDetected devices\n")
    for index, info in enumerate(devices):
        print(f"[{index}] {info.name}")
        print(f"    {info.path}")
    while True:
        value = input("\nSelect device number: ").strip()
        try:
            return JoystickDevice(devices[int(value)].path)
        except (ValueError, IndexError):
            print("Please enter one of the listed numbers.")


def wait_for_enter() -> None:
    input("\nPress Enter to continue.")


def observe_idle(device: JoystickDevice, duration: float) -> JoyState:
    print("\nDo not touch the controller.")
    print("Measuring center values and noise...\n")
    samples: dict[int, list[int]] = {axis: [] for axis in device.state.axes}
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        device.read_events(timeout=0.02)
        for axis, value in device.state.axes.items():
            samples[axis].append(value)
    centers = {axis: int(sum(values) / len(values)) if values else 0 for axis, values in samples.items()}
    print("Axis center candidates:")
    for axis, value in centers.items():
        spread = max(samples[axis]) - min(samples[axis]) if samples[axis] else 0
        print(f"  axis {axis}: center {value}, noise {spread}")
    return JoyState(axes=centers, buttons=dict(device.state.buttons))


def confirm_detection(text: str) -> bool:
    answer = input(f"{text}\nOK? [Enter: accept / r: retry] ").strip().lower()
    return answer != "r"


def detect_button(
    device: JoystickDevice,
    prompt: str,
    assigned_buttons: set[int],
    timeout: float = 15.0,
) -> int:
    while True:
        print(f"\n{prompt}")
        print("Waiting...")
        before = device.snapshot()
        deadline = time.monotonic() + timeout
        candidate = None
        while time.monotonic() < deadline and candidate is None:
            for event in device.read_events(timeout=0.02):
                if event.is_button and not event.is_init and event.value == 1:
                    previous = before.buttons.get(event.number, 0)
                    if previous == 0 and event.number not in assigned_buttons:
                        candidate = event.number
                        break
        if candidate is None:
            print("No button press was detected.")
            continue
        if confirm_detection(f"Detected: button {candidate}"):
            assigned_buttons.add(candidate)
            return candidate


def collect_changes(device: JoystickDevice, duration: float) -> tuple[JoyState, dict[int, list[int]], dict[int, list[int]]]:
    before = device.snapshot()
    axis_samples = {axis: [value] for axis, value in before.axes.items()}
    button_samples = {button: [value] for button, value in before.buttons.items()}
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        device.read_events(timeout=0.02)
        for axis, value in device.state.axes.items():
            axis_samples.setdefault(axis, []).append(value)
        for button, value in device.state.buttons.items():
            button_samples.setdefault(button, []).append(value)
    return before, axis_samples, button_samples


def detect_trigger(device: JoystickDevice, name: str, assigned_buttons: set[int], assigned_axes: set[int]) -> dict[str, Any]:
    while True:
        print(f"\nSlowly press {name} all the way, then release it.")
        print("Recording for 3 seconds...")
        before, axis_samples, button_samples = collect_changes(device, 3.0)
        axis_candidates = []
        for axis, values in axis_samples.items():
            movement = max(values) - min(values)
            if movement > TRIGGER_MOVEMENT_THRESHOLD and axis not in assigned_axes:
                axis_candidates.append((movement, axis, min(values), max(values)))
        button_candidates = []
        for button, values in button_samples.items():
            compact = compact_sequence(values)
            if compact == [0, 1, 0] and button not in assigned_buttons:
                button_candidates.append(button)
        axis_candidates.sort(reverse=True)
        print("\nDetected changes")
        for movement, axis, min_value, max_value in axis_candidates[:3]:
            print(f"  axis {axis}: min {min_value}, max {max_value}, movement {movement}")
        for button in button_candidates[:3]:
            print(f"  button {button}: 0 -> 1 -> 0")
        if not axis_candidates and not button_candidates:
            print("No usable trigger movement was detected.")
            continue
        _, axis, min_value, max_value = axis_candidates[0] if axis_candidates else (0, -1, 0, 0)
        button = button_candidates[0] if button_candidates else -1
        initial_value = before.axes.get(axis, 0)
        pressed_value = max(
            axis_samples.get(axis, [initial_value]),
            key=lambda value: abs(value - initial_value),
        )
        inverted = pressed_value < initial_value
        if confirm_detection(f"{name} analog = axis {axis}\n{name} digital = button {button}\ninverted = {str(inverted).lower()}"):
            if axis >= 0:
                assigned_axes.add(axis)
            if button >= 0:
                assigned_buttons.add(button)
            return {
                "axis": axis,
                "button": button,
                "min": min_value,
                "max": max_value,
                "released": initial_value,
                "pressed": pressed_value,
                "deadzone": 500,
                "inverted": inverted,
            }


def compact_sequence(values: list[int]) -> list[int]:
    compact = []
    for value in values:
        if not compact or compact[-1] != value:
            compact.append(value)
    return compact


def detect_stick(device: JoystickDevice, name: str, assigned_axes: set[int]) -> dict[str, Any]:
    while True:
        print(f"\nMove the {name} stick in a large circle.")
        print("Recording for 5 seconds...")
        before, axis_samples, _ = collect_changes(device, 5.0)
        candidates = []
        for axis, values in axis_samples.items():
            movement = max(values) - min(values)
            if movement > STICK_MOVEMENT_THRESHOLD and axis not in assigned_axes:
                center = before.axes.get(axis, 0)
                candidates.append((movement, axis, min(values), max(values), center))
        candidates.sort(reverse=True)
        if len(candidates) < 2:
            print("Could not find two moving axes for this stick.")
            continue
        x = candidates[0]
        y = candidates[1]
        invert_x = False
        invert_y = True
        message = (
            f"{name.title()} stick candidate\n"
            f"X axis: axis {x[1]}\n"
            f"Y axis: axis {y[1]}\n"
            f"Y inverted: {str(invert_y).lower()}\n"
            f"Center: X {x[4]}, Y {y[4]}\n"
            f"Range: X {x[2]}..{x[3]}, Y {y[2]}..{y[3]}"
        )
        if confirm_detection(message):
            assigned_axes.update({x[1], y[1]})
            return {
                "x_axis": x[1],
                "y_axis": y[1],
                "invert_x": invert_x,
                "invert_y": invert_y,
                "center_x": x[4],
                "center_y": y[4],
                "min_x": x[2],
                "max_x": x[3],
                "min_y": y[2],
                "max_y": y[3],
                "deadzone": 2500,
            }


def detect_dpad(device: JoystickDevice, assigned_buttons: set[int], assigned_axes: set[int]) -> dict[str, Any]:
    detections = {}
    for key, label in DPAD_DIRECTIONS:
        while True:
            print(f"\nPress {label}.")
            print("Waiting...")
            before = device.snapshot()
            deadline = time.monotonic() + 15.0
            candidate = None
            while time.monotonic() < deadline and candidate is None:
                for event in device.read_events(timeout=0.02):
                    if event.is_button and not event.is_init and event.value == 1:
                        if before.buttons.get(event.number, 0) == 0 and event.number not in assigned_buttons:
                            candidate = ("button", event.number, 1)
                            break
                    if event.is_axis and not event.is_init:
                        delta = event.value - before.axes.get(event.number, 0)
                        if abs(delta) > AXIS_DETECTION_THRESHOLD and event.number not in assigned_axes:
                            candidate = ("axis", event.number, event.value)
                            break
            if candidate is None:
                print("No d-pad input was detected.")
                continue
            if confirm_detection(f"Detected: {candidate[0]} {candidate[1]} value {candidate[2]}"):
                detections[key] = candidate
                break
    button_numbers = [value[1] for value in detections.values() if value[0] == "button"]
    axis_values = [value for value in detections.values() if value[0] == "axis"]
    if len(button_numbers) == 4:
        assigned_buttons.update(button_numbers)
        return {
            "type": "button",
            "up": detections["up"][1],
            "right": detections["right"][1],
            "down": detections["down"][1],
            "left": detections["left"][1],
        }
    if len(axis_values) == 4:
        x_axis = detections["right"][1]
        y_axis = detections["down"][1]
        assigned_axes.update({x_axis, y_axis})
        return {
            "type": "axis",
            "x_axis": x_axis,
            "y_axis": y_axis,
            "left_value": detections["left"][2],
            "right_value": detections["right"][2],
            "up_value": detections["up"][2],
            "down_value": detections["down"][2],
        }
    return {"type": "mixed", **{key: list(value) for key, value in detections.items()}}


def build_profile(device: JoystickDevice, idle: JoyState) -> dict[str, Any]:
    assigned_buttons: set[int] = set()
    assigned_axes: set[int] = set()
    profile: dict[str, Any] = {
        "device": {
            "name": device.info.name,
            "path": device.info.path,
            "vendor_id": device.info.vendor_id,
            "product_id": device.info.product_id,
        },
        "buttons": {},
        "triggers": {},
        "sticks": {},
        "dpad": {},
        "idle_axes": idle.axes,
    }
    total_steps = len(BUTTON_NAMES) + 2 + 2 + 4
    step = 1
    for key, label in BUTTON_NAMES:
        print(f"\n{step} / {total_steps}")
        profile["buttons"][key] = detect_button(device, f"Press {label}.", assigned_buttons)
        step += 1
    for key, label in (("r2", "R2"), ("l2", "L2")):
        print(f"\n{step} / {total_steps}")
        profile["triggers"][key] = detect_trigger(device, label, assigned_buttons, assigned_axes)
        step += 1
    for key, label in (("left", "left"), ("right", "right")):
        print(f"\n{step} / {total_steps}")
        profile["sticks"][key] = detect_stick(device, label, assigned_axes)
        step += 1
    print(f"\n{step} / {total_steps}")
    profile["dpad"] = detect_dpad(device, assigned_buttons, assigned_axes)
    return profile


def to_yaml(value: Any, indent: int = 0) -> str:
    spaces = " " * indent
    if isinstance(value, dict):
        lines = []
        for key, child in value.items():
            if isinstance(child, (dict, list)):
                lines.append(f"{spaces}{key}:")
                lines.append(to_yaml(child, indent + 2))
            else:
                lines.append(f"{spaces}{key}: {format_scalar(child)}")
        return "\n".join(lines)
    if isinstance(value, list):
        lines = []
        for child in value:
            if isinstance(child, (dict, list)):
                lines.append(f"{spaces}-")
                lines.append(to_yaml(child, indent + 2))
            else:
                lines.append(f"{spaces}- {format_scalar(child)}")
        return "\n".join(lines)
    return f"{spaces}{format_scalar(value)}"


def format_scalar(value: Any) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if value is None:
        return "null"
    if isinstance(value, (int, float)):
        return str(value)
    text = str(value)
    if text == "" or any(char in text for char in ":#[]{}&,!*|>'\"%@`"):
        return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'
    return text


def save_yaml(path: str, data: dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as output:
        output.write(to_yaml(data))
        output.write("\n")


def load_simple_yaml(path: str) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load profiles for tester mode.") from exc
    with open(path, encoding="utf-8") as input_file:
        return yaml.safe_load(input_file)


def teleop_cmd_yaml(profile: dict[str, Any]) -> dict[str, Any]:
    sticks = profile["sticks"]
    triggers = profile["triggers"]
    buttons = profile["buttons"]
    return {
        "teleop_cmd_node": {
            "ros__parameters": {
                "steering_axis": sticks["left"]["x_axis"],
                "throttle_axis": triggers["r2"]["axis"],
                "reverse_axis": triggers["l2"]["axis"],
                "brake_button": buttons.get("circle", -1),
                "deadman_button": buttons.get("l1", -1),
                "steering_scale": -1.0 if sticks["left"].get("invert_x") else 1.0,
                "throttle_scale": 1.0,
                "reverse_scale": 1.0,
                "brake_value": 1.0,
                "deadzone": 0.05,
                "trigger_min": -1.0,
                "trigger_max": 1.0,
                "throttle_trigger_min": -1.0,
                "throttle_trigger_max": 1.0,
                "throttle_trigger_inverted": triggers["r2"].get("inverted", False),
                "reverse_trigger_min": -1.0,
                "reverse_trigger_max": 1.0,
                "reverse_trigger_inverted": triggers["l2"].get("inverted", False),
            }
        }
    }


def button_mapping_yaml(profile: dict[str, Any]) -> dict[str, Any]:
    buttons = profile["buttons"]
    return {
        "teleop_button_manager_node": {
            "ros__parameters": {
                "auto_button": buttons.get("triangle", -1),
                "manual_button": buttons.get("cross", -1),
                "stop_button": buttons.get("circle", -1),
                "back_button": buttons.get("share", -1),
                "bag_start_button": buttons.get("r1", -1),
                "bag_stop_button": buttons.get("l1", -1),
                "hold_time_s": 1.0,
            }
        }
    }


def normalize_axis(raw: int, deadzone: int = 2500, inverted: bool = False) -> float:
    if abs(raw) < deadzone:
        return 0.0
    value = max(-1.0, min(1.0, raw / float(AXIS_MAX)))
    return -value if inverted else value


def normalize_trigger(raw: int, trigger: dict[str, Any]) -> float:
    min_value = float(trigger.get("min", AXIS_MIN))
    max_value = float(trigger.get("max", AXIS_MAX))
    if max_value == min_value:
        return 0.0
    if trigger.get("inverted", False):
        raw = int(max_value - (raw - min_value))
    value = (raw - min_value) / (max_value - min_value)
    return max(0.0, min(1.0, value))


def bar(value: float, width: int = 10) -> str:
    filled = int(round(max(0.0, min(1.0, value)) * width))
    return "#" * filled + "." * (width - filled)


def tester(profile_path: str, device_path: str | None, raw: bool) -> None:
    profile = load_simple_yaml(profile_path)
    device = select_device(device_path or profile.get("device", {}).get("path"))
    try:
        print("\nPress Ctrl-C to exit.")
        while True:
            device.read_events(timeout=0.05)
            os.system("clear")
            print(profile.get("device", {}).get("name", device.info.name))
            print("=" * 42)
            for name in ("left", "right"):
                stick = profile["sticks"].get(name, {})
                x_raw = device.state.axes.get(stick.get("x_axis", -1), 0)
                y_raw = device.state.axes.get(stick.get("y_axis", -1), 0)
                if raw:
                    print(f"{name.title()} stick  X:{x_raw:6d}  Y:{y_raw:6d}")
                else:
                    x = normalize_axis(x_raw, stick.get("deadzone", 2500), stick.get("invert_x", False))
                    y = normalize_axis(y_raw, stick.get("deadzone", 2500), stick.get("invert_y", False))
                    print(f"{name.title()} stick  X:{x: .3f}  Y:{y: .3f}")
            print()
            for name in ("l2", "r2"):
                trigger = profile["triggers"].get(name, {})
                raw_value = device.state.axes.get(trigger.get("axis", -1), 0)
                value = normalize_trigger(raw_value, trigger)
                if raw:
                    print(f"{name.upper()}: raw {raw_value:6d}  [{bar(value)}] {value:.3f}")
                else:
                    print(f"{name.upper()}: [{bar(value)}] {value:.3f}")
            print()
            button_line = []
            for key in ("cross", "circle", "square", "triangle", "l1", "r1"):
                button = profile["buttons"].get(key, -1)
                state = device.state.buttons.get(button, 0)
                button_line.append(f"{key}:{'ON' if state else 'off'}")
            print("  ".join(button_line))
    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        device.close()


def calibrate(args: argparse.Namespace) -> None:
    device = select_device(args.device)
    try:
        idle = observe_idle(device, args.idle_seconds)
        wait_for_enter()
        profile = build_profile(device, idle)
        save_yaml(args.profile, profile)
        print(f"\nSaved profile: {args.profile}")
        if args.teleop_cmd:
            save_yaml(args.teleop_cmd, teleop_cmd_yaml(profile))
            print(f"Saved teleop cmd parameters: {args.teleop_cmd}")
        if args.button_mapping:
            save_yaml(args.button_mapping, button_mapping_yaml(profile))
            print(f"Saved button mapping parameters: {args.button_mapping}")
    finally:
        device.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    calibrate_parser = subparsers.add_parser("calibrate", help="run the interactive calibration wizard")
    calibrate_parser.add_argument("--device", help="device path, for example /dev/input/js0")
    calibrate_parser.add_argument("--idle-seconds", type=float, default=2.0)
    calibrate_parser.add_argument(
        "--profile",
        default="ros2_ws/src/tool/jetpilot_teleop_tools/config/joy_profile.yaml",
        help="output full controller profile YAML",
    )
    calibrate_parser.add_argument("--teleop-cmd", help="optional teleop_cmd_node parameter YAML output")
    calibrate_parser.add_argument("--button-mapping", help="optional teleop_button_manager_node parameter YAML output")
    calibrate_parser.set_defaults(func=calibrate)

    tester_parser = subparsers.add_parser("test", help="show a terminal Joy Tester using a saved profile")
    tester_parser.add_argument("--profile", required=True)
    tester_parser.add_argument("--device", help="device path, for example /dev/input/js0")
    tester_parser.add_argument("--raw", action="store_true", help="show raw axis values as well as normalized triggers")
    tester_parser.set_defaults(func=lambda args: tester(args.profile, args.device, args.raw))

    args = parser.parse_args()
    try:
        args.func(args)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            termios.tcflush(sys.stdin, termios.TCIFLUSH)
        except termios.error:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
