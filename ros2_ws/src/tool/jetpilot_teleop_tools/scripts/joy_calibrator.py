#!/usr/bin/env python3
"""Interactive joystick profile calibrator and terminal tester."""

from __future__ import annotations

import argparse
import array
import fcntl
import glob
import html
import json
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
    ("cross", "x / cross", "× / cross"),
    ("circle", "circle", "○ / circle"),
    ("triangle", "triangle", "△ / triangle"),
    ("square", "square", "□ / square"),
    ("l1", "L1", "L1"),
    ("r1", "R1", "R1"),
    ("share", "share", "Share / 共有"),
    ("options", "options", "Options"),
    ("ps", "PS", "PS"),
    ("l3", "L3", "L3"),
    ("r3", "R3", "R3"),
]
DPAD_DIRECTIONS = [
    ("up", "d-pad up", "十字キー 上 / d-pad up"),
    ("right", "d-pad right", "十字キー 右 / d-pad right"),
    ("down", "d-pad down", "十字キー 下 / d-pad down"),
    ("left", "d-pad left", "十字キー 左 / d-pad left"),
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
        raise RuntimeError("/dev/input/js* デバイスが見つかりません。 / No /dev/input/js* devices were found.")
    print("\n検出されたデバイス / Detected devices\n")
    for index, info in enumerate(devices):
        print(f"[{index}] {info.name}")
        print(f"    {info.path}")
    while True:
        value = input("\nデバイス番号を選択 / Select device number: ").strip()
        try:
            return JoystickDevice(devices[int(value)].path)
        except (ValueError, IndexError):
            print("一覧にある番号を入力してください。 / Please enter one of the listed numbers.")


def wait_for_enter() -> None:
    input("\nEnterで続行 / Press Enter to continue.")


def observe_idle(device: JoystickDevice, duration: float) -> JoyState:
    print("\nコントローラーに触れないでください。 / Do not touch the controller.")
    print("中心値・ノイズ幅を測定しています... / Measuring center values and noise...\n")
    samples: dict[int, list[int]] = {axis: [] for axis in device.state.axes}
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        device.read_events(timeout=0.02)
        for axis, value in device.state.axes.items():
            samples[axis].append(value)
    centers = {axis: int(sum(values) / len(values)) if values else 0 for axis, values in samples.items()}
    print("axis中心候補 / Axis center candidates:")
    for axis, value in centers.items():
        spread = max(samples[axis]) - min(samples[axis]) if samples[axis] else 0
        print(f"  axis {axis}: 中心/center {value}, ノイズ/noise {spread}")
    return JoyState(axes=centers, buttons=dict(device.state.buttons))


def confirm_detection(text: str) -> bool:
    answer = input(f"{text}\nこれでよいですか？ / OK? [Enter: 決定 / accept, r: やり直し / retry] ").strip().lower()
    return answer != "r"


def detect_button(
    device: JoystickDevice,
    prompt: str,
    assigned_buttons: set[int],
    timeout: float = 15.0,
) -> int:
    while True:
        print(f"\n{prompt}")
        print("待機中... / Waiting...")
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
            print("ボタン入力を検出できませんでした。 / No button press was detected.")
            continue
        if confirm_detection(f"検出 / Detected: button {candidate}"):
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
        print(f"\n{name}をゆっくり最後まで押してから離してください。 / Slowly press {name} all the way, then release it.")
        print("3秒間記録します... / Recording for 3 seconds...")
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
        print("\n変化を検出しました / Detected changes")
        for movement, axis, min_value, max_value in axis_candidates[:3]:
            print(f"  axis {axis}: 最小/min {min_value}, 最大/max {max_value}, 変化量/movement {movement}")
        for button in button_candidates[:3]:
            print(f"  button {button}: 0 -> 1 -> 0")
        if not axis_candidates and not button_candidates:
            print("使えるトリガー入力を検出できませんでした。 / No usable trigger movement was detected.")
            continue
        _, axis, min_value, max_value = axis_candidates[0] if axis_candidates else (0, -1, 0, 0)
        button = button_candidates[0] if button_candidates else -1
        initial_value = before.axes.get(axis, 0)
        pressed_value = max(
            axis_samples.get(axis, [initial_value]),
            key=lambda value: abs(value - initial_value),
        )
        inverted = pressed_value < initial_value
        if confirm_detection(
            f"{name} analog / アナログ = axis {axis}\n"
            f"{name} digital / デジタル = button {button}\n"
            f"反転 / inverted = {str(inverted).lower()}"
        ):
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
        print(f"\n{name}スティックを大きく円を描くように回してください。 / Move the {name} stick in a large circle.")
        print("5秒間記録します... / Recording for 5 seconds...")
        before, axis_samples, _ = collect_changes(device, 5.0)
        candidates = []
        for axis, values in axis_samples.items():
            movement = max(values) - min(values)
            if movement > STICK_MOVEMENT_THRESHOLD and axis not in assigned_axes:
                center = before.axes.get(axis, 0)
                candidates.append((movement, axis, min(values), max(values), center))
        candidates.sort(reverse=True)
        if len(candidates) < 2:
            print("このスティック用の2軸を検出できませんでした。 / Could not find two moving axes for this stick.")
            continue
        x = candidates[0]
        y = candidates[1]
        invert_x = False
        invert_y = True
        message = (
            f"{name.title()} stick candidate\n"
            f"X軸 / X axis: axis {x[1]}\n"
            f"Y軸 / Y axis: axis {y[1]}\n"
            f"Y反転 / Y inverted: {str(invert_y).lower()}\n"
            f"中心 / Center: X {x[4]}, Y {y[4]}\n"
            f"範囲 / Range: X {x[2]}..{x[3]}, Y {y[2]}..{y[3]}"
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
    for key, _, label in DPAD_DIRECTIONS:
        while True:
            print(f"\n{label} を押してください。 / Press {label}.")
            print("待機中... / Waiting...")
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
                print("十字キー入力を検出できませんでした。 / No d-pad input was detected.")
                continue
            if confirm_detection(f"検出 / Detected: {candidate[0]} {candidate[1]} value {candidate[2]}"):
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
    for key, _, label in BUTTON_NAMES:
        print(f"\n{step} / {total_steps}")
        profile["buttons"][key] = detect_button(device, f"{label} を押してください。 / Press {label}.", assigned_buttons)
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


def default_html_report_path(profile_path: str) -> str:
    root, _ = os.path.splitext(profile_path)
    return root + ".html"


def default_sibling_path(profile_path: str, filename: str) -> str:
    directory = os.path.dirname(os.path.abspath(profile_path))
    return os.path.join(directory, filename)


def find_editor_template_path() -> str:
    candidates = [
        os.path.join(os.path.dirname(__file__), "joy_profile_editor.html"),
    ]
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(
                os.path.join(prefix, "share", "jetpilot_teleop_tools", "web", "joy_profile_editor.html")
            )
            candidates.append(
                os.path.join(prefix, "lib", "jetpilot_teleop_tools", "joy_profile_editor.html")
            )
    for path in candidates:
        if os.path.exists(path):
            return path
    searched = "\n  ".join(candidates)
    raise RuntimeError(f"joy_profile_editor.html が見つかりません。 / Template not found. Searched:\n  {searched}")


def _save_legacy_html_report(path: str, profile: dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    profile_json = json.dumps(profile, ensure_ascii=False)
    device = profile.get("device", {})
    rows = []
    for name, button in profile.get("buttons", {}).items():
        rows.append(("button", name, f"button {button}", ""))
    for name, trigger in profile.get("triggers", {}).items():
        detail = (
            f"axis {trigger.get('axis', -1)}, button {trigger.get('button', -1)}, "
            f"min {trigger.get('min')}, max {trigger.get('max')}"
        )
        rows.append(("trigger", name, detail, f"inverted: {trigger.get('inverted', False)}"))
    for name, stick in profile.get("sticks", {}).items():
        detail = f"x axis {stick.get('x_axis', -1)}, y axis {stick.get('y_axis', -1)}"
        extra = f"deadzone: {stick.get('deadzone')}, invert_y: {stick.get('invert_y', False)}"
        rows.append(("stick", name, detail, extra))
    dpad = profile.get("dpad", {})
    rows.append(("dpad", dpad.get("type", "unknown"), ", ".join(f"{key}: {value}" for key, value in dpad.items()), ""))
    rows_html = "\n".join(
        "<tr>"
        f"<td>{html.escape(kind)}</td>"
        f"<td>{html.escape(name)}</td>"
        f"<td>{html.escape(detail)}</td>"
        f"<td>{html.escape(extra)}</td>"
        "</tr>"
        for kind, name, detail, extra in rows
    )
    title = f"{device.get('name', 'Controller')} Joy Profile"
    page = f"""<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(title)}</title>
  <style>
    :root {{
      color-scheme: light dark;
      --bg: #f6f7f9;
      --panel: #ffffff;
      --text: #1e242c;
      --muted: #667085;
      --line: #d9dee7;
      --accent: #1f7a8c;
      --accent-2: #b8325f;
    }}
    @media (prefers-color-scheme: dark) {{
      :root {{
        --bg: #14171b;
        --panel: #1d2229;
        --text: #eef2f6;
        --muted: #aab3bf;
        --line: #333b46;
      }}
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      line-height: 1.5;
    }}
    main {{
      width: min(1120px, calc(100% - 32px));
      margin: 0 auto;
      padding: 28px 0 48px;
    }}
    header {{
      display: grid;
      gap: 8px;
      padding: 20px 0;
      border-bottom: 1px solid var(--line);
    }}
    h1 {{ margin: 0; font-size: 28px; letter-spacing: 0; }}
    h2 {{ margin: 0 0 14px; font-size: 18px; letter-spacing: 0; }}
    .muted {{ color: var(--muted); }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 16px;
      margin-top: 20px;
    }}
    section {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 18px;
      min-width: 0;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 14px;
    }}
    th, td {{
      padding: 10px 8px;
      border-bottom: 1px solid var(--line);
      text-align: left;
      vertical-align: top;
      word-break: break-word;
    }}
    th {{ color: var(--muted); font-weight: 600; }}
    .tester {{
      display: grid;
      gap: 14px;
    }}
    .row {{
      display: grid;
      grid-template-columns: 120px 1fr 72px;
      gap: 10px;
      align-items: center;
      font-variant-numeric: tabular-nums;
    }}
    .meter {{
      height: 12px;
      overflow: hidden;
      background: color-mix(in srgb, var(--line), transparent 25%);
      border-radius: 999px;
    }}
    .fill {{
      height: 100%;
      width: 50%;
      background: var(--accent);
      transform-origin: left center;
    }}
    .buttons {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(104px, 1fr));
      gap: 8px;
    }}
    .button-state {{
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 9px 10px;
      display: flex;
      justify-content: space-between;
      gap: 8px;
    }}
    .on {{
      border-color: var(--accent-2);
      background: color-mix(in srgb, var(--accent-2), transparent 84%);
    }}
    pre {{
      max-height: 280px;
      overflow: auto;
      padding: 14px;
      border-radius: 8px;
      background: color-mix(in srgb, var(--line), transparent 70%);
      font-size: 12px;
    }}
    @media (max-width: 820px) {{
      .grid {{ grid-template-columns: 1fr; }}
      .row {{ grid-template-columns: 92px 1fr 64px; }}
    }}
  </style>
</head>
<body>
<main>
  <header>
    <h1>{html.escape(str(device.get('name', 'Controller')))}</h1>
    <div class="muted">
      コントローラー設定レポート / Controller profile report<br>
      path: {html.escape(str(device.get('path', '')))}
      vendor: {html.escape(str(device.get('vendor_id', '')))}
      product: {html.escape(str(device.get('product_id', '')))}
    </div>
  </header>

  <div class="grid">
    <section>
      <h2>割り当て / Mapping</h2>
      <table>
        <thead><tr><th>type</th><th>name</th><th>mapping</th><th>extra</th></tr></thead>
        <tbody>{rows_html}</tbody>
      </table>
    </section>

    <section>
      <h2>Joy Tester</h2>
      <div class="tester" id="tester">
        <div class="muted">コントローラーのボタンを押してください。 / Press any controller button.</div>
      </div>
    </section>

    <section>
      <h2>生プロファイル / Raw Profile</h2>
      <pre id="profile-json"></pre>
    </section>
  </div>
</main>
<script>
const profile = {profile_json};
document.getElementById("profile-json").textContent = JSON.stringify(profile, null, 2);

function axisValue(gamepad, index) {{
  if (index === undefined || index < 0 || index >= gamepad.axes.length) return 0;
  return gamepad.axes[index];
}}

function buttonValue(gamepad, index) {{
  if (index === undefined || index < 0 || index >= gamepad.buttons.length) return false;
  return gamepad.buttons[index].pressed;
}}

function normalizedTrigger(gamepad, trigger) {{
  const axis = axisValue(gamepad, trigger.axis);
  const min = -1;
  const max = 1;
  const raw = trigger.inverted ? max - (axis - min) : axis;
  return Math.max(0, Math.min(1, (raw - min) / (max - min)));
}}

function meterRow(label, value) {{
  const pct = Math.round(((value + 1) / 2) * 100);
  return `<div class="row"><div>${{label}}</div><div class="meter"><div class="fill" style="width:${{pct}}%"></div></div><div>${{value.toFixed(3)}}</div></div>`;
}}

function triggerRow(label, value) {{
  const pct = Math.round(value * 100);
  return `<div class="row"><div>${{label}}</div><div class="meter"><div class="fill" style="width:${{pct}}%"></div></div><div>${{value.toFixed(3)}}</div></div>`;
}}

function render() {{
  const tester = document.getElementById("tester");
  const pads = navigator.getGamepads ? [...navigator.getGamepads()].filter(Boolean) : [];
  if (!pads.length) {{
    tester.innerHTML = '<div class="muted">Gamepad APIでコントローラー待機中... / Waiting for a controller via Gamepad API...</div>';
    requestAnimationFrame(render);
    return;
  }}
  const pad = pads[0];
  const left = profile.sticks?.left || {{}};
  const right = profile.sticks?.right || {{}};
  const r2 = profile.triggers?.r2 || {{}};
  const l2 = profile.triggers?.l2 || {{}};
  const buttons = profile.buttons || {{}};
  const buttonHtml = Object.entries(buttons).map(([name, index]) => {{
    const on = buttonValue(pad, index);
    return `<div class="button-state ${{on ? 'on' : ''}}"><span>${{name}}</span><strong>${{on ? 'ON' : 'off'}}</strong></div>`;
  }}).join("");
  tester.innerHTML = `
    <div class="muted">${{pad.id}}</div>
    ${{meterRow('Left X', axisValue(pad, left.x_axis))}}
    ${{meterRow('Left Y', axisValue(pad, left.y_axis))}}
    ${{meterRow('Right X', axisValue(pad, right.x_axis))}}
    ${{meterRow('Right Y', axisValue(pad, right.y_axis))}}
    ${{triggerRow('L2', normalizedTrigger(pad, l2))}}
    ${{triggerRow('R2', normalizedTrigger(pad, r2))}}
    <div class="buttons">${{buttonHtml}}</div>
  `;
  requestAnimationFrame(render);
}}
render();
</script>
</body>
</html>
"""
    with open(path, "w", encoding="utf-8") as output:
        output.write(page)


def save_html_report(path: str, profile: dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    template_path = find_editor_template_path()
    with open(template_path, encoding="utf-8") as template_file:
        page = template_file.read()
    embedded_profile = json.dumps(profile, ensure_ascii=False)
    page = page.replace("__EMBEDDED_PROFILE_JSON__", embedded_profile)
    with open(path, "w", encoding="utf-8") as output:
        output.write(page)


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
                "steer_offset_inc_button": profile.get("dpad", {}).get("right", -1),
                "steer_offset_dec_button": profile.get("dpad", {}).get("left", -1),
                "hold_time_s": 0.1,
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
        print(f"\nプロファイルを保存しました / Saved profile: {args.profile}")
        if not args.no_teleop_cmd:
            teleop_cmd_path = args.teleop_cmd or default_sibling_path(args.profile, "teleop_cmd.param.yaml")
            save_yaml(teleop_cmd_path, teleop_cmd_yaml(profile))
            print(f"teleop cmd設定を保存しました / Saved teleop cmd parameters: {teleop_cmd_path}")
        if not args.no_button_mapping:
            button_mapping_path = args.button_mapping or default_sibling_path(
                args.profile, "joy_button_mapping.param.yaml"
            )
            save_yaml(button_mapping_path, button_mapping_yaml(profile))
            print(f"ボタン設定を保存しました / Saved button mapping parameters: {button_mapping_path}")
        if not args.no_html_report:
            html_report = args.html_report or default_html_report_path(args.profile)
            save_html_report(html_report, profile)
            print(f"HTMLレポートを保存しました / Saved HTML report: {html_report}")
    finally:
        device.close()


def report(args: argparse.Namespace) -> None:
    profile = load_simple_yaml(args.profile)
    output = args.output or default_html_report_path(args.profile)
    save_html_report(output, profile)
    print(f"HTMLレポートを保存しました / Saved HTML report: {output}")


def ui(args: argparse.Namespace) -> None:
    output = args.output
    if args.profile:
        profile = load_simple_yaml(args.profile)
    else:
        profile = {
            "device": {
                "name": "DualShock4",
                "path": "/dev/input/js0",
                "vendor_id": "054c",
                "product_id": "09cc",
            },
            "buttons": {},
            "triggers": {},
            "sticks": {},
            "dpad": {},
            "idle_axes": {},
        }
    save_html_report(output, profile)
    print(f"UIを保存しました / Saved UI: {output}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    calibrate_parser = subparsers.add_parser("calibrate", help="run the interactive calibration wizard")
    calibrate_parser.add_argument("--device", help="device path, for example /dev/input/js0")
    calibrate_parser.add_argument("--idle-seconds", type=float, default=2.0)
    calibrate_parser.add_argument(
        "--profile",
        default="joy_profile.yaml",
        help="output full controller profile YAML",
    )
    calibrate_parser.add_argument(
        "--teleop-cmd",
        help="teleop_cmd_node parameter YAML output. Defaults to teleop_cmd.param.yaml next to --profile",
    )
    calibrate_parser.add_argument(
        "--button-mapping",
        help="teleop_button_manager_node parameter YAML output. Defaults to joy_button_mapping.param.yaml next to --profile",
    )
    calibrate_parser.add_argument("--no-teleop-cmd", action="store_true", help="do not generate teleop_cmd YAML")
    calibrate_parser.add_argument(
        "--no-button-mapping", action="store_true", help="do not generate teleop button mapping YAML"
    )
    calibrate_parser.add_argument("--html-report", help="optional HTML report output")
    calibrate_parser.add_argument("--no-html-report", action="store_true", help="do not generate an HTML report")
    calibrate_parser.set_defaults(func=calibrate)

    tester_parser = subparsers.add_parser("test", help="show a terminal Joy Tester using a saved profile")
    tester_parser.add_argument("--profile", required=True)
    tester_parser.add_argument("--device", help="device path, for example /dev/input/js0")
    tester_parser.add_argument("--raw", action="store_true", help="show raw axis values as well as normalized triggers")
    tester_parser.set_defaults(func=lambda args: tester(args.profile, args.device, args.raw))

    report_parser = subparsers.add_parser("report", help="generate an HTML report from a saved profile")
    report_parser.add_argument("--profile", required=True)
    report_parser.add_argument("--output", help="HTML report output path")
    report_parser.set_defaults(func=report)

    ui_parser = subparsers.add_parser("ui", help="generate a standalone YAML editor UI")
    ui_parser.add_argument("--profile", help="optional profile YAML to embed")
    ui_parser.add_argument(
        "--output",
        default="joy_profile_editor.html",
        help="HTML UI output path",
    )
    ui_parser.set_defaults(func=ui)

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
