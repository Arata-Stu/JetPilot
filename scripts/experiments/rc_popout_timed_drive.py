#!/usr/bin/env python3
"""One bounded JPBB AUTO run; invoked by rc_popout_experiment.sh.

Scheduling uses monotonic wall time, never sensor/ROS simulated time.
ROS imports are deliberately deferred so validation/tests work off-vehicle.
"""
import argparse
import json
import math
import signal
import time


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--throttle', type=float, required=True)
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--brake', type=float, required=True)
    parser.add_argument('--brake-duration', type=float, default=1.0)
    parser.add_argument('--steering', type=float, default=0.0)
    parser.add_argument('--label', default='')
    parser.add_argument('--log')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--validate', action='store_true')
    args = parser.parse_args(argv)
    for name, low, high in [('throttle', 0, 1), ('brake', 0, 1),
                            ('steering', -1, 1), ('duration', 0, 60),
                            ('brake_duration', 0, 10)]:
        value = getattr(args, name)
        if not math.isfinite(value) or not low <= value <= high:
            parser.error(f'{name} must be finite and within [{low}, {high}]')
    if args.duration <= 0 or args.brake_duration <= 0:
        parser.error('duration and brake-duration must be positive')
    return args


def command_at(args, elapsed):
    if elapsed < args.duration:
        return 'drive', args.throttle, 0.0
    if elapsed < args.duration + args.brake_duration:
        return 'brake', 0.0, args.brake
    return 'stop', 0.0, 0.0


def run(args):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from diagnostic_msgs.msg import DiagnosticArray
    from jetpilot_msgs.msg import ControlCommand, OperationModeRequest, OperationModeState, BagStatus

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('rc_popout_timed_drive')
    command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(ControlCommand, '/auto/control_cmd', command_qos)
    mode_pub = node.create_publisher(OperationModeRequest, '/operation_mode/request', 10)
    latest = {}
    interrupted = False
    owned = False

    def receive(key, value):
        latest[key] = (value, time.monotonic())

    def diagnostics(msg):
        for status in msg.status:
            if status.name == 'jetpilot_bridge_interface':
                receive('bridge', status)

    node.create_subscription(DiagnosticArray, '/diagnostics', diagnostics, 10)
    node.create_subscription(OperationModeState, '/operation_mode/state',
                             lambda msg: receive('mode', msg.mode), state_qos)
    node.create_subscription(BagStatus, '/bag/status', lambda msg: receive('bag', msg), 10)

    def interrupt(signum, frame):
        nonlocal interrupted
        interrupted = True

    previous_signals = {s: signal.signal(s, interrupt) for s in (signal.SIGINT, signal.SIGTERM)}

    def log(phase, details=None):
        print(f'[ego] {phase}', flush=True)
        if args.log:
            with open(args.log, 'a', encoding='utf-8') as stream:
                stream.write(json.dumps(dict(
                    phase=phase, details=details, recording_label=args.label, wall_time_ns=time.time_ns(),
                    monotonic_s=time.monotonic(), settings=vars(args),
                    semantics='host_command_timing_not_measured_motion',
                ), ensure_ascii=False) + '\n')

    def command(throttle=0.0, brake=0.0):
        msg = ControlCommand()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.steering = args.steering if throttle or brake else 0.0
        msg.throttle, msg.brake, msg.reverse = throttle, brake, 0.0
        pub.publish(msg)

    def mode(value):
        msg = OperationModeRequest()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.mode, msg.source = value, 'rc_popout_timed_drive'
        mode_pub.publish(msg)

    def fresh(key, limit):
        value, stamp = latest.get(key, (None, -math.inf))
        return value if time.monotonic() - stamp < limit else None

    def ready(armed=False):
        status = fresh('bridge', 1.5)
        bag = fresh('bag', 2.5)
        if status is None or status.level != 0:
            return False
        values = {item.key: item.value for item in status.values}
        return (values.get('selector') == 'HOST'
                and values.get('status_fresh') == 'true'
                and (not armed or values.get('host_arm_state') == 'ARMED')
                and bag is not None and bag.recording
                and args.label in bag.current_uri)

    def exclusive():
        return (node.count_publishers('/auto/control_cmd') == 1
                and node.count_subscribers('/auto/control_cmd') >= 1
                and mode_pub.get_subscription_count() >= 1)

    def failure_details(expected_mode, armed=False):
        rows = []

        def row(ok, name, detail):
            rows.append(f"  [{'OK' if ok else 'NG'}] {name}: {detail}")

        def observed(key, limit):
            if key not in latest:
                return None, '未受信'
            value, stamp = latest[key]
            age = time.monotonic() - stamp
            return value, f'最終受信から{age:.2f}s（期限{limit:g}s）'

        names = {1: 'AUTO', 2: 'MANUAL', 3: 'STOP', 4: 'PROPO'}
        value, age = observed('mode', 1.5)
        row(fresh('mode', 1.5) == expected_mode, '/operation_mode/state',
            f'{names.get(value, value)} / 必要={names[expected_mode]} / {age}')
        status, age = observed('bridge', 1.5)
        row(fresh('bridge', 1.5) is not None, 'JPBB診断の受信', age)
        if status is not None:
            values = {item.key: item.value for item in status.values}
            row(status.level == 0, 'JPBB診断level',
                f'{status.level} / {getattr(status, "message", "")} / fault_bits={values.get("fault_bits", "不明")}')
            row(values.get('selector') == 'HOST', 'CH3 selector', values.get('selector', '不明'))
            row(values.get('status_fresh') == 'true', '基板status_fresh', values.get('status_fresh', '不明'))
            if armed:
                row(values.get('host_arm_state') == 'ARMED', 'host_arm_state', values.get('host_arm_state', '不明'))
        bag, age = observed('bag', 2.5)
        row(fresh('bag', 2.5) is not None, '/bag/statusの受信', age)
        if bag is not None:
            row(bag.recording, '記録状態', f'recording={bag.recording}')
            row(args.label in bag.current_uri, '試行labelと記録先の一致',
                f'expected_label={args.label!r} / current_uri={bag.current_uri!r}')
        publishers = node.count_publishers('/auto/control_cmd')
        subscribers = node.count_subscribers('/auto/control_cmd')
        mode_subscribers = mode_pub.get_subscription_count()
        row(publishers == 1, 'AUTO publisher数', f'{publishers}（必要=この補助の1件のみ）')
        row(subscribers >= 1, 'AUTO subscriber数', f'{subscribers}（必要>=1）')
        row(mode_subscribers >= 1, 'モード要求subscriber数', f'{mode_subscribers}（必要>=1）')
        if interrupted:
            rows.append('  [NG] 中断信号を受信しました')
        return '\n'.join(rows)

    try:
        # Do not send even neutral commands until discovery and STOP checks pass.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not interrupted:
            rclpy.spin_once(node, timeout_sec=0.02)
            if ready() and fresh('mode', 1.5) == OperationModeState.STOP and exclusive():
                break
        else:
            details = failure_details(OperationModeState.STOP)
            log('preflight_failed', details)
            raise RuntimeError('開始不可（自車指令は未送信）:\n' + details)
        owned = True
        command()
        arm_started = time.monotonic()
        mode(OperationModeRequest.AUTO)  # One request only: never re-arm after an override.
        log('arming_neutral')
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not interrupted:
            command()
            rclpy.spin_once(node, timeout_sec=0.02)
            if (ready(armed=True) and latest['bridge'][1] >= arm_started
                    and fresh('mode', 1.5) == OperationModeState.AUTO):
                break
        else:
            raise RuntimeError('JPBBのAUTOアームが完了しませんでした:\n'
                               + failure_details(OperationModeState.AUTO, armed=True))
        start = time.monotonic()
        previous_phase = None
        previous_tick = start
        while True:
            now = time.monotonic()
            if interrupted:
                raise RuntimeError('信号による走行中断')
            if now - previous_tick > 0.15:
                raise RuntimeError('送信周期の遅延による中断')
            previous_tick = now
            if not exclusive() or not ready(armed=True) or fresh('mode', 1.5) != OperationModeState.AUTO:
                raise RuntimeError('記録・JPBB・操作モード・publisherの変化により中断')
            phase, throttle, brake = command_at(args, now - start)
            command(throttle, brake)
            if phase != previous_phase:
                log(phase)
                previous_phase = phase
            if phase == 'stop':
                break
            rclpy.spin_once(node, timeout_sec=0.02)
            # spin_once can return immediately on incoming data; cap publication at 50 Hz.
            time.sleep(max(0.0, 0.02 - (time.monotonic() - now)))
    finally:
        if owned:
            # STOP is neutral in the existing mux. Emergency abort does not claim active braking.
            for _ in range(10):
                command()
                mode(OperationModeRequest.STOP)
                rclpy.spin_once(node, timeout_sec=0.02)
                time.sleep(0.02)
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)
        node.destroy_node()
        rclpy.shutdown()


def main():
    args = arguments()
    if args.validate:
        return
    if args.dry_run:
        for elapsed in (0, args.duration, args.duration + args.brake_duration):
            print(f'[dry-run] t={elapsed:g}s ego={command_at(args, elapsed)} steering={args.steering}')
        return
    if not args.label:
        raise SystemExit('recording --label is required')
    try:
        run(args)
    except (RuntimeError, KeyboardInterrupt) as error:
        raise SystemExit(str(error)) from error


if __name__ == '__main__':
    main()
