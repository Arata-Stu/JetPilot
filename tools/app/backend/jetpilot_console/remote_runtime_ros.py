"""Short-lived ROS requests, executed only inside the Jetson container."""
import json
import sys
import time


def execute(request):
    import rclpy
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import GetParameters, SetParametersAtomically
    from jetpilot_msgs.msg import BagStatus

    rclpy.init(args=[])
    node = rclpy.create_node('jetpilot_web_request_' + str(__import__('os').getpid()))
    try:
        if request['action'] == 'bag-status':
            received = []
            sub = node.create_subscription(BagStatus, '/bag/status', received.append, 10)
            deadline = time.monotonic() + 3
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
            if not received:
                return {'bag': {'state': 'unknown', 'message': '/bag/statusの応答がありません'}}
            msg = received[-1]
            return {'bag': {'state': 'recording' if msg.recording else 'idle',
                            'current_uri': msg.current_uri, 'message': msg.message, 'last_event': msg.last_event}}

        def call(service_type, suffix, message):
            client = node.create_client(service_type, request['node'] + '/' + suffix)
            if not client.wait_for_service(timeout_sec=2):
                raise RuntimeError('ノードのパラメータサービスが応答しません')
            future = client.call_async(message)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
            if not future.done():
                raise RuntimeError('応答がありません。適用結果は未確認です。現在値を再取得してください。')
            return future.result()

        if request['action'] in ('camera-get', 'camera-set'):
            import re
            rgb = request['stream'] == 'rgb'
            profile = 'rgb_camera.color_profile' if rgb else 'depth_module.infra_profile'
            enables = ['enable_color'] if rgb else ['enable_infra1', 'enable_infra2']
            def get_camera():
                msg = GetParameters.Request()
                msg.names = [profile] + enables
                return call(GetParameters, 'get_parameters', msg).values
            def set_camera(values):
                msg = SetParametersAtomically.Request()
                msg.parameters = [Parameter(key, value=value).to_parameter_msg() for key, value in values.items()]
                result = call(SetParametersAtomically, 'set_parameters_atomically', msg).result
                if not result.successful:
                    raise RuntimeError(result.reason)
            values = get_camera()
            if values[0].type != 4 or any(value.type != 1 for value in values[1:]):
                raise RuntimeError('カメラのプロファイル・有効状態を取得できません')
            old_profile = values[0].string_value
            enabled = {key: value.bool_value for key, value in zip(enables, values[1:])}
            if request['action'] == 'camera-set':
                shape = re.fullmatch(r'\s*(\d+)\s*[x,]\s*(\d+)\s*[x,]\s*(\d+)\s*', old_profile)
                if not shape:
                    raise RuntimeError('現在の解像度を取得できません')
                new_profile = f'{shape[1]}x{shape[2]}x{request["fps"]}'
                try:
                    set_camera({key: False for key in enables})
                    set_camera({profile: new_profile})
                    set_camera(enabled)
                except Exception as error:
                    try:
                        set_camera({profile: old_profile})
                        set_camera(enabled)
                    except Exception:
                        raise RuntimeError(f'Hz変更と復元に失敗しました。カメラの再起動が必要です: {error}')
                    raise RuntimeError(f'Hz変更に失敗し、元の設定に復元しました: {error}')
                values = get_camera()
            return {'camera': {'stream': request['stream'], 'profile': values[0].string_value,
                                'enabled': any(value.bool_value for value in values[1:])},
                    'message': 'カメラ設定を取得しました' if request['action'] == 'camera-get' else 'カメラのHz変更を受理しました。映像の再開を確認してください。'}

        capability = GetParameters.Request()
        capability.names = ['dynamic_tuning_parameters']
        supported = call(GetParameters, 'get_parameters', capability).values[0]
        if supported.type != 9 or request['parameter'] not in supported.string_array_value:
            raise RuntimeError('このノードのビルドは動的調整に未対応です。更新・再ビルド後にbringupを起動してください。')

        if request['action'] == 'param-set':
            msg = SetParametersAtomically.Request()
            msg.parameters = [Parameter(request['parameter'], value=request['value'] if request['parameter'] == 'algorithm' else float(request['value'])).to_parameter_msg()]
            response = call(SetParametersAtomically, 'set_parameters_atomically', msg)
            if not response.result.successful:
                raise RuntimeError(response.result.reason)
        msg = GetParameters.Request()
        msg.names = [request['parameter']]
        response = call(GetParameters, 'get_parameters', msg)
        value = response.values[0]
        if value.type not in (2, 3, 4):
            raise RuntimeError('パラメータが見つかりません')
        return {'parameter': request['parameter'], 'node': request['node'],
                'value': value.string_value if value.type == 4 else value.integer_value if value.type == 2 else value.double_value,
                'message': '現在値を取得しました' if request['action'] == 'param-get' else 'ノードが変更を受理しました'}
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        print(json.dumps(execute(json.loads(sys.argv[1])), ensure_ascii=False))
    except Exception as exc:
        print(json.dumps({'error': str(exc)}, ensure_ascii=False))
