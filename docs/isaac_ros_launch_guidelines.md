# Isaac ROS Launch Guidelines

JetPilot の launch ファイルでは Isaac ROS 4.6 の `isaac_ros_launch_utils`
を使う。特に `ArgumentContainer.add_opaque_function()` の内側では、launch
引数が `LaunchConfiguration` ではなく Python の値へ評価済みになる点に注意する。

## 基本ルール

- `generate_launch_description()` 直下では `args.foo` は `LaunchConfiguration`。
- `args.add_opaque_function()` で呼ばれる関数内では `args.foo` は評価済みの Python 値。
- 評価済み値は `true` / `false` / `5.0` / `1` などが `bool` / `float` / `int` に戻る。
- `lu.include(..., launch_arguments={...})` は Isaac ROS 側で文字列化されるため、Python 値を渡してよい。
- `Node(parameters=[...])` は typed value を受けられるため、`bool` / `int` / `float` を渡してよい。
- `Node(arguments=[...])` と `ExecuteProcess(cmd=[...])` には文字列を渡す。数値の可能性がある値は `str(...)` する。
- opaque function 内では `IfCondition(bool)` を作らず、Python の `if lu.is_true(args.foo):` で分岐する。
- opaque function 外で `str(args.foo)` しない。`LaunchConfiguration` の表示文字列になり、値の置換ではなくなる。

## よい例

opaque function 内で `Node(arguments=...)` に評価済みの float が入り得る場合:

```python
def add_nodes(args: lu.ArgumentContainer):
    return [
        lu.Node(
            package='vslam_map_tools',
            executable='record_vslam_reference_snapshot.py',
            arguments=[
                '--write-interval-sec', str(args.vslam_snapshot_write_interval_s),
            ],
        )
    ]
```

opaque function 内の条件分岐:

```python
def add_nodes(args: lu.ArgumentContainer):
    actions = []
    if lu.is_true(args.run_standalone):
        actions.append(component_container(args.container_name))
    return actions
```

include へ渡す場合:

```python
lu.include(
    'jetpilot_system_launch',
    'launch/localization/vslam.launch.py',
    launch_arguments={
        'use_sim_time': lu.is_true(args.use_sim_time),
        'vslam_enable_visualization': lu.is_true(args.vslam_enable_visualization),
    },
)
```

## 避ける例

opaque function 内で `IfCondition()` に Python bool を渡す:

```python
condition=IfCondition(lu.is_true(args.run_standalone))
```

opaque function 内で `Node(arguments=...)` に float をそのまま渡す:

```python
arguments=['--write-interval-sec', args.vslam_snapshot_write_interval_s]
```

opaque function 外で `LaunchConfiguration` を `str()` する:

```python
str(args.some_launch_argument)
```

## 典型的な症状

- `'float' object is not iterable`
- `'bool' object is not iterable`
- `could not convert string to float: '<launch.substitutions.launch_configuration.LaunchConfiguration object ...>'`

これらは多くの場合、`LaunchConfiguration` / Python 評価済み値 / ROS launch
Action が期待する型の境界をまたいでいる。

## センサー・E2E推論の共有コンテナ

センサーと画像ベースE2E推論の既定コンテナ名は
`multi_sensor_container`とする。実行器には
`rclcpp_components`の`component_container_mt`を使う。

- センサーlaunchがコンテナを所有する場合、推論launchは
  `run_standalone:=false`で同じコンテナへロードする。
- 推論launchだけを起動する場合は`run_standalone:=true`とし、推論側で
  `multi_sensor_container`を作る。
- コンテナ名の`_mt` suffixは付けない。マルチスレッドかどうかは
  executableの選択で表現する。
- 画像の前処理からTensorRTまでにPython/OpenCV/NumPyノードを挟まない。
  GPU上のNITROS tensor経路を維持する。
- TensorRT出力のdecoderもC++ Composable Nodeとし、同じコンテナ内で
  `rclcpp::Subscription<NitrosTensorList>`をintra-processで直接使用する。
- decoderがCUDA bufferを読むときはconsumer用streamを取得し、同じstreamで
  `get_read_handle()`、非同期GPU-to-CPU copy、stream同期を行う。
- 通常のCPUメモリをpublishするカメラでは、GPU入口の転送自体は残る。
  同一コンテナ化だけでカメラ取得時点から完全なzero-copyにはならない。
- camera driverからimage encoderまではrclcpp intra-process、image encoderから
  TensorRTとdecoderまではNITROSを使用する。decoderから通常ROSメッセージを
  publishするための最小限のGPU-to-CPU copyは許容する。

## 参考

- `isaac_ros_launch_utils.core.ArgumentContainer.add_opaque_function()`
- `isaac_ros_launch_utils.core.include()`
- Isaac ROS 4.6 `isaac_mapping_ros/launch/algorithms/vslam.launch.py`
- Isaac ROS 4.6 `isaac_mapping_ros/launch/localize_realsense.launch.py`
