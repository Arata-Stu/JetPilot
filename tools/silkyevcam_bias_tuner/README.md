# SilkyEvCam Bias Tuner

SilkyEvCam のイベント画像とイベントレートを見ながらバイアスを調整する、
ROS 非依存の C++ / OpenCV ツールです。旧 `c++/silkyevcam_bias_tuner` から移動しました。

## ビルド

SilkyEvCam 対応 Metavision SDK / HAL、OpenCV（GUI 対応）、CMake、C++17 が必要です。
JetPilot の `/usr/local` にインストールした SDK を使う場合、project root で実行します。

```bash
cmake -S tools/silkyevcam_bias_tuner -B tools/silkyevcam_bias_tuner/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DMetavisionSDK_DIR=/usr/local/share/cmake/MetavisionSDK \
  -DMetavisionHAL_DIR=/usr/local/share/cmake/MetavisionHAL
cmake --build tools/silkyevcam_bias_tuner/build -j2
```

## 調整・保存

ROS ドライバーなど、同じカメラを使用しているアプリケーションを終了してから起動します。
最初に見つかったカメラを開き、保存先は起動時のカレントディレクトリになります。
次の例では、生成ファイルを `record/silkyevcam_bias` にまとめます。

```bash
# project root から実行
mkdir -p record/silkyevcam_bias
cd record/silkyevcam_bias
../../tools/silkyevcam_bias_tuner/build/silkyevcam_bias_tuner
```

| 操作 | 動作 |
| --- | --- |
| スライダー | バイアスを手動調整 |
| `a` | 背景ノイズの自動調整。成功すると `silkyevcam_autotuned.bias` を保存 |
| `s` | 現在の値を `silkyevcam_custom.bias` に保存 |
| `r` | カメラの現在値を GUI に再取得（ファイル読み込みではありません） |
| `q` | 終了 |

自動調整は背景ノイズを測るため、静止した対象・カメラで実行します。
同名ファイルは次の保存時に上書きされます。

## ROS 2 で使用

運用用の設定は
[`jetpilot_system_launch/config/sensing/silkyevcam`](../../ros2_ws/src/launch/jetpilot_system_launch/config/sensing/silkyevcam/README.md)
で管理します。旧 `ros2_ws/data.bias` と `ros2_ws/settings.json` もそこへ移動しています。
`record/silkyevcam_bias/` は調整中の出力先とし、使用する `.bias` を運用用ディレクトリへ
コピーして再ビルドしてください。JSON は設定全体の保管用で、現在のドライバーへは
`.bias` を渡します。

`openeb_ros2` と `jetpilot_system_launch` を変更後に再ビルドし、workspace の
`install/setup.bash` を source してください。チューナーを終了してから起動します。

```bash
ros2 launch openeb_ros2 pipeline.launch.py \
  bias_file:=/absolute/path/to/silkyevcam_custom.bias
```

`driver.launch.py`、`composed.launch.py` でも同じ `bias_file` 引数を使用できます。
JetPilot のセンサー launch を直接使う場合は `silky_evcam_bias_file`、
bringup 経由では `sensor_kit_silky_evcam_bias_file` を指定します。

```bash
./scripts/bringup.sh --preset calibration --vehicle jpbb --sensor-kit realsense-silky \
  sensor_kit_silky_evcam_bias_file:=/workspaces/record/silkyevcam_bias/silkyevcam_custom.bias
```

`realsense-silky-flir` でも同じ引数を使用できます。
Docker 内で起動する場合は、コンテナから見えるファイルの絶対パスを指定してください。
自動調整結果を使う場合はファイル名を `silkyevcam_autotuned.bias` に変更します。

ドライバーは記録・配信開始前に SDK の `I_LL_Biases::load_from_file()` で適用します。
空文字（既定値）は読み込みを省略し、従来のカメラ設定を使用します。
SDK が読み込みエラーを報告すると、対象パスを含むエラーで起動を中止します。
`bias_file` は起動時のみのパラメーターです。変更したファイルを適用するには再起動します。

`openeb_ros2` は `packages.repos` で管理する別 Git リポジトリです。
ドライバー側の変更はそのリポジトリでも管理する必要があります。
