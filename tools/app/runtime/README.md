# 実車調整モード

Notebookの **Maps → 対象Mapを開く → 実車調整モード** から使います。
画像・動画・点群の継続転送は行いません。地図はNotebookにある形状データを描画し、
Jetsonからはmap座標の自己位置・速度・操作モード・適用版だけを約1 Hzで取得します。
転送はSSHの既存接続を再利用し、走行ライン全体はプレビュー・適用・適用版変更時だけ送ります。

## TUIからまとめて起動する（推奨）

更新後、Jetsonで`jetpilot_system_launch`のインストールを更新してください。

```sh
cd /workspaces/ros2_ws
colcon build --packages-select jetpilot_system_launch --symlink-install
source /workspaces/ros2_ws/install/setup.bash
/workspaces/scripts/bringup.sh
```

TUIで`tuning`を選ぶと、センサー・自己位置推定・操作系・車両・調整サービスとcontrollerを
まとめて起動します。車両プロファイルの既定は`jpbb`です。ほかの車両はTUIまたは`--vehicle`で選べます。
自己位置推定の初期化は既存TUIと同じ手順で行います。走行ラインはNotebookのUIから選ぶため、
このpresetではTUIの走行ライン選択を省略します。通常走行用のplanning/controllerとの重複起動は拒否します。

```sh
/workspaces/scripts/bringup.sh tuning --map /workspaces/map/course_a
```

Mapは`cuvgl_map/`または`cuvslam_map/`が直接入っているフォルダーを選んでください。
日時付きサブフォルダーに生成物がある場合は、親フォルダーではなくそのサブフォルダーを指定します。
この一括起動を使う場合、別に起動したセンサー・localization・調整launchは先に終了してください。

## 自己位置が表示されない場合

HTTP/SSHの「接続中」と、自己位置を取得できていることは別です。UIにTFの取得失敗理由、
TF座標、ROS_DOMAIN_ID、各topicの配信元数を表示します。`localized`なのに位置がない場合は
`map → base_link`のTFと時刻差を確認してください。TFは0.5秒以内のものだけ採用します。
「走行許可失効」は適用・走行条件の表示で、位置表示を直接無効にするものではありません。
表示用の接続はMANUALなどでも可能ですが、適用・許可の再取得は引き続きSTOP・停車確認が必要です。

## Jetsonの起動

このディレクトリと `tools/app/backend/jetpilot_console` を含む同じリポジトリをJetsonにも配置します。
Notebook・Jetsonの自己位置推定用Mapは同じ内容にしてください。ファイルの更新日時は一致不要です。

1. 既存のセンサー・自己位置推定・operation manager・command mux・車両driverを起動します。
2. 操作モードをSTOPにします。通常のplanning/controllerやE2E自動制御は停止します。
3. **自己位置推定と同じROS_DOMAIN_ID・ROS環境**で、以下を起動します。

```sh
# 標準Docker環境。現在のディレクトリに依存しない絶対パスを使用
source /workspaces/ros2_ws/install/setup.bash
ros2 launch /workspaces/tools/app/runtime/tuning.launch.py \
  map_dir:=/workspaces/map/course_a
```

`/workspaces`以外に配置した場合は、リポジトリの配置先に合わせて絶対パスを変更してください。
`source install/setup.bash`を実行するROS workspace直下には、通常`tools/app`はありません。
launchファイルが指定位置に存在しないと、ROS 2がその引数をパッケージ名として扱い、
`is not a valid package name`となることがあります。
`ls /workspaces/tools/app/runtime/tuning.launch.py`でも見つからない場合は、Jetson側に
今回追加したコードが配置されているか、Docker内にリポジトリ全体がマウントされているかを確認してください。

車両用controller設定がある場合は `controller_config:=/absolute/path/controller.param.yaml`
を指定します。標準の設定は `jetpilot_controller/config/controller.param.yaml` です。
ROS関連依存はJetson側の既存rclpy・tf2_ros・jetpilot_msgs・jetpilot_controllerを使用します。
Mac側へのROSや追加Pythonパッケージのインストールは不要です。

bridgeはJetsonの `127.0.0.1:8781` のみで待受します。SSH先の `python3` からこのポートに
接続できる必要があります。ROSをDocker内で動かす場合は既存のhost networkingを使います。
ブラウザーやLANへ8781を直接公開しないでください。

`map_dir` は実際に自己位置推定が使うMapを指定してください。bridgeはこの指定先の
cuVGL/cuVSLAM資産・reference snapshot・landmarks YAMLの内容ハッシュを確認します。
別プロセスであるlocalizationの起動引数を自動検出する機能はありません。

## 調整の流れ

1. NotebookのConsoleで同じMapを開き、実車調整モードを選びます。
2. Jetsonのホスト名・ユーザーを指定して「接続」。SSH鍵による接続を使用します。
3. 既存エディターで編集して保存します。
   - **Geometry**：左右境界・センターラインの点を追加／移動／削除。
   - **Topology**：Section Gateを追加・移動し、区間を作り直す。
   - **Driving Lines**：Custom Lineを編集。レースラインの手修正はレースラインから
     Custom Lineを複製して行います。元の最適化結果を再生成する場合は既存Raceline生成を使います。
4. 上部の選択欄でセンター／レース／任意のCustom Lineを選びます。
5. 全体・区間別の目標速度上限を指定し、「保存済み編集をプレビュー」。
6. 青い候補線と最大速度を確認し、STOP・1秒以上の停車後に「この版を実車に適用」。
7. 実車側の版が表示されたことを確認して、既存の操作系でAUTOへ切り替えます。

白線は適用済みのライン、青線はプレビュー、橙色は自己位置と直近最大1,200点の走行軌跡です。
全ラインを既存のCustom Line用コンパイラーで再計算するため、入力した速度は上限です。
曲率・加減速制約により実際のprofileは低くなることがあります。レースCSV内の元の速度値よりも
調整パネルの目標値を優先します。Custom Line選択時は保存済みの全体・区間速度を初期値にします。

編集中の保存・プレビューは実車を変更しません。「適用」で形状・速度・HD Map・区間を一つの
ハッシュ付きsnapshotとしてJetsonに送り、STOP中に交換します。センター／レース／Customの
切替にもノードの再起動は不要です。通常のMapファイルや競技用経路選択設定は上書きしません。
適用したsnapshotの物理境界・固定障害物を`/tuning/drivable_area`へ出し、調整用の`drivable_guard`で判定します。
調整用controllerは `/tuning/*` の経路・速度・readyと`/tuning/safety_status`を使います。
通常の地図publisherの内容と混ざらないため、rollback時も経路と地図が同じ適用版へ戻ります。
判定方法・車体寸法・制動の仮設定は[Plannerの走行領域チェック](../README.md#plannerの走行領域チェック)を参照してください。signal分岐・recoveryなどの
競技planning managerを通すモードではありません。

「直前の版に戻す」は同じbridgeプロセス内で一つ前のsnapshotへ戻します。
再起動時は未適用から始まり、自動的に走行を再開しません。作成したsnapshotはNotebookの
Console state directory内 `tuning/<revision>.json` に保存されます。パネルだけで指定した速度は
snapshotに保存され、元のCustom Lineの編集設定には書き戻しません。

## 適用と通信断の条件

- 適用・rollbackは、新しいSTOP通知、0.05 m/s未満の速度が1秒以上継続、速度の受信から
  0.5秒以内を必要とします。途中の速度受信途絶は停車確認時間をリセットします。
- 異なる自己位置推定用Map、非map座標、不正な値、境界外経路、不明なSection、
  コンパイル後20,000点／8 MiB超のsnapshotは拒否します。
- UIが認識している実車の版とJetson側の版が異なる場合は上書きしません。
- 自己位置が古い・未確定の場合はUIの車両表示を消し、走行readyを無効化します。
- 接続状態の更新が4秒間届かないと走行readyを無効化します。復旧してもAUTOのままでは
  再許可しません。STOP・停車確認後に状態更新を受ける必要があります。
- controllerが使うwatchdogも有効です。停止指令の制動量は既存controller・車両設定に従います。
- `/auto/control_cmd` のpublisherが複数ある場合は調整経路をreadyにしません。
  これは別のpublisherを停止する機能ではないので、通常controller／E2Eとの同時起動を避けてください。

この版では速度だけの変更も停車して適用します。走行中の段階的な速度変更や次周適用は未実装です。

## ローカルでの検証

標準ライブラリだけで実行できます。ROS adapterの動作確認はJetson上で別途行います。

```sh
PYTHONPATH=tools/app/backend python3 -S -m unittest discover \
  -s tools/app/backend/tests -p test_live_tuning.py
node --test tools/app/frontend/tests/live_tuning.test.cjs
```
