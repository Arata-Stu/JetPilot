# 実車調整モード

Notebookの **Maps → 対象Mapを開く → 実車調整モード** から使います。
画像・動画・点群の継続転送は行いません。地図はNotebookにある形状データを描画し、
Jetsonからはmap座標の自己位置・速度・操作モード・適用版だけを約1 Hzで取得します。
転送はSSHの既存接続を再利用し、走行ライン全体はプレビュー・適用・適用版変更時だけ送ります。

## Jetsonの起動

このディレクトリと `tools/app/backend/jetpilot_console` を含む同じリポジトリをJetsonにも配置します。
Notebook・Jetsonの自己位置推定用Mapは同じ内容にしてください。ファイルの更新日時は一致不要です。

1. 既存のセンサー・自己位置推定・operation manager・command mux・車両driverを起動します。
2. 操作モードをSTOPにします。通常のplanning/controllerやE2E自動制御は停止します。
3. **自己位置推定と同じROS_DOMAIN_ID・ROS環境**で、以下を起動します。

```sh
# リポジトリのルートで、ROS workspaceのsetupを読み込んだ後に実行
ros2 launch tools/app/runtime/tuning.launch.py \
  map_dir:=/workspaces/map/course_a
```

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
調整用controllerは `/tuning/*` の経路・速度・readyのみを使います。signal分岐・recoveryなどの
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
