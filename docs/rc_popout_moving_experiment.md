# 飛び出し実験：自車の固定スロットル走行

既存の `scripts/experiments/rc_popout_experiment.sh` に自車走行オプションを追加した。
オプションなしでは従来どおり自車静止で記録する。相手車の飛び出し操作は引き続きプロポで行う。

## 起動

Terminal 1（ROS 2環境／Isaac ROS container内）:

```bash
scripts/bringup.sh rc-popout --vehicle jpbb
```

Terminal 2で、ビルド済みROS 2環境を読み込んで実行する。
以下の数値は引数の書式例であり、実車で校正した値に置き換える。

```bash
source /workspaces/ros2_ws/install/setup.bash
scripts/experiments/rc_popout_experiment.sh \
  --experiment-dir /workspaces/record/rc_popout_moving_t010_d2 \
  --ego-throttle 0.10 \
  --ego-duration 2.0 \
  --ego-brake 0.20 \
  --ego-brake-duration 1.0 \
  --ego-steering 0.0
```

- `ego-throttle`: 自車の正規化スロットル `[0,1]`。一定速度を保証するものではない。
- `ego-duration`: 最初のスロットル指令から制動指令までの秒数 `(0,60]`。
- `ego-brake`: 正規化制動指令 `[0,1]`。`0` は中立であり能動制動ではない。
- `ego-brake-duration`: 制動指令の保持時間 `(0,10]`。既定1秒。
- `ego-steering`: 固定操舵 `[-1,1]`。既定0。経路追従・方位保持は行わない。

スロットル・時間・制動値は明示指定を必須にしている。相手車の `plan.tsv` の `throttle`
は従来のプロポ上限設定であり、自車の値とは別物。
開始位置へ毎回戻し、壁－カメラ距離は発進前の距離として記録する。
静止実験とは別の `--experiment-dir` を使い、自車条件を変える場合も別ディレクトリにする。
再開時は同じ自車オプションを指定する。保存条件と一致しない場合は開始しない。

## 各試行の流れ

1. CH3をHOST許可側にし、操作モードをSTOPにする。自車・相手車を開始位置へ戻す。
2. 開始LEDをRGBとEVSの両方に見える位置へ配置し、条件を確認してEnterで記録開始。LED同期パターンを撮影する。このEnterでは発進しない。
3. LEDを外して1秒以上待つ。記録を継続したまま待機し、準備ができたら別のEnterで走行を開始する。
4. 補助プログラムが記録状態とJPBBを確認し、AUTOを1回要求。中立ハンドシェイク完了後に発進する。
5. 固定スロットルを50 Hzで送り、指定時間後にスロットル0＋指定brakeへ切り替える。
   相手車の飛び出しはこの走行中に手動で行う。`none`試行でも自車は走行する。
6. 制動保持時間後に中立＋STOPを要求。記録は継続する。車両が実際に停止したことを確認し、LEDをRGBとEVSの両方に見える位置へ再配置して終了同期パターンを撮影する。
7. Enterで記録を終了し、従来どおり採用／取り直し／除外を選ぶ。

走行が失敗・中断した試行は自動採用せず未完了のまま保持する。
記録は終了LED撮影のため継続する。スクリプト自体のCtrl+C／TERMでは走行補助へTERMを送り、
STOP処理後にBag STOPを要求する。

## 指令経路と中断

`/auto/control_cmd` → 既存command mux → `/vehicle/control_cmd` → JPBBを使用する。
`/vehicle/control_cmd`への直接publishは行わない。
発進にはSTOP状態、正常かつHOST許可のJPBB診断、空でないURIでの記録中状態（STARTの重複拒否がないこと）、
AUTO publisherがこの補助だけであることが必要。AUTOアームを待つ間は中立を送る。
記録先の固定名設定にも対応し、開始確認時のURIを試行labelとともにログへ保存する。
走行中にそのURIが変更された場合も中断する。

走行中に操作モード変更、記録停止、診断の異常・期限切れ、競合publisher、
150 msを超える送信ループ遅延を検知すると中断し、再アームしない。
通常の制動はAUTOを維持したまま `brake` を送り、その後STOPにする。
異常中断・Ctrl+Cでは中立＋STOPを要求する。**既存muxのSTOPはbrake=0であり、能動制動ではない。**
プロセス強制終了やUSB断ではこの終了処理は実行できず、既存mux／JPBB／firmwareのタイムアウト処理に依存する。

このチェックアウトには基板firmwareのソースがなく、実機のESCによる制動・後退挙動は未検証。
初回はタイヤを浮かせて中立・前進方向・制動時の動作・CH3切替を確認し、走行中の制動距離は別途実測する。
固定指令時間はホストのmonotonic clockで管理し、実際の発進時刻・速度・停止時刻とは区別する。

## 保存と確認

- `ego_motion.json`: 自車の固定条件。
- `ego_drive.jsonl`: 試行labelと実際の記録先URI、各指令段階のホスト時刻・設定値。
- `attempts.jsonl`: 既存の採否に加え、自車開始要求・正常終了・失敗。
- MCAP: `/auto/control_cmd`、`/vehicle/control_cmd`、`/operation_mode/request`、
  `/operation_mode/state`、`/diagnostics`、JPBBの`output_channels`を収録対象にする。
  Bag Managerの設定変更を反映するにはbringupを再起動する。

送信なしで時間と指令の組み合わせを確認するには:

```bash
python3 scripts/experiments/rc_popout_timed_drive.py \
  --throttle 0.10 --duration 2 --brake 0.20 --brake-duration 1 --dry-run
```

作成済み試行表に対しては、進行shの同じ引数へ `--dry-run` を追加すれば、
最初の未完了試行の画面をROSなしで確認できる。ROSへの送信・採否更新・走行ログ保存は行わない。
新規の試行表作成は従来どおり対話端末が必要。

## 9月30日の出現なし6記録：動画生成と注釈

`t_0.1-none{,_01,_02}`は自車スロットル0.1、`t_0.2-none{,_01,_02}`は0.2。
後半3件はフォルダのみ改名している。MCAP名はそのままで読み込める。
転送先ホストでROS 2とmulti_sensor_calibrationの実行環境が利用できる場合:

```bash
cd /home/arata-24/workspaces/JetPilot
bash scripts/experiments/run_rc_popout_auto_pipeline.sh \
  --record-root "$PWD/record/09-30/2026-09-30" \
  --summary-dir "$PWD/record/09-30/analysis/auto_pipeline"
```

通常どおりROS環境をDocker内で使う場合は、転送先のJetPilotを`/workspaces`に
マウントしたcontainer内で実行する:

```bash
cd /workspaces
bash scripts/experiments/run_rc_popout_auto_pipeline.sh \
  --record-root /workspaces/record/09-30/2026-09-30 \
  --summary-dir /workspaces/record/09-30/analysis/auto_pipeline
```

最初に同じコマンドへ`--list-sessions`を追加すれば、処理対象の6フォルダだけを表示する。
この一覧確認にはROS環境が不要で、動画生成も行わない。
ROS workspaceは既定でスクリプトのあるリポジトリの`ros2_ws`を使用し、
別配置の場合は`ROS2_WS`環境変数または`--ros-setup`、`--config`、`--camchain`を指定する。
校正はRGB/EVSの取り付け関係と使用設定が対応するものを選ぶ。

各sessionの出力（`<root>`は指定した2026-09-30フォルダ）:

- 比較動画: `<root>/analysis/scenario_overlay/<session>/rotation_only_auto/rgb_vs_overlay.mp4`
- LED同期確認ページ: `<root>/analysis/led_sync/<session>/review/index.html`
- 一括結果: `--summary-dir`内の`summary.tsv`

LED抽出→自動同期→比較動画→確認ページの順で逐次処理する。生成済みの段階は再利用し、
低信頼度の同期は確認対象として残す。再実行で途中から継続できる。

動画生成後は、静止時と同様にLED同期と評価区間を確認する。ただし今回の6件は
「相手車の出現なし」であり、初出現時刻を付ける必要はない。映像で出現なしを確認し、
LED・手・配置戻しの区間を除外して、自車走行の評価区間とROIを注釈する。
自車の移動で背景も動くため、静止時の画像ROIや検出閾値の妥当性は改めて確認する。

## 全シーンのEVS共通視野・ROI指定用動画を先に生成

時間同期用のRGB座標動画とは別に、帯ROIを指定できるEVS座標動画を一括生成する。
Docker内で次を実行する（生成先・解析環境は上記と同じ）:

```bash
cd /workspaces
bash scripts/experiments/generate_rc_popout_common_views.sh \
  --record-root /workspaces/record/09-30/2026-09-30
```

- シーンごとの手動`time_sync_led.yaml`を優先し、ない場合だけ自動同期を使用する。
- 出力は`analysis/scenario_overlay/<session>/common_evs_batch_*/`。
  動画と`frames.csv`、投影・同期情報を生成するのでアノテーションUIから選択できる。
- 既定の投影は`rotation-only`。奥行きを使わないため視差は残る。
  固定深度投影を選ぶ場合は`--projection fixed-depth --depth-m <平面の仮定値>`を明示する。
  壁－カメラ距離をそのまま全画素の深度として扱わない。
- 同じ入力・同期・設定で生成済みなら再利用する。同期を修正した場合は別動画として生成する。
- `--dry-run`で対象・同期ファイル・実行コマンドを確認できる。
- 既存の動画・注釈は書き換えない。生成失敗は各動画横のlogと
  `analysis/common_views_summary.json`で確認する。

完了後、UIの「動画一覧を更新」で`common_evs_batch_... / evs`を選び、帯を指定して
「注釈を保存」する。保存済みの開始・終了は、ROIが未設定かつ同期・時刻基準と区間の
包含条件が一致すれば引き継げる。異なる同期の動画からは移行を拒否するので、
その場合は修正後の動画で区間を再確認する。
