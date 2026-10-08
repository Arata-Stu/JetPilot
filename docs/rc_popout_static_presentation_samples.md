# 静止・プロポ100%の資料用サンプル

既存の静止解析の注釈・同期・キャリブレーションを保持し、出現前・RGB初出現・出現後の静止画とスロー動画を生成する。検知の再推論や同期の再推定はしない。プロポ100%は送信機の設定であり、実速度ではない。

## 対象と表示条件

- 左3件：`popout-0928-static-100`、`_01`、`_02`。
- 右3件：`popout-0928-static-100_03`、`_04`、`_05`。
- EVS座標640×480、保存済みのrotation-only投影・共通有効視野。並進視差は残る。
- **時間軸もEVS：1 ms刻み、過去2 msのイベント**。RGBは取得済みの最新フレームを保持し、補間や未来フレームの利用はしない。
- 初期値の動画はRGB初出現`t0`の150 ms前〜350 ms後。60 fpsで再生するため約16.67倍スロー（約8.3秒）。
- 静止画は`t0−100 ms`、`t0`、`t0＋100 ms`。`t0`は車体の先端が初めて見えるRGB注釈であり、車体全体が出た時刻ではない。
- 画像は出力時間軸で指定時刻以降の最初のサンプルを使う。実際の表示時刻・RGB取得時刻は`summary.json`に記録する。初出現画像では注釈したRGBフレームとの一致も検査する。

## ノートPCの既存コンテナで実行

次の2ファイルを今回の版へ反映する。

- `scripts/experiments/render_rc_popout_static_samples.sh`
- `tools/rc_popout_static_samples.py`

既存の`multi_sensor_calibration`に`scenario-overlay --timeline event`があることが前提。今回そのパッケージは変更していない。ホストの`/home/arata-24/workspaces/JetPilot`を`/workspaces`へマウントした、従来のROS・Metavision環境で実行する。解析JSONに保存された`/workspaces/...`の参照をそのまま使う。

```bash
cd /workspaces

bash scripts/experiments/render_rc_popout_static_samples.sh \
  --static-dir /workspaces/record/09-28/analysis/grid_background_static_trial01 \
  --output /workspaces/record/09-28/analysis/static100_samples_trial01 \
  --preflight
```

成功後、`--preflight`を外して生成する。

```bash
bash scripts/experiments/render_rc_popout_static_samples.sh \
  --static-dir /workspaces/record/09-28/analysis/grid_background_static_trial01 \
  --output /workspaces/record/09-28/analysis/static100_samples_trial01
```

最初に1件だけ試す場合は、末尾へ `--sessions popout-0928-static-100_02` を追加し、出力先を `static100_sample_check01` など別名にする。既存フォルダは上書きしない。

時刻を変える例：`--sample-offset-ms -100 0 200`（出現後200 ms）。動画を変更する場合は `--before-s 0.2 --after-s 0.5`。イベントの1 ms刻みを保って60 fpsで再生する限り、スロー倍率は16.67倍。

## 生成物

`index.html`から全6記録を比較できる。各記録の中身：

```text
contact_sheet.png          # 横：前・初出現・後／縦：RGB・EVS・重畳
stills/
  before/{rgb,evs,overlay}.png
  onset/{rgb,evs,overlay}.png
  after/{rgb,evs,overlay}.png
video/
  rgb_vs_overlay.mp4       # 左RGB／右重畳、EVS時間軸のスロー
  polarity_only.mp4        # EVS単独のスロー
  overlay_polarity.mp4     # 重畳のスロー
  frames.csv              # 再生時間・記録時間・RGB時刻・RAW窓の対応
  summary.json
summary.json              # 注釈・同期・静止画の正確な時刻と出典
render.log
```

PNG9枚は文字・検知枠なしで元RGB/RAWから直接生成する。圧縮済みMP4を切り抜いた画像ではない。contact sheetと動画には時刻等を表示する。PNGのEVSは青＝正極性、赤＝負極性。画像の拡大によって元センサの分解能が上がるわけではない。

`t0`はRGBのフレーム注釈であり、物理的な初出現時刻やEVS側の最初の可視時刻を独立に確定したものではない。RGB保持中にもEVSは更新されるため、映像だけを使ってセンサ固有の遅延と断定しない。今回の生成物は資料用の観測例であり、新しい検出成績ではない。

## 検証

同期・注釈の改変、校正・投影の不一致、RGB未来フレームの使用、クリップ不足、初出現フレームの欠落を検査する。静止画のRAW窓・イベント数が動画と一致することも確認する。元RAW/MCAPは保存済みバイト数を照合し、内容全体のハッシュは計算しない。

```bash
PYTHONPATH=tools python3 -m unittest discover -s tests -p test_rc_popout_static_samples.py -v
```

2026-10-08にローカルで新規7テストと、利用するscenario-overlayの既存13テストが成功した。合成データによるPNG生成・時刻対応・事前検査・途中失敗の報告を確認した。実データのRGB/RAW生成はノートPCで実行し、`failed=0`と画像・動画の表示を確認する。
