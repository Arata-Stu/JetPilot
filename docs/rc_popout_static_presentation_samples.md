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

## RGB初出現の前後1フレームだけに差し替える

基準時刻の説明図には、`--rgb-adjacent-only`で元RGBの「直前・初出現・直後」の3枚だけを生成できる。EVS座標640×480・共通有効視野を維持し、PNGを元RGBから直接書き出す。RAWイベントの復号や動画の再生成は行わない。`--sample-offset-ms`との同時指定はできない。

```bash
cd /workspaces

bash scripts/experiments/render_rc_popout_static_samples.sh \
  --static-dir /workspaces/record/09-28/analysis/grid_background_static_trial01 \
  --sessions popout-0928-static-100_01 \
  --rgb-adjacent-only \
  --output /workspaces/record/09-28/analysis/static100_rgb_adjacent_trial01
```

各記録の `stills/before/rgb.png`、`stills/onset/rgb.png`、`stills/after/rgb.png` を使用する。`contact_sheet.png`は実際のRGB取得時刻差を添えた3枚組。`summary.json`に元RGBのフレーム番号と取得時刻を保存する。1 msのEVS表示時刻や公称60 fpsの逆数をラベルに使わず、実際のRGB取得時刻差を用いる。

転送済み`static100_samples_trial01`の時刻対応表では、`popout-0928-static-100_01`のRGB隣接フレームは初出現から **−16.694 ms／0 ms／＋16.786 ms**。以前の−100.6 ms／＋83.8 msの画像を単に改名する処理ではない。前後フレームのない記録や、注釈時刻に対応するRGBフレームがない場合は停止する。

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

2026-10-08にローカルで初版7テストと、利用するscenario-overlayの既存13テストが成功した。RGB隣接フレーム機能では追加3テストも含め、ネイティブRGBの隣接性、不等間隔・エポック時刻、RAWと動画生成を呼ばないPNG出力を検証する。実データの初版生成は利用者ログで6記録すべて成功。隣接RGBの新しいPNG生成はノートPCで実行し、`failed=0`と画像を確認する。
