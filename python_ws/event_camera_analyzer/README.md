# Event Camera ROSBag Analyzer (`event_camera_analyzer`)

`event_camera_analyzer` は、`openeb_ros2` (SilkyEvCam 等の OpenEB ドライバおよびプリプロセッサ) によって出力され `rosbag` (`.mcap` または `.db3`) に記録された以下のトピックを突合・解析し、**イベント画像 (`event_image`) の描写がチラついた際の原因がイベントストリーム (`events` / `events_raw`) にあるのか、画像生成側 (`on_event_image_timer` のジッターやデコードエラー) にあるのかを自動判定・可視化**するスタンドアロン Python ツールです。

## 特徴

- **スタンドアロン動作:** ROS 2 環境や `rclpy` を必要とせず、`rosbags` ライブラリを使用して `.mcap` / `.db3` をオフラインで直接読み出します。
- **時系列同期解析:** `/event_camera/events_raw`, `/event_camera/events`, `/event_camera/event_image`, `/event_camera/diagnostics` を共通タイムライン上で評価します。
- **チラつき検知＆原因帰属エンジン (`Diagnoser`):**
  - **`STREAM_STARVATION` (イベントストリーム起因):** パケットの到着間隔 ($\Delta T$) スパイク、パケット欠落、あるいはイベントレート (MEV/s) 低下に起因するチラつき。
  - **`TIMER_DESYNC_OR_JITTER` (画像生成・タイマー起因):** イベントは定常的に到着しているにもかかわらず、プリプロセッサの Wall-clock タイマーのゆらぎや到着タイミング干渉によって画像バッファへの蓄積期間が極端に短くなった現象。
  - **`DECODE_ERROR` (デコードエラー起因):** `preprocessor_stats` で `decode_errors` や領域外イベントが検出された現象。
- **インタラクティブ Plotly レポート:** ズーム可能な HTML チャートとコンソールサマリーを同時生成します。

## インストール

```bash
cd python_ws/event_camera_analyzer
pip install -r requirements.txt
# CLI コマンドとしてインストールする場合:
pip install -e .
```

## 使い方

### CLI コマンド

```bash
openeb-bag-analyzer --bag /path/to/rosbag_dir --output-html ./report.html
```

または Python モジュールとして実行:

```bash
python3 -m event_camera_analyzer.cli \
  --bag /path/to/rosbag_dir \
  --output-html ./report.html \
  --namespace /event_camera \
  --fps 25.0
```

### コマンドライン引数

- `--bag` (必須): 解析対象の ROSBag ディレクトリパス（`.mcap` や `.db3` が含まれるフォルダ）。
- `--output-html`: 出力する Plotly HTML ダッシュボードのファイルパス（デフォルト: `event_camera_report.html`）。
- `--namespace`: トピックの名前空間（デフォルト: `/event_camera`）。
- `--fps`: 期待される `event_image` のフレームレート（デフォルト: `25.0` Hz）。
- `--drop-threshold`: チラつき（輝度/アクティブピクセル率のドロップ）と判定する比率閾値（移動平均に対する割合。デフォルト: `0.5`）。
- `--window-ms`: チラつきフレーム直前の要因分析を行う時間枠（デフォルト: `60.0` ms）。
