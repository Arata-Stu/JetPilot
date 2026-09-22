# 開発・保守コマンド

## 方針

**`uv.lock` は Git 管理に必ず含める。** `pyproject.toml`、対応する `uv.lock`、lock から生成した requirements を同じ変更でコミットする。
`.venv/`、`.venvs/`、ダウンロードキャッシュ、`config/host.local.json` は Git 管理に含めない。
lock は手編集しない。依存を更新したら実際の uv による解決と差分確認を行う。

Mac は編集と軽量検証に使用する。ROS ビルドと Python 環境の導入は Linux コンテナで行う。
以下のコマンドはプロジェクトルートから実行する。スクリプト内のルート解決は呼び出した場所に依存しない。

## ROS ビルド

```bash
./scripts/build.sh                                  # 対話選択。キャンセルで全体ビルドしない
./scripts/build.sh --all                            # 全体
./scripts/build.sh --packages jetpilot_controller   # 対象とワークスペース内の依存
./scripts/build.sh --packages jetpilot_controller --no-deps
./scripts/build.sh --packages jetpilot_controller --clean
./scripts/build.sh --all --clean-all --dry-run       # 削除予定とコマンドの確認のみ
```

- `--packages` は既定で `--packages-up-to`、`--no-deps` は `--packages-select`。
- `--clean` は指定パッケージの `build/<name>` と `install/<name>` のみ削除する。`--all --clean` または `--clean-all` では build/install/log 全体を削除する。
- merged install のパッケージ別クリーンは拒否する。
- x86・Jetson ともに並列数は colcon／ビルドツールの標準設定に任せる。スクリプト独自の制限は加えない。必要な場合だけ `--jobs N` で `CMAKE_BUILD_PARALLEL_LEVEL` を指定できる。この指定でもパッケージ並列数は制限しない。
- ccache がインストール済みなら自動使用する。`--no-ccache` で無効化できる。
- `--workspace` または共通設定の `ROS2_WS` でワークスペースを変更できる。
- CI・非対話入力では `--all` か `--packages` を必須にする。

## 録画先を指定する

bag manager が有効な bringup を端末から起動すると、録画名を入力する。
保存先は `record/<録画日のYYYY-MM-DD>/<入力した名前>/`。このフォルダ自体が bag で、
その下に `metadata.yaml` と MCAP 等のファイルが入る。時刻のサブフォルダは作らない。
日付には bag manager が動作しているマシンのローカル日付を使い、録画開始ごとに決める。
同じ日に同名で録画し直す場合は `名前_01`、`名前_02` と空いている連番に保存する。

```bash
./scripts/bringup.sh record                         # 起動時に録画名を入力
./scripts/bringup.sh record --record-name コースA_低速
./scripts/bringup.sh record --record-name コースA_低速 --dry-run
```

`RECORD_ROOT` で record の場所を変更できる。日本語・名前の途中の空白は使用可能。
`/`・`\`・制御文字・前後の空白・`.`・`..` は録画名に使用できない。
bag manager が無効なら入力は求めない。`--yes`・`--dry-run`・非対話実行でも入力は求めず、
`--record-name` または `BRINGUP_RECORD_NAME` で指定する。これらで名前を省略した場合は、
既存の自動実行との互換性のため従来の日時付き保存を維持する。
名前の指定は録画開始を意味せず、実際の開始・停止はこれまでどおりJoy等で行う。

ROS launch を直接呼ぶ場合は `bag_manager_output_dir` に保存ルート、
`bag_manager_recording_name` に名前を指定する。
変更反映には実行環境で `jetpilot_system_launch` と `jetpilot_bag_tools` をビルドする。

## 外部リポジトリ

```bash
./scripts/repos.sh                                 # 全リポジトリの状態
./scripts/repos.sh pull --dry-run                   # 更新方針を表示
./scripts/repos.sh pull                             # クリーンなブランチを一括更新
./scripts/repos.sh pull tools/isaac-ros-cli          # 一つだけ更新
./scripts/repos.sh import                           # packages.repos の不足分のみ取得
./scripts/repos.sh lock                             # 現在のコミットを packages.lock.repos に記録
./scripts/repos.sh import --manifest packages.lock.repos
```

`packages.repos` を唯一の対象一覧として使い、vcstool や PyYAML は不要。
状態には branch、HEAD、未コミット変更、ローカルに記録された upstream との差を表示する。
status と dry-run では fetch しないので、upstream 差分は最後に取得した情報に基づく。

pull は追跡先からの fast-forward のみ。未コミット変更があるもの、upstream 未設定、履歴が分岐したものは更新せず、その対象を報告する。タグなど detached HEAD は固定状態として維持する。
一つが失敗しても他の対象を確認し、失敗や未コミット変更によるスキップがあれば終了コードを非ゼロにする。自動 stash、reset、rebase はしない。
import は存在するリポジトリのブランチを切り替えない。取得先に通常ディレクトリがある場合も上書きしない。

`packages.lock.repos` も作成後は Git 管理に含める。未コミット変更があると再現できないため、lock の書き込みを拒否する。
部分的な lock には別の `--output` が必要。全体 lock を誤って部分一覧で置き換えない。

Isaac ROS CLI などは**独立した Git リポジトリ**。その変更は各リポジトリでコミットし、その後に親プロジェクトの lock を更新する。親だけのコミットでは内部の変更を保存できない。
今回の作業ではリモートの一括更新・コミット・push は実行していない。

## Python 環境と lock

| 環境 | 定義 | 対象 | 既定の導入先 |
| --- | --- | --- | --- |
| training | `python_ws/environments/training/` | Linux x86_64 / Python 3.12 | `/opt/env` |
| calibration | `python_ws/environments/calibration/` | Linux / Python 3.12、ROS の NumPy 1 ABI | `/opt/multi_sensor_calibration_env` |
| analysis | `python_ws/environments/analysis/` | Linux / Python 3.12、ROS 非依存の解析 | `.venvs/analysis` |

各ディレクトリの **pyproject.toml / uv.lock / 生成 requirements はすべてコミット対象**。
training は GPU 依存と周辺ツールを dependency group で分け、1つの lock で互換性を解決する。CPU 版 ONNX Runtime が GPU 版を置き換えないよう、E2E package の依存条件も揃えている。
calibration は system site-packages を参照し、学習用の NumPy 2 環境と分離する。

```bash
# Linux 上で、既存環境を消さずに lock の依存バージョンを反映
./scripts/maintenance/python_env.sh sync training
./scripts/maintenance/python_env.sh sync calibration
./scripts/maintenance/python_env.sh sync analysis
./scripts/maintenance/python_env.sh sync calibration --venv /workspaces/.venvs/calibration

# 依存定義を変更した後（Mac でも Python 3.12 があれば実行可能）
./scripts/maintenance/python_env.sh lock training --python /path/to/python3.12
./scripts/maintenance/python_env.sh export training --python /path/to/python3.12
./scripts/diagnostics/check.sh locks
```

uv 0.12.0 を基準にしている。`UV_BIN` で uv のパス、`JETPILOT_ENV_PYTHON` または `--python` で既存 Python 3.12 を指定できる。Python を自動ダウンロードしない。
sync は `--locked --inexact` を使う。lock の不整合はエラーにし、lock にない手動導入パッケージは残す。完全な環境再現が必要なら、新しい `--venv` パスを使う。
Mac 上の sync と ARM64 上の training 導入は拒否する。
校正環境では導入後に ROS/OpenCV/Metavision の import を確認する。対応するセンサー SDK を持つコンテナで実行する。

export は全間接依存と配布物のハッシュを含む requirements を生成し、Isaac ROS CLI が存在すればその Docker コンテキストにも同一内容を配置する。
Docker ビルドではこの生成ファイルを使用するため、毎回依存解決をし直さない。
CLI 側の生成ファイルも独立リポジトリの変更として保存する。CLI を別環境で単独ビルドする場合も生成済みファイルを利用できる。
`export --check` はファイルを変更せず、不一致を検出する。CI でも検査する。

既存の `trajectory_planning_helpers` は Docker の `TRAJECTORY_HELPERS_REF` による別管理を維持しており、この lock の対象外。完全なイメージ再現には、その Git 参照と APT・基底イメージの固定も必要。

## ホスト設定

共通の既定値は `config/host.json`。マシンごとの変更は、例を参考に `config/host.local.json` を作成する。

```bash
cp config/host.local.example.json config/host.local.json
```

優先順位は **環境変数 > host.local.json > host.json > 導出される既定値**。
`JETPILOT_HOST_CONFIG` で別のローカル JSON を選択できる。相対パスはプロジェクトルート基準。

- `JETSON_REMOTE_USER` / `JETSON_REMOTE_IPS` / `JETSON_WORKSPACE_ROOT`
- `JETSON_MAP_ROOT` / `JETSON_RECORD_ROOT`（未指定なら Jetson workspace から導出）
- `ROS2_WS` / `PYTHON_WS` / `MAP_ROOT` / `RECORD_ROOT`（未指定ならローカルのプロジェクトルートから導出）

bringup、tmux、地図作成・転送、ビルド、Console で共通利用する。
Console の「起動・運転」もこの設定を初期値に使う。ただしブラウザで保存した接続先は維持する。「既定値に戻す」で共通設定を反映できる。
JSON はコードとして実行しない。対応していないキーは入力ミスとして拒否する。

GUI の必要性は CPU と別に選べる。追加 Docker レイヤーの `INSTALL_GUI=auto` は従来どおり amd64 のみ、`true` は Jetson にも Terminator/RViz/rqt を追加、`false` は両方で追加しない。直接ビルドでは `--build-arg INSTALL_GUI=true` を指定する。

## テストの入口

```bash
./scripts/diagnostics/check.sh quick
./scripts/diagnostics/check.sh frontend
./scripts/diagnostics/check.sh locks
./scripts/diagnostics/check.sh ros --packages jetpilot_controller
./scripts/diagnostics/check.sh gpu
./scripts/diagnostics/check.sh jetson
```

- **quick**：Python 構文、シェル構文、保守スクリプト、既存 scripts、Console backend。`python -S` で標準ライブラリのテストを実行する。NumPy/OpenCV が必要な既存2ケースは環境未導入なら skip。ROS/GPU は不要。
- **frontend**：Node.js 標準テストランナー。npm install は不要。`NODE_BIN` で実行ファイルを指定できる。
- **locks**：Python 3.12 と uv が必要。3環境の lock と生成ファイルの整合性のみ確認し、依存をインストールしない。
- **ros**：Linux で ROS/workspace を source し、事前ビルドした package の colcon test と結果確認。
- **gpu**：x86_64 Linux の学習環境で CUDA の小さな演算と ONNX Runtime CUDA provider を確認する。モデル精度の検証ではない。
- **jetson**：起動済み ROS の node/topic 一覧を読むだけ。センサー周期・キャリブレーション・推論結果の実機検証は別途必要。

GitHub Actions は quick / frontend / locks のみ。ROS/GPU/Jetson は対応する実行環境で明示的に選択する。

## Console の分割方針

追加ビルドや npm 依存を導入せず、既存の classic script と共有状態を維持した。

- `preflight_ui.js`：実行前の準備状況・競合確認
- `list_ui.js`：一覧の検索・並び替え・日付分類
- `training_ui.js`：E2E / 物体検出 / shared ViT の学習・配備
- `transfer_ui.js`：Jetson への転送・記録データ取得
- `map_formats.py`：YAML/JSON 読み込み
- `custom_trajectory.py`：ファイル操作に依存しない軌道・速度計算

起動処理は `app.js` に残す。新しい機能ファイルは `index.html` の app.js より前に読み込む。
テストも同じ宣言順で読み込み、静的ファイルのキャッシュ識別値には分割ファイルを含める。
既存の大きなファイル全体を一度に置き換えるのではなく、今回分離した責務を追加機能の置き場所として使う。
