# 調整用5記録のタイル活動量を取り出す

目的は、全ROIの合計で埋もれていた局所的な活動増加があるかを確認すること。
`tools/rc_popout_tile_activity.py` は**診断用の抽出器**であり、飛び出し検知の成功率や検知時刻を出すものではない。
全ROI検知の第2試行と判断の根拠は [rc_popout_change_detection.md](rc_popout_change_detection.md) に保存した。

## 2026-10-06：第1回タイル抽出の実データ結果

ユーザー提示の実行ログにより調整用5件すべて `complete`、各180タイル、比較図のエラーは0件。
元CSVは [tile_diagnostics.csv](evidence/rc_popout_20260930/tile_dev_trial01/tile_diagnostics.csv)、サンプル数と実行条件は
[provenance.json](evidence/rc_popout_20260930/tile_dev_trial01/provenance.json) に保存した。
元の全時系列・RAW・比較図そのものはこの環境では確認していない。

各正例のEVSでafter最大のタイルにおける活動密度は次の通り。

| session | タイル原点 (x, y) | 正例before p95 | 正例after p95 | 負例同時刻・同座標after p95 |
|---|---|---:|---:|---:|
| `test_01` | (0, 256) | 0.046875 | 0.101758 | 0 |
| `test_05` | (576, 256) | 0.012842 | 0.130664 | 0.004053 |
| `test_11` | (160, 288) | 0 | 0.392188 | 0 |
| `test_14` | (480, 256) | 0.023877 | 0.312061 | 0.011719 |

単位は元の蓄積窓内イベント数／有効画素数。分母0の比を無限大の改善として扱わない。
RGBも全4件で選択タイルのafterがbeforeと負例afterを上回るが、例えば `test_11` は負例afterも0.480273あり、背景は無視できない。
これは事後的な最大タイル選択による結果で、全時間・全タイルの誤反応率や検知時刻は分からない。

局所比較の試作へ進む根拠はあるが、検知成功を確認した段階ではない。
次は [局所背景・同じ高さの帯・隣接タイルを使う候補検知](rc_popout_local_detection.md) を調整用5件で試す。
抽出の再実行は不要。

## 実行

このリポジトリの変更をデータ側へ反映した後、従来の解析用コンテナで実行する。
今回の変更はトップレベルのPython・shなので、この変更だけのためのcolcon buildは不要。
ROS、multi_sensor_calibration、Metavision、NumPy、OpenCVは従来の解析環境を使用する。
こちらからSSHはせず、ユーザーが実行する。

```bash
cd /workspaces

bash scripts/experiments/export_rc_popout_tiles.sh \
  --motion-dir /workspaces/record/09-30/analysis/detection_motion_trial01 \
  --subset development \
  --tile-px 32 \
  --output /workspaces/record/09-30/analysis/tile_dev_trial01

cat /workspaces/record/09-30/analysis/tile_dev_trial01/summary.json
cat /workspaces/record/09-30/analysis/tile_dev_trial01/review_errors.json
cat /workspaces/record/09-30/analysis/tile_dev_trial01/tile_diagnostics.csv
```

対象は固定済みの `t_0.2-none`, `test_01`, `test_05`, `test_11`, `test_14`。
`--subset evaluation` は受け付けず、評価用の各記録・注釈・スコアは読まない。
共通の元設定JSONには他セッションのパスも含まれるが、開くのは調整用5件だけ。
保存先が存在すれば中止する。再実行は新しい保存先にする。

入力はmotion側の `run_config.json` が参照する元スコアの設定、および選択した各記録の `alignment.json`。
元設定にある注釈JSONからRAW/bagへ辿るため、共通ROI・校正・同期・時間範囲を再指定する必要はない。
元スコア結果・注釈・同期のハッシュ、RGB初出現と発進基準時刻の整合を検査する。
参照先を移動した場合は `--score-config /移動先/run_config.json` も指定する。注釈内の参照パスも有効である必要がある。

## 抽出される値

- EVS画像座標の原点から32×32画素で分割し、元解析の共通ROIとの交差部分だけを集計する。
- RGB：校正・投影後のグレースケール画像に元の `rgb_pixel_delta` を適用した、変化画素数。
- EVS：同期・歪み補正後のイベント数。元の更新周期・蓄積窓をそのまま使用する（今回の元設定では1 ms更新・2 ms窓）。同一画素に繰り返し来るイベントも数える。
- 各タイルの有効画素数も保存する。端の部分タイルを32×32=1024画素として扱わない。
- 時間範囲・除外区間は元注釈と同じ。発進前後・制動・その後を含め、RGB初出現で切ったり発進後だけに限定したりしない。
- RAWの順序が前後しても最後まで読んで時刻ビンへ加算する。時刻の書換えや遅着イベントの削除はしない。
- 出力前に、全タイル合計が同時に再計算した従来の全ROIスコアと全時刻で一致することを検査する。RGBは合計をROI有効画素数で割って比較する。

スコアを空間的に分ける処理であり、自車運動補償や物体識別はまだ行わない。
推定配列容量が既定の512 MiBを超えると中止する。容量に余裕がある場合だけ `--max-memory-mb` を増やす。
これは配列の概算値であり、RAWデコーダなどを含むプロセス全体のメモリ上限ではない。

## 出力と確認

| ファイル | 内容 |
|---|---|
| `summary.json` | 抽出成否、タイル数 |
| `review_errors.json` | 比較図のエラー。成功時は空配列 |
| `index.html` | 正例4件の比較図へのリンク |
| `tile_diagnostics.csv` | 正例×センサの計8行。出現後p95最大タイルの診断値 |
| `<session>/rgb_tiles.npz`, `evs_tiles.npz` | 全時間範囲・全タイルの時系列 |
| `<session>/tiles.json` | タイル座標・有効画素数、ROI・窓幅 |
| `<session>/tile_review.svg` | 正例の出現前後と負例の同じ発進経過時刻を並べた空間分布 |
| `<session>/result.json` | 再計算の元結果、発進基準情報、入力ハッシュ |
| `run_config.json` | 元設定・区分・抽出コードのハッシュなど |

`summary.json` の5件が `complete`、`review_errors.json` が `[]` であることを確認する。
`index.html` をブラウザで開くと各正例の比較図を選べる。

NPZ内の `counts` は `(観測数, タイル数)` の整数配列。
`interval`, `support_start_s`, `time_s` は各観測の区間ID・観測窓始点・判定可能な終端時刻。
秒数は元RGB記録の基準時刻からの相対秒であり、発進基準へは `alignment.drive_start_s` を引く。
`tile_id` と `valid_pixels` は列に対応する。**各観測の終端時刻より前へ判定を戻さない。**
この保存後は、タイルの局所背景比較を試すたびにRAWを再読込する必要はない。

### 比較図・CSVの意味

正例のRGB初出現を0として、前は−130～−30 ms、後は+30～+130 msの100 ms区間。
観測窓全体が区間内に入るサンプルだけでp95を計算する。
負例は同じ「発進指令からの経過秒」で揃える。これは車両位置や背景画像が厳密に一致することを保証しない。

図はRGBとEVSそれぞれに、正例before・after／負例before・afterの4枚を並べる。
同じ行の色尺度は共通。RGBの単位は変化画素数／有効画素数、EVSは蓄積窓内のイベント数／有効画素数で、両者の数値の大小を直接比べない。
部分タイルもセル全体を着色するが、値の計算には有効画素だけを使う。灰色は有効画素を持つタイルがない領域。
RGBは100 ms内に数サンプルしかなくp95は最大値に近い。EVSの重複窓も独立サンプルではなく、p95を統計的有意性と解釈しない。

CSVは、**各正例・各センサでafterのp95が最大のタイルを事後選択**し、同じ座標のbefore・負例を併記する。
これは車両位置の推定でも、未知の出現時刻で動く検知器でもない。背景タイルが選ばれる可能性もある。
最大タイルだけで結論せず、空間分布と、保存した出現前・負例の全時系列を合わせて確認する。

## この後の判断

正例に局所的な増加があり、そのタイルの過去活動や周辺タイルと区別できるかを確認する。
分離の見込みがあれば、タイルごとの過去背景からの偏差・周囲に共通する変化・空間的なまとまりを共通ルールとして設計する。
初出現時刻は診断・評価用に限り、検知の許可時刻や背景更新停止の条件には使わない。
負例や出現前にも同じ変化があれば、その誤反応を含めて判定を見直す。評価用は方式固定まで開かない。

## ローカル検証

NumPyがあるPython環境で以下を実行する。

```bash
python3 -m unittest discover -s tests -p 'test_rc_popout_tile_activity.py' -v
python3 -m unittest discover -s tests -p 'test_rc_popout_detection.py' -v
python3 -m unittest discover -s tests -p 'test_rc_popout_change_detection.py' -v
bash -n scripts/experiments/export_rc_popout_tiles.sh
```

2026-10-06時点：計32テスト成功。ROI境界、イベント重複と時刻の前後、区間分離、全ROIとの一致、開発用だけの読込、観測窓と発進基準の比較、メモリ概算制限を含む。
合成データのSVGも描画確認した。実記録のタイル抽出は上記ユーザー提示ログで成功を確認した。検知性能の評価は未実施。
