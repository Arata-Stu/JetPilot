# grid背景モデル：内部評価候補の映像照合

2026-10-07。受領した `grid_background_eval_review_trial01` の全30候補について、各4コマのcontact sheetと必要箇所の原寸PNGを確認した。検知器・閾値・ROI・split・LED同期は変更していない。

## 結論

**初出現後250 ms以内のEVS候補8件は、車体との空間的な対応を支持する6件、背景1件、判断保留1件だった。** 「8/8件で車両を検知」とは記述しない。`test_04` の候補は車両ではなく右側の段ボール上端に位置する。`test_15` は車体と床・遮蔽物境界の寄与を分離できず、判断を保留する。

RGBでは同じ時間窓内の4候補のうち3件が段ボール上端、1件が車体下縁・影付近で判断保留だった。一方、時間窓より後の `test_04`・`test_06` には車体との対応を支持する候補がある。

初版では確認動画8本で画像と付記時刻が1フレームずれる不整合を発見した。**修正版trial02を受領し、全120枚で時刻表示と対応表の不一致が解消したことを確認した。** 位置の判定は変更していない。候補CSVの時刻やLED同期値も変更していない。

## trial02の再照合結果

全245ファイルを受領。30候補すべて `complete`、`review_errors.json=[]`、切り出し方式は全件 `sequential_from_start` だった。候補・注釈・同期・元動画を含むmanifestはtrial01と同一、summaryも切り出し方式の欄を除き同一である。

| 確認項目 | trial02の結果 |
|---|---|
| 動画のデコード | 30本、計1061フレーム |
| 画像の時刻ラベルと対応表 | 120/120枚が許容値0.75 ms以内、最大差0.497 ms |
| 前回から変更された画像 | 32枚。前回の時刻不一致32枚と一致 |
| 変更されなかった画像 | 88枚。ファイルのSHA-256が一致 |
| 同一元動画からの重複クリップ照合 | 同じフレーム番号で一致。前回の+1フレーム対応は解消 |

変更された8クリップすべてと、指定された `test_02/evs_c002`・`test_15/evs_c001` のcontact sheetを再度目視した。判定変更は0件。`test_02` の出現後候補は車体との対応を支持する。`test_15` は上側タイルに車体がある一方、下側タイルは床・遮蔽物境界付近であり、タイル対の反応源を分離できないため判断保留を維持する。

[trial02の検証記録](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/trial02/verification.json)、[120枚の時刻照合](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/trial02/timestamp_label_audit.csv)、[重複区間比較](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/trial02/overlap_audit.json)、[目視再確認](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/trial02/visual_recheck.json)を保存した。

ここで確認したのは切り出し画像と時刻対応表の整合であり、物理的なRGB/EVS同期誤差がゼロという意味ではない。既存のLED同期・注釈の不確かさは残る。この不整合のために再生成を繰り返す必要はなく、次は固定方式の静止記録への適用確認へ進める。

## 判定の意味と範囲

- `vehicle_supported`（車体との対応を支持）：候補近傍で、枠と車体またはそのイベント輪郭が明瞭に重なる。背景の混入を排除した因果的な証明ではない。
- `background`（背景位置）：枠が壁面ボード・段ボール・床の境界にあり、車体と離れている、または枠内に車体がない。
- `uncertain`（判断保留）：車体下縁・影・床・遮蔽物境界が重なり、候補を車体由来と判断するには不足する。

同じ基準をRGBとEVSに適用した。RGB候補の判定にEVS側のイベント輪郭を代用しない。以前の開発用映像で床・影付近を保留した基準も維持する。

表示枠はグローバル警報開始時の2タイルを固定したもので、追跡枠ではない。警報継続中の最大スコアタイル対は移り得る。また動画のEVS表示は中央10 ms、検知器の入力窓は過去2 msで、検知器には時間積分もある。この映像照合だけで、候補を発火させた入力イベント集合を特定したことにはならない。

## 集計

時間窓は保存済みCSVの候補開始時刻とRGB手動初出現との差で分類した。下表は正例8記録の内訳であり、検知成功率の確定値ではない。

| RGB初出現後0〜250 msの候補 | RGB | EVS |
|---|---:|---:|
| 車体との対応を支持 | 0 | 6 |
| 背景位置 | 3 | 1 |
| 判断保留 | 1 | 1 |
| 当該時間窓に候補なし | 4 | 0 |

全保存区間の候補総数はRGB19件、EVS11件。その目視分類は次の通り。

| 全候補の分類 | RGB | EVS |
|---|---:|---:|
| 車体との対応を支持 | 2 | 6 |
| 背景位置 | 15 | 4 |
| 判断保留 | 2 | 1 |

全30候補の所見・タイル・元画像の相対パスは[判定CSV](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/visual_decisions.csv)、集計は[JSON](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/visual_aggregate.json)に保存した。

### EVSの出現後候補

| 記録 | タイル | 暫定判定 | 観察 |
|---|---|---|---|
| `test_02` | 160・180 | 車体との対応を支持 | 左画像端の車体・イベント輪郭と重なる |
| `test_04` | 97・98 | **背景位置** | 右側段ボール上端・縦縁。車両は左下 |
| `test_06` | 177・178 | 車体との対応を支持 | 右側から現れる車体・車輪と重なる |
| `test_07` | 174・175 | 車体との対応を支持 | 右側から現れる車体と重なる |
| `test_12` | 160・180 | 車体との対応を支持 | 左画像端の車体・イベント輪郭と重なる |
| `test_13` | 160・180 | 車体との対応を支持 | 左画像端の車体・イベント輪郭と重なる |
| `test_15` | 176・196 | **判断保留** | 上側は出現車体、下側は床・段ボール下端付近 |
| `test_16` | 175・176 | 車体との対応を支持 | 右側の車体と重なる。trial02で時刻表示を再確認済み |

`test_04` と `test_16` の該当クリップはtrial02で再確認済みである。表は候補前後の位置関係の判断で、発火瞬間の車体由来反応を断定しない。

EVSの早期候補 `test_02/c001` と `test_16/c001` は、ともに左上の壁面ボード下端（タイル82・83）に対応した。`test_16/c003` は車両通過後の壁床境界・床で、車体との対応を確認できない。EVSでも正例3記録（02・04・16）に背景位置の候補が残っている。

飛び出しなし2記録のEVS候補は0件、RGB候補は1件で、RGBの枠は左上ボードの下端・角だった。負例の走行指令区間は合計約4.02秒に限られる。

### RGBで見えたもの

早期候補は左上の壁面ボード付近、出現後の複数候補は右側段ボール上端に集中している。正例8記録すべてに少なくとも1件の背景位置の候補があった。

車体との対応を支持する候補は `test_04/c003` と `test_06/c003` で、CSV上では初出現後それぞれ約748 ms、603 ms。初出現直後の検知とは呼べない。`test_12/c002`（約352 ms）と `test_13/c002`（約150 ms）は枠の大部分が床で、車体下縁・影付近と重なるため判断保留とした。

### 保存した原寸例

コピー元の画素を変更していない。`test_04` は時刻対応を修正したtrial02の画像を参照する。ほかの3例はtrial01とtrial02で同一の画像だった。

- [test_04 EVS：右上の段ボール](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/trial02/test_04_evs_c001_candidate_after.png)
- [test_06 EVS：車体との対応](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/test_06_evs_c001_candidate_after.png)
- [test_15 EVS：判断保留](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/test_15_evs_c001_candidate_after.png)
- [test_13 RGB：判断保留](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/test_13_rgb_c002_candidate_after.png)

## 初版の1フレーム不整合と修正

上段に焼き込まれたRGB時刻と、下段・`frames.csv` の時刻を照合した。120枚中32枚が丸め誤差の許容値0.75 msを超えて不一致となり、その32枚すべてが対応表の**次フレーム**の時刻と一致した。OCR読取不能は0枚。対象は以下の8クリップ、各4枚だった。

- `test_04`：`rgb_c001`、`rgb_c002`、`rgb_c003`、`evs_c001`
- `test_16`：`rgb_c001`、`rgb_c002`、`evs_c002`、`evs_c003`

さらに同一の元動画ハッシュを持つ `test_16/evs_c001` と `test_16/rgb_c001` の重複区間を照合した。同じ元フレーム番号を付けられた画像同士より、1フレーム先との間で時刻ヘッダの画素が一致した。これはLED同期値の正負の問題を示すものではなく、確認動画のフレーム割り当ての不整合である。

[時刻ラベル照合CSV](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/timestamp_label_audit.csv)と[重複区間の比較](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/overlap_audit.json)を保存した。

既存rendererはOpenCVで途中フレームへシークし、返されたフレーム番号が正しいことを確認していた。番号が正しくても画素が次フレームになる状況に備え、[共用renderer](../tools/rc_popout_candidate_videos.py)を**先頭から順にデコードして指定区間を取り出す方式**へ変更した。生成レポートに `source_decode_strategy: sequential_from_start` を記録する。

シークが要求番号を報告しながら次フレームを返す模擬実装を用いた回帰テストを追加した。逐次デコードした基準画像と出力PNGの一致を確認し、関連15テストが成功した。その後trial02で上記の修正効果を確認した。元のフル動画は未受領のため、特定のOpenCV／コーデック組み合わせが原因だという点は推定にとどまる。

## 実施済みの再生成手順

修正コードをデータ側へ反映して、確認動画だけを新しい出力先へ生成する。LED同期・共通視野の元動画・検知処理をやり直す必要はない。

```bash
cd /workspaces

bash scripts/experiments/render_rc_popout_grid_evaluation_reviews.sh \
  --evaluation-dir /workspaces/record/09-30/analysis/grid_background_eval_trial01 \
  --record-base /workspaces/record/09-30 \
  --output /workspaces/record/09-30/analysis/grid_background_eval_review_trial02

cat /workspaces/record/09-30/analysis/grid_background_eval_review_trial02/review_errors.json
```

上記コマンドで全30候補を再出力し、trial01を残したままtrial02を受領・照合した。推論結果のCSVや閾値は書き換えていない。既存trial02へ再実行する必要はない。

## 受領確認と再現

- 受領元：`/Users/at/Downloads/scp/grid_background_eval_review_trial01/`
- 245ファイル、30候補すべて生成完了、`review_errors.json=[]`。
- 全30動画・計1061フレームをデコードでき、対応PNGと出力動画の整合も確認した。**全フレームを連続再生して目視したという意味ではない**。
- 30候補と20件のセンサ別summaryは、先に受領した貼付CSVと一致した。
- 受領manifestの固定設定ハッシュはローカルの凍結ファイルと一致した。生成コードのハッシュは修正前のコミット `9e0371c94f89a8092403dd5c5f4fc4312035c0de` と一致した。
- フル共通視野動画、RAW、bag、評価用NPZは未受領。元入力からの推論再現や実際の物理同期を独立確認したものではない。

[検証記録](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/verification.json)に受領全ファイルのサイズ・SHA-256と照合結果を保存した。[監査スクリプト](evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/audit_received.py)は、NumPy・OpenCV・Tesseractを使用し、受領ディレクトリを引数にして再実行できる。目視判定CSVは人による入力であり、このスクリプトで自動判定したものではない。

```bash
python3 docs/evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/audit_received.py \
  /Users/at/Downloads/scp/grid_background_eval_review_trial01
```

trial02の照合を再現する場合は、出力を分けて修正版のコミットと比較元を指定する。描画プログラムの再実行は行わない。

```bash
python3 docs/evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/audit_received.py \
  /Users/at/Downloads/scp/grid_background_eval_review_trial02 \
  --output docs/evidence/rc_popout_20260930/grid_background_evaluation_20261007/visual_review/trial02 \
  --code-commit d27d37b1e78cd2f4eaabad2e56daf430a155f22e \
  --previous-review /Users/at/Downloads/scp/grid_background_eval_review_trial01
```

## 次の評価と発表での位置づけ

移動中の背景を課題として示すだけでなく、背景モデルを用いた局所残差に車体と対応する候補が得られたことを示せる。ただし背景境界の警報と曖昧な候補が残り、実用的な車両検知の完成やEVS一般の優位性を示した結果ではない。

今回の評価を見て閾値・タイル・発進直後の除外時間を変更しない。確認動画の時刻不整合は解消したため、次に固定方式の静止記録への適用可否を確認し、同じ定義で誤警報・候補位置・反応時刻を整理する。追加調整が必要なら探索実験として区別する。今回の内部評価用記録は以前の別解析でも一部集計値を見ており、完全な未知データとは呼ばない。

ポスターの暫定表現案：

> 自車移動に伴う背景活動を抑えるため、飛び出しなし記録からgrid単位の背景モデルを構成した。固定条件での内部評価8記録において、EVSではRGB初出現後250 ms以内の候補のうち6記録で車体との空間的な対応を確認した。一方、背景境界に対応する警報と判定困難な候補も残り、検知性能の確定には追加検証を要する。

この文章は空間的な暫定所見である。提出図は時刻対応を確認したtrial02から選ぶ。人間との比較、オンライン計算遅延、制動性能は今回の結果に含まない。
