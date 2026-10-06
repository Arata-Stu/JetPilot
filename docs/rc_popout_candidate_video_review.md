# EVS候補領域の枠付き確認動画

調整用の `test_01`, `test_05`, `test_11`, `test_14` に対し、EVS開始閾値0.25で得た候補を映像で確認する。
対象は[2026-10-06の解析結果](rc_popout_development_bundle_analysis_20261006.md)。候補の再計算や設定変更はしない。
使用する候補時刻・2タイルの座標・同期／校正／注釈ハッシュは
[candidate_review_manifest.json](evidence/rc_popout_20260930/development_bundle_analysis_20261006/candidate_review_manifest.json)に保存した。

## 実行

新しいスクリプト・Pythonツール・上記manifestとsplit JSONをLinux側の同じチェックアウトへ反映してから実行する。
既存の共通視野動画、OpenCV、NumPy、H.264エンコーダを含むffmpegが必要。ROSの起動やRAW再読み込みは不要。
ラッパーは既存のキャリブレーション用Python環境を優先する。

```bash
cd /workspaces
bash scripts/experiments/render_rc_popout_candidate_reviews.sh \
  --record-root /workspaces/record/09-30/dynamic \
  --output /workspaces/record/09-30/analysis/candidate_review_trial01
```

対象4件を自動選択する。デフォルトは候補の前後各0.3秒、4倍スロー。
既存出力は上書きしない。再実行時には `candidate_review_trial02` など新しい出力先を使う。
メタデータと時刻対応の確認だけなら、同じコマンドに `--dry-run` を付ける（動画の復号検査は本実行時）。
校正ファイルの置き場所が異なる場合は `--camchain <path>` を指定できるが、校正内容のハッシュは解析時と同一である必要がある。

確認ページ：

```text
/workspaces/record/09-30/analysis/candidate_review_trial01/index.html
```

各記録の `candidate_review.mp4` をVLCなどで直接開くこともできる。

## 表示の意味

- 左はEVS座標へ変換したRGB、右は同じ座標のRGB＋EVS重畳画像。
- 枠は、候補が発生した時点の2タイルを全フレームへ固定表示したもの。車両追跡結果や警報継続状態ではない。
- 水色は候補時刻より前のフレーム、橙色は候補時刻以降のフレーム。候補時刻をまだ迎えていないフレームに橙色を先取り表示しない。
- 下部に録画基準秒、発進指令からの秒、候補およびRGB初出現からの差を表示する。動画の再生位置を録画時刻と誤認しないため、元動画の `frames.csv` からフレームを選ぶ。
- 元動画のRGBフレーム刻みを維持し、4倍スローは再生fpsを1/4にする。補間フレームは作らない。
- 候補がRGBフレーム間にある場合、最初の候補以降フレームとの時刻差をページとJSONに明記する。正確な候補時刻のRGB画像が新しく得られるわけではない。
- 既存の通常プレビューはEVSを中心10 ms窓で表示している。検知で使った過去2 ms窓とは異なるため、ここでは反応位置と対象の対応を確認する。厳密なEVS初出現時刻やms単位の遅延を、このプレビューから再測定しない。

## 出力

```text
candidate_review_trial01/
  index.html
  summary.json
  review_manifest.json
  run_config.json
  test_01/                       # 残り3件も同様
    candidate_review.mp4         # 前後0.3秒、4倍スロー、H.264
    before_onset.png             # 原則RGB初出現の約100 ms前の過去側フレーム
    onset_after.png              # RGB初出現時刻以降の最初のフレーム
    candidate_before.png         # 候補時刻の直前
    candidate_after.png          # 候補時刻以降の最初のフレーム
    contact_sheet.png            # 上記4枚を2×2に配置
    frames.csv                   # 元／出力のフレーム番号と時刻対応
    summary.json                 # 使用動画、入力ハッシュ、候補情報
```

`before_onset.png` の目標時刻がクリップの先頭より前の場合は、クリップの最初のフレームを使用する。実時刻は画像とCSVで確認できる。
クリップにRGB初出現の前後フレームが入らない場合は停止するため、`--before-s` または `--after-s` を広げる。

## 見るところ

1. 候補直前・直後の枠内にRCカーがあるか。
2. 強いイベントがRCカーの輪郭に対応するか、段ボール端・床・別の背景に対応するか。
3. RGBとEVSの位置ずれが、対象の同定を妨げるほど大きくないか。

枠内に車両があるだけでは、そのイベント増加が車両由来と確定したことにはならない。前後の映像と輪郭を併せて確認する。
不明なケースは「不明」のまま残し、候補時刻を映像に合わせて動かさない。

## 誤った動画の使用を防ぐ検査

- 現在の注釈・選択される同期YAML・校正ファイルのハッシュが解析時と一致すること。
- 動画のEVS座標、解像度、投影法、変換行列、同期、RGBの時刻基準・原点が一致すること。
- まず注釈で使ったプレビューを優先し、なければ同じ条件を満たすRGBタイムライン動画を選ぶ。更新日時だけで選ばない。
- フレーム番号・元動画再生時刻・録画時刻の対応が連続し、実際の動画の解像度・fps・フレーム数と一致すること。
- 出力動画も復号し、フレーム数・再生fps・解像度が維持されていること。

元動画が `truncated: true`（要求した録画末尾まで生成されていない）でも、候補前後の指定区間が
`frames.csv` に含まれ、生成済み動画との整合が取れていれば使用する。末尾未生成の状態はログ・ページ・JSONへ記録する。
候補周辺そのものが不足している場合や、CSVと動画のフレーム数が異なる場合は引き続き停止する。
時間軸が `event` の動画はこのツールでは使用せず、エラーに実際の `timeline` と `truncated` の値を表示する。

同期や注釈が解析時から変わっていた場合は、古い候補を現在の動画へ重ねず停止する。候補の再解析とmanifestの更新が必要。
条件が一致する動画だけが見つからない場合は、対象4件の共通視野動画を生成してから実行する。

```bash
bash scripts/experiments/generate_rc_popout_common_views.sh \
  --record-root /workspaces/record/09-30/dynamic \
  --sessions test_01 test_05 test_11 test_14
```

## 検証範囲

合成動画4本でH.264生成と復号、4倍スロー、左右の枠、時刻対応、出力フレーム数を確認。
切り出し動画の録画時刻と再生時刻が異なるケース、同期・注釈・投影・フレーム対応の不一致検出、既存出力の保護をテストした。
元動画の末尾だけが未生成の場合の受け入れ、候補周辺が不足する場合の拒否、時間軸エラーとの区別もテストした。
候補manifestと元の解析CSVの一致もテストしている。
実データの動画生成はユーザーがLinux側で4件実行済み。受領したcontact sheetと候補直後PNGの[定性レビュー](rc_popout_candidate_visual_review_20261006.md)を行い、保存された候補manifest・各36行の時刻対応と整合することも確認した。
元録画全体の目視確認、RAW再抽出による過去2 ms窓の検証は未実施。

```bash
python3 -m unittest discover -s tests -p 'test_rc_popout_candidate_videos.py' -v
bash -n scripts/experiments/render_rc_popout_candidate_reviews.sh
```
