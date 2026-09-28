# 会場間でのHDMap再利用

練習会場で調整したHDMapと走行ラインを、新会場で作り直したVSLAM地図に
XY平行移動とヨー回転で合わせる。コース形状と縮尺が共通であることを前提とする。
背景の特徴点まで同じである必要はない。

## Consoleでの操作

1. 練習会場側のHDMap編集、Section、走行ライン、速度設定を保存する。
   保存済みバージョンを使いたい場合は、そのバージョンを元の地図で選択しておく。
2. 本番会場でVSLAM地図とlandmark rasterを作り、新しい地図としてConsoleで開く。
3. 「Geometry」の「編集」内で「別会場のHDMapを再利用」を開き、練習会場の地図を選ぶ。
4. X/Y移動量（m）と反時計回りの回転（度）を入力する。
   または「同じ基準位置・向きから計算」に、同一スタート枠などの基準姿勢を
   両方の地図座標で入力して変換を求める。
5. 「位置合わせをプレビュー」で新会場のlandmark raster上の水色の形状を確認する。
   原点付近だけでなく、コース全体の壁・分岐・狭路も合わせる。
6. 「この位置で保存」で新会場の座標に変換したHDMapと走行ラインを保存する。
   必要なら新会場側の通常編集で設営差を補正し、従来の転送・実走行フローへ進む。

設定変更後は再度プレビューする。元地図または適用先の保存データがプレビュー後に
変わった場合、保存は拒否される。元地図は変更しない。再度位置合わせするときも
元地図から変換するため、前回の変換は累積しない。

## 座標契約

`p_new = R(yaw) * p_old + [x_m, y_m]`

回転中心は元地図の原点。基準姿勢を使う場合は
`yaw = target_yaw - source_yaw`、`t = target_xy - R(yaw) * source_xy`。
これは `new_map <- old_map` のSE(2)変換に相当する。

実装はTFを追加せず、変換後の座標を既存の `map` frameの成果物へ保存する。
現在のplanner・controller・section localizerは同一world frameを前提としているため、
VSLAMの `map -> odom`、車両の初期姿勢、landmark地図自体は変更しない。
`frame_id_override` だけで座標を読み替える操作もしない。

変換対象:

- 全レーンの左右境界、走行可能境界、centerline、network raceline
- Section Gate、信号/分岐位置、静的障害物
- centerline CSV、raceline CSV、カスタムラインの編集点と軌道CSV
- 走行ルート設定内の地図ファイル参照と、座標変更に伴う整合性hash

距離station、幅、Z、曲率、速度、加速度、Section設定、レーン接続・IDは維持する。
軌道CSVはXYとheadingだけ変更し、最適化や速度プロファイル生成をやり直さない。
元のカスタムラインが不正な場合は、修復してから再利用する。
旧会場のraster・VSLAM/VGL・odometry snapshot・preview画像・HDMap版履歴はコピーしない。
適用先の古いpreview画像とactive版指示は無効化する。版履歴そのものは保持する。

適用履歴と変換は `hd_map_registration.json`、置換前のファイルは
適用先内の `.hd-registration-backup-*` に保存する。復元は走行停止中に、
バックアップ内の同名ファイルを適用先の同じ相対位置へ戻す。
バックアップに存在しない新規作成ファイルは、復元時に別途取り除く必要がある。

## 限界と運用

これは地図編集時の操作で、走行中の地図切替機能ではない。
複数成果物を更新するため、関連ノードを停止して適用し、走行時に読み直す。
Console内のmap生成・解析とはsource/target両方のresource lockで排他する。
書込み失敗時は置換済みファイルをバックアップから戻すが、電源断に対する
複数ファイル全体のatomic transactionではない。

コースが同形でも、設営誤差・VSLAMのドリフトや局所歪み・床面の傾きにより
SE(2)だけで一致しない場合がある。縮尺変更・鏡映・局所変形・自動点群照合は行わない。
背景特徴点による会場間の自動対応付けにも依存しない。

## 検証

```sh
PYTHONPATH=tools/app/backend python3 -m unittest discover -s tools/app/backend/tests -p 'test_map_registration.py'
node --test tools/app/frontend/tests/map_registration.test.cjs
```
