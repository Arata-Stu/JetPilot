# Isaac ROS 5.0 移行計画

更新日: 2026-09-23

## 方針と予定

**将来の開発予定に含める。現時点では更新せず、移行実装は未着手とする。**
当面は現行のIsaac ROS 4.6 / ROS 2 Jazzyを前提に開発を継続する。
着手時期は未定とし、現行環境での評価結果と移行に必要な依存関係が揃った段階で判断する。
この計画の追加では、コード、Docker、APT設定、`packages.repos`、依存lockを変更しない。

現行構成の経緯は[Isaac ROS CLI v4.6 移行監査](isaac_ros_cli_v46_migration.md)を参照。
本書は移行手順の実行指示ではなく、着手時に詳細化する調査・検証項目を記録する。

## JetPilot v2の方向性

**Isaac ROS 5.0への移行に合わせて、JetPilotをv1.0.0からv2.0.0へ進める方針とする。**
ROS 2の世代とGPUデータの受け渡し方式が変わり、実行環境・拡張コードの互換性に
影響するため、この移行をJetPilotのメジャーバージョン更新の節目にする。
Isaac ROSとバージョン番号を揃えるのではなく、JetPilot v2の基盤を
Isaac ROS 5.0 / ROS 2 Lyricalと位置付ける。

| JetPilotのバージョン | 位置付け |
| --- | --- |
| `v1.x.x` | 現行のIsaac ROS 4.6 / ROS 2 Jazzyでの開発・改善を継続する。 |
| `v2.0.0-alpha.1`以降 | Isaac ROS 5.0への移行開発版。依存関係とGPUデータ経路を段階的に移行する。 |
| `v2.0.0-rc.1`以降 | 移行実装が揃い、実機で最終評価するリリース候補版。 |
| `v2.0.0` | 本書の統合評価と採用判断を完了した正式版。 |

正式版の公開時期は未定。今は方向性の記録に留め、バージョン定義、Gitタグ、
外部リポジトリの参照は変更しない。移行着手時に各パッケージと外部リポジトリの
リリース単位を整理し、正式版では対応する依存バージョンを記録する。

## 公式情報で確認した主な変更

Isaac ROS 5.0.0は2026-09-21に公開された。対応ROSがLyricalへ移行し、
Ubuntu 24.04向けのIsaac ROS Buildfarmが追加されている。
GPUデータの受け渡しは`rosidl::Buffer`とCUDA buffer backendへ移行する。
リリースノートでは従来の`isaac_ros_nitros`、Managed NITROS、各NITROS型パッケージなどの
削除と、これらを直接使うコードのソース修正が必要であることが明記されている。

出典: [Isaac ROS 5.0 リリースノート](https://nvidia-isaac-ros.github.io/v/release-5.0/releases/index.html)、
[NITROSからrosidl::Bufferへの移行ガイド](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/rosidl_buffer/nitros_migration.html)。
移行ガイドには将来削除という表現も残るため、着手時は対象リリースの実際の配布物・APIも確認する。

## JetPilotで調査する範囲

以下は現行ソースの確認に基づく影響候補であり、5.0環境での動作は未検証。

| 対象 | 現行の依存と移行時の確認事項 |
| --- | --- |
| `tools/isaac-ros-cli` | JazzyのDockerレイヤー、`ros-jazzy-*`、NITROS型の明示導入を見直す。JetPilot固有の追加レイヤー、デバイス権限、マウント、CycloneDDS設定を新しい公式CLIと比較する。 |
| `jetpilot_e2e_inference` | EVS前処理、tensor encoder、latent state manager、control / trajectory decoderが`NitrosTensorList`、memory pool、CUDA stream APIを利用。新しいbuffer APIで所有権・寿命・同期・型・shapeを再確認する。 |
| `jetpilot_object_detection` | YOLOv8 decoderのNITROS tensor購読とCUDA同期を移行する。ReIDを含む後段出力への影響を確認する。 |
| 推論launch・モデル情報 | `input_tensor_formats` / `output_tensor_formats`と`nitros_tensor_list_nchw_rgb_f32`の扱いを確認する。学習側metadata、engine生成・配備との整合性も調べる。 |
| VSLAM / VGL / 地図作成 | パッケージ名、component、launch引数、topic・frame・QoS、既存地図の互換性を確認する。 |
| センサー・外部リポジトリ | RealSense、SilkyEvCam / OpenEB、車両interface、校正ツールなどのLyrical対応とビルド可否を確認する。 |
| Jetson / x86_64 | 対象ハードウェアに必要なJetPack、ドライバ、CUDA、TensorRTと配布パッケージの組合せを確認する。ホスト更新の要否もこの段階で判断する。 |

## 着手後の進め方

- [ ] **現行環境の基準を保存する。** 各リポジトリのcommit、コンテナimageの識別子、依存バージョン、モデル・engine・地図・設定を記録し、4.6へ戻せる環境を保持する。同一bagで推論出力、遅延、処理周期、GPUメモリ使用量を記録し、用途ごとの許容差を決める。
- [ ] **互換性調査を完了する。** 上表を対象に、5.0のAPI・パッケージ差分と外部依存の対応状況を確認し、変更一覧と未解決事項をまとめる。
- [ ] **独立した検証環境を用意する。** 移行用ブランチと別コンテナでCLI・ROS依存を更新し、x86_64 Linux / Jetsonでビルドする。Macでは文書編集と軽量な静的確認を行う。
- [ ] **GPUデータ経路を移行する。** E2E・EVS・YOLOv8の順に、buffer寿命、CUDA同期、intra-process経路、tensor契約を検証する。TensorRT engineは対象環境での再生成を計画し、旧engineの互換性を前提にしない。
- [ ] **統合評価を行う。** センサー単体、記録bag再生、VSLAM / VGL、推論、planning / controlの順に確認する。bag再生時の車両出力抑止、入力途絶時の停止、実機での制御・停止動作も確認する。
- [ ] **採用可否を判断する。** 合意した性能・出力の許容差を満たし、必要な機能と復帰手順が実機で確認できた段階で本番環境を切り替える。移行結果に合わせてセットアップ文書、launchガイド、各README、依存参照を更新する。

未解決の互換性問題や性能退行が残る場合は現行環境を継続する。
5.0の新機能追加は、既存機能の移行と回帰確認の後に個別に検討する。
