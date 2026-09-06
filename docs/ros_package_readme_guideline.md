# ROS package README guideline

JetPilot の first-party ROS 2 package（`jetpilot_*`）は、package の責務と静的な
ROS interface contract を `README.md` に記載します。実行中の ROS graph を確認しなくても、
レビュー時に接続関係、型、QoS、安全上の前提を追える状態を維持することが目的です。

## Required sections

node を持つ package は、原則として次の順序で section を配置します。

1. `# package_name`
2. `## Purpose`
3. `## Nodes`
4. `## Inputs / Outputs`
5. `## Parameters`
6. `## Assumptions / Known limits`
7. algorithm、safety、usage など package 固有の説明
8. `## How to launch`

message 定義または launch 専用 package のように node を持たない場合も、同じ見出しを置き、
該当しない interface には `なし。` と明記します。

## Inputs / Outputs contract

`## Inputs / Outputs` から次の level-2 heading までが機械可読領域です。topic table の
heading と列名は変更しないでください。

```markdown
## Inputs / Outputs

### Input topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `controller_node` | `/planning/trajectory` | `nav_msgs/msg/Path` | Reliable / Transient Local | 追従軌道 |

### Output topics

| Node | Name | Type | QoS | Description |
| --- | --- | --- | --- | --- |
| `controller_node` | `/auto/control_cmd` | `jetpilot_msgs/msg/ControlCommand` | Best Effort / Volatile | 自律指令 |

### Services

| Node | Name | Type | Direction | Description |
| --- | --- | --- | --- | --- |
| `manager_node` | `/localization/relocalize` | `std_srvs/srv/Trigger` | Server | 再測位を開始 |

### TF

| Node | Parent | Child | Mode | Description |
| --- | --- | --- | --- | --- |
| `localizer` | `map` | `odom` | Dynamic publish | global pose |
```

- `Name` は標準 bringup で解決される topic 名を記載します。
- node 内の相対名が launch で remap される場合は、`Description` に元の相対名を併記します。
- 設定で変化する名前は、標準値を `Name` に記載し、変更元の parameter/launch argument を
  `Description` に記載します。
- topic が複数の mode でだけ有効になる場合は、その条件を `Description` に記載します。
- `QoS` は最低でも reliability と durability を `Reliable / Transient Local` のように記載します。
- 同一 topic の型は全 package で一致させます。
- topic 名を1セルへ複数まとめず、1 topic を1行にします。
- topic を持たない場合は table の代わりに `なし。` と記載できます。

## Parameters

少数の parameter は `Name / Type / Default Value / Description` の表へ列挙します。多数ある場合は、
主要parameterと、そのpackageが所有する標準parameter fileへのリンクを示します。機体・course・
sensor構成固有の上書きは `jetpilot_system_launch/config` が所有します。

## Topic graph generation

全packageのREADMEからシステム図を更新します。

```bash
python3 scripts/generate_topic_graph.py
```

生成済みの `docs/topic_graph.md` が最新かだけを確認する場合:

```bash
python3 scripts/generate_topic_graph.py --check
```

生成器はPython標準ライブラリだけを使用し、同一topicのmessage型不一致と、README上で判定できる
reliability/durabilityのQoS非互換も検出します。図は全機能を重ねた静的なsupersetであり、launch
condition、namespaceの動的変更、実行時に外部packageが生成するtopicまでは解決しません。
実機状態の最終確認にはROS graphを使用してください。
