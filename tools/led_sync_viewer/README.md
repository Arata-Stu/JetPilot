# LED Sync Inspector

Docker内で次を実行します。

```bash
scripts/led_sync_gui.sh
```

Macのbrowserで <http://localhost:8765> を開きます。GUIの「Docker内JSON」へ
`/workspaces/record`配下の`led_sync_data.json`を入力すると、Docker内の解析データを
直接読み込めます。

既定ではlocalhostだけで待ち受けます。Dockerがbridge networkでhost側へportを公開する
必要がある場合のみ、container起動時に`8765:8765`を公開したうえで次を使います。

```bash
scripts/led_sync_gui.sh --host 0.0.0.0
```

serverが読み出せるのは、既定では`/workspaces/record`配下のJSONだけです。
