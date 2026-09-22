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

serverが読み出せるのは、既定では`/workspaces/record`配下の解析JSONと、そのJSONが
参照するpreview画像だけです。

## v1/v2の一括export

checkerboard校正sessionを除外し、`evs-popup-v1`と`evs-popup-v2`の各走行を順番に
処理します。既存の出力は自動的にskipされるため、途中で止まっても再実行できます。

```bash
scripts/export_led_sync_batch.sh
```

既定では各sessionの先頭・末尾12秒を20 fpsでpreview出力します。古いJSONを
preview付きで作り直す場合は`--force`を付けます。

LEDのROIが分かっている場合は、全画面より小さいROIを指定してください。

```bash
scripts/export_led_sync_batch.sh \
  --rgb-roi X,Y,W,H \
  --evs-roi X,Y,W,H
```

出力先は`evs-popup-vN/analysis/led_sync/<session>/`です。

GUIではタイムライン中央の橙線に近いRGB/EVS画像が並んで表示されます。「開始側」または
「終了側」を選び、両方の画像上でLEDをドラッグして囲めます。選んだ4つのROIは
「結果を書き出す」で保存されるJSONの`roi_selection`に含まれます。
