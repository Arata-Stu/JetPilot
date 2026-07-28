# JetPilot Kalibr runner

このディレクトリは、`multi_sensor_calibration export-kalibr`が生成した
データセットをROS 1 bagへ変換し、Kalibrのmulti-camera calibrationを実行します。
ROS 2、OpenEB、実センサーdriverはコンテナに含めません。

## Build

```bash
./tools/kalibr/build.sh
```

既定ではKalibrの`master`をbuildし、解決されたcommit SHAをimage内へ記録します。
再現性のためcommitを固定する場合:

```bash
KALIBR_REF=<commit-sha> ./tools/kalibr/build.sh
```

Apple Silicon上でamd64 imageが必要な場合は、build時と実行時に
`DOCKER_DEFAULT_PLATFORM=linux/amd64`を設定してください。実運用ではJetPilotの
Linux Docker環境でのbuild・実行を推奨します。

```bash
DOCKER_DEFAULT_PLATFORM=linux/amd64 ./tools/kalibr/build.sh
```

## Calibration

```bash
./tools/kalibr/calibrate.sh \
  result/kalibr_dataset \
  result/kalibr_output
```

Apple Siliconでamd64 imageを使う場合:

```bash
DOCKER_DEFAULT_PLATFORM=linux/amd64 ./tools/kalibr/calibrate.sh \
  result/kalibr_dataset \
  result/kalibr_output
```

入力はread-only mountされます。出力には次が作成されます。

- `kalibr.bag`
- `kalibr-camchain.yaml`
- `kalibr-results-cam.txt`
- `kalibr-report-cam.pdf`
- `kalibr.log`
- `run_metadata.json`

`run_metadata.json`には実際に使用したKalibr commit SHAと成否を記録します。
