# JPBB-01 Firmware / Build / Flash

JPBB-01は、受信機によるRC操作とJetson Orin NanoからのUSB操作を安全に
切り替えるSTM32G0B1搭載基板である。ROS 2側の
`jetpilot_bridge_interface`と、基板側の`jetpilot_bridge_board` firmwareを
組み合わせて使用する。

## リポジトリの役割

| 対象 | JetPilot内の配置 | 役割 |
| --- | --- | --- |
| STM32 firmware・基板資料 | `hardware/jetpilot_bridge_board` | CH3切替、PWM入出力、USB通信、failsafe、watchdog |
| ROS 2 interface | `ros2_ws/src/vehicle/jetpilot_bridge_interface` | `/vehicle/control_cmd`とUSB JPB1プロトコルの変換 |
| 起動profile | `ros2_ws/src/launch/jetpilot_system_launch/config/bringup_profiles/vehicle/jpbb.json` | `--vehicle jpbb`の解決 |

firmwareの正本は
[Arata-Stu/jetpilot_bridge_board](https://github.com/Arata-Stu/jetpilot_bridge_board)
である。JetPilotへソースを複製せず、`packages.repos`から外部repositoryとして
取得する。これにより基板設計、firmware、JetPilotの更新履歴が混在しない。

## 1. FirmwareをJetPilotへ取得する

JetPilotのルートで実行する。

```bash
cd /path/to/JetPilot
export RC_AS_ROOT="$PWD"
vcs import < packages.repos
```

firmwareだけを取得したい場合は、次でもよい。

```bash
git clone --branch rev2-cost-optimized \
  https://github.com/Arata-Stu/jetpilot_bridge_board.git \
  hardware/jetpilot_bridge_board
```

以降の説明では次の環境変数を使用する。

```bash
export JPBB_ROOT="${RC_AS_ROOT}/hardware/jetpilot_bridge_board"
```

macOSまたはUbuntuでJetPilotを別の場所へcloneしている場合は、実際の絶対pathへ変更する。

## 対応する作業環境

同じCMake projectを使用するため、macOS、Ubuntu 24.04 x86_64、Ubuntu 24.04
arm64（Jetson Orin Nano）のいずれでも同じStepと生成物を扱う。`build/`は
OSやtoolchainごとに作り直し、異なるhost間で共有しない。

- firmwareのクロスビルドはhost OS上で行う。
- ROS 2はJetPilotのIsaac ROS container内でビルドする。
- ST-LINK／DFU書き込みはhost OSから行うのを基本とする。
- Jetson上でもfirmwareをビルドできるが、車両稼働中に書き込まない。

生成物のファイル名とFlash layoutは共通だが、Arm GNU Toolchainのversionが
異なればbinary hashまで同一になるとは限らない。実車releaseでは
`arm-none-eabi-gcc --version`を記録し、macOS／Ubuntu間でも同じtoolchain
versionを使用する。

## 2. Build toolを準備する

必要なものはCMake 3.22以降、Ninja、Arm GNU Toolchain、STM32CubeG0である。

### Ubuntu 24.04（x86_64／arm64）

```bash
sudo apt update
sudo apt install -y \
  git cmake ninja-build \
  gcc-arm-none-eabi libnewlib-arm-none-eabi \
  libusb-1.0-0-dev
```

Ubuntu 24.04では`gcc-arm-none-eabi`とnewlibを明示的に導入する。これは
`nano.specs`を使う最終linkまで同じ手順で通すためである。

### macOS

```bash
brew install cmake ninja
brew install --cask gcc-arm-embedded
```

toolchainを確認する。

```bash
cmake --version
ninja --version
arm-none-eabi-gcc --version
```

### Ubuntu 24.04へSTM32CubeProgrammerを入れる

[ST公式STM32CubeProgrammer](https://www.st.com/content/st_com/en/stm32cubeprogrammer.html)
からLinux版を取得する。Ubuntu 24.04 x86_64ではLinux installer、Jetsonなど
arm64では対応するArm 64-bit Debian packageのCLIを利用できる。

```bash
# arm64 Debian packageを取得した場合の例
sudo apt install ./stm32cubeprogrammer_*_arm64.deb
```

ST-LINKとUSB DFUを一般ユーザーから利用するため、インストール先の
`Driver/rules`にあるudev ruleをhostへコピーする。

```bash
cd /path/to/STM32CubeProgrammer/Drivers/rules
sudo cp *.* /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

USB CDC `/dev/ttyACM*`の利用者を`dialout`へ追加し、一度ログアウトして
ログインし直す。

```bash
sudo usermod -aG dialout "$USER"
```

STM32CubeProgrammerの公式Linux導入手順は
[Installing STM32CubeProgrammer](https://dev.st.com/stm32cube-docs/prog/latest/en/docs/markup/CubeProg_How_To_Start/CubeProg_Installation.html)
を参照する。

## 3. STM32CubeG0を取得する

STM32 HALとUSB Device Libraryはfirmware repositoryへ直接含めず、ビルド時に
STM32CubeG0 v1.6.3を参照する。

```bash
git clone --recursive --shallow-submodules --depth 1 --branch v1.6.3 \
  https://github.com/STMicroelectronics/STM32CubeG0.git \
  "${JPBB_ROOT}/firmware/vendor/STM32CubeG0"
```

すでにclone済みなら繰り返さない。

## 4. 最終Firmware（Step 8）をビルドする

```bash
cmake \
  -S "${JPBB_ROOT}/firmware" \
  -B "${JPBB_ROOT}/firmware/build/step8" \
  -G Ninja \
  -DJPBB_STEP=step8_final \
  -DSTM32CUBE_G0_PATH="${JPBB_ROOT}/firmware/vendor/STM32CubeG0" \
  -DCMAKE_BUILD_TYPE=Release

cmake --build "${JPBB_ROOT}/firmware/build/step8"
```

成功すると次の3ファイルが生成される。

```text
hardware/jetpilot_bridge_board/firmware/build/step8/step8_final.elf
hardware/jetpilot_bridge_board/firmware/build/step8/step8_final.hex
hardware/jetpilot_bridge_board/firmware/build/step8/step8_final.bin
```

通常はSTM32CubeProgrammerで`.hex`または`.elf`を書き込む。
macOSとUbuntuで生成物名、Flash layout、JPB1 protocolは同一である。

## 5. 初回はStep 1から確認する

未確認の基板へいきなりStep 8を書き込んで車両を動かさない。初回は次の順で
段階的に書き込み、各段階が合格してから進む。

| Step | `JPBB_STEP` | 主な確認 |
| --- | --- | --- |
| 1 | `step1_safe_boot` | MCU起動、起動時安全状態、ST-LINK再接続 |
| 2 | `step2_usb_cdc` | USB CDC認識、`PING`→`PONG` |
| 3 | `step3_rc_input` | CH1〜CH3 PWM入力 |
| 4 | `step4_pwm_output` | サーボ／ESC PWM生成 |
| 5 | `step5_manual_auto` | hardware MUX切替 |
| 6 | `step6_failsafe` | USB・VBEC・watchdog・RC喪失 |
| 7 | `step7_full_bridge` | USB指令による車両出力 |
| 8 | `step8_final` | CH3権限切替とJPB1通信を含む最終版 |

例えばStep 1は次のようにビルドする。

```bash
cmake \
  -S "${JPBB_ROOT}/firmware" \
  -B "${JPBB_ROOT}/firmware/build/step1" \
  -G Ninja \
  -DJPBB_STEP=step1_safe_boot \
  -DSTM32CUBE_G0_PATH="${JPBB_ROOT}/firmware/vendor/STM32CubeG0" \
  -DCMAKE_BUILD_TYPE=Release

cmake --build "${JPBB_ROOT}/firmware/build/step1"
```

基板の向き、J7 SWD pin、Option Bytes、各Stepの実測手順は
[jetpilot_bridge_board GETTING_STARTED](https://github.com/Arata-Stu/jetpilot_bridge_board/blob/rev2-cost-optimized/firmware/GETTING_STARTED.md)
を参照する。

## 6. ST-LINKで初回書き込みする

1. サーボ、ESC、受信機、モーターを外す。
2. J6 USB-Cから基板へ給電する。
3. ST-LINKのCortex 10-pin SWDをJ7へ接続する。J7 pin 1はVTrefであり、給電端子ではない。
4. STM32CubeProgrammerで`ST-LINK`／`SWD`を選択する。
5. 初回だけ`BOOT_LOCK=0`、`nBOOT1=1`、`nBOOT_SEL=0`を設定して読み戻す。
6. Step 1の`.hex`を書き込み、Verify後にRSTを押す。
7. Step 2でUSB CDC認識を確認し、その後Step 3〜8を順番に検証する。

Ubuntu 24.04ではGUIの代わりにCLIでも書き込める。Option Bytesを確認した後の
通常書き込み例は次のとおり。

```bash
STM32_Programmer_CLI \
  -c port=SWD freq=1000 reset=HWrst \
  -w "${JPBB_ROOT}/firmware/build/step1/step1_safe_boot.hex" \
  -v
```

Step 5以降はサーボやESCが動く可能性がある。タイヤを浮かせ、モーター線を
外し、電源を直ちに切れる状態で確認する。

## 7. 2回目以降はUSB DFUでも書き込める

初回Option Bytes設定後は、BOOTを押したままRSTを押して離し、その後BOOTを
離すとSTM32 ROM DFUへ入れる。CubeProgrammerで`USB`を選び、Step 8の`.hex`を
書き込む。DFUで認識しない場合はST-LINKへ戻す。

## 8. JetPilotから起動する

実車ではUSB serialを安定したby-id pathへ設定する。

```bash
ls -l /dev/serial/by-id/
```

確認したpathを次へ設定する。

```text
ros2_ws/src/vehicle/jetpilot_bridge_interface/config/jetpilot_bridge_interface_node.param.yaml
```

ROS 2 packageをビルド後、JetPilotルートから起動する。

```bash
# 対話画面でall、または必要なpackageを選択する
./scripts/build.sh

./scripts/bringup.sh drive --vehicle jpbb
```

packageをコマンドで指定する場合は、Isaac ROS container内で次を使用する。

```bash
cd /workspaces/ros2_ws
colcon build --symlink-install --packages-up-to \
  jetpilot_bridge_interface jetpilot_system_launch
source install/setup.bash
```

起動時はSTOPを維持し、CH3を2000 us付近のホスト許可側にしてから、Joy操作は
MANUAL、自律走行はAUTOを選ぶ。CH3を1000 us付近へ戻すとPROPO操作へ移る。

USBを抜いた場合、firmwareは安全PWMへ移行し、ROS 2側はSTOPを要求する。
再接続後も以前の指令は自動再開せず、STOPからMANUALまたはAUTOを明示的に
選び直して再アームする。

## 9. 実機投入前チェック

- ESCのニュートラルが1500 usであることを確認する。
- CH3が実測で約1000／2000 usになることを確認する。
- CH3中間値、受信機断、USB断、ROS指令停止、VBEC異常を個別に試す。
- `~/rc_channels`と`~/output_channels`をrosbagへ記録し、指令と実出力を比較する。
- USBを差し直しただけでは車両が再始動しないことを確認する。
- 実機試験が完了するまでタイヤを接地せず、モーターを接続しない。
