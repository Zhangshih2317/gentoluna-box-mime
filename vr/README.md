# Luna 的 PICO 遥操、1080p 视频和腕部双夹爪

v0.3.0，2026-11-21。适用于已有 Gento Luna 厂家镜像、标定和 Apex 的 Orin。现场操作者已确认左右手柄可控制实体夹爪。[环境与原厂下载](SOURCES.md) · [开发记录与故障定位](HISTORY.md) · [夹爪协议和历次实测](wrist-grippers/README.md)

## 1. 接线和地址

| 设备 | 连接和用途 | 验收 |
|---|---|---|
| Windows PC | 网线直连 Luna 的对外以太网，同网段一个空闲静态地址，如 `6.6.7.101/24`，直连网卡无需默认网关 | `Find-NetRoute -RemoteIPAddress 6.6.7.100` 命中以太网；本地地址状态为 `Preferred` |
| PICO | 支持数据的 USB 线接 PC，开启开发者模式及 USB 调试，头显中授权。Wi‑Fi 可以留给投屏 | `adb -d get-state` 返回 `device` |
| Orin | 对外地址 `6.6.7.100`，内部运动控制器为 `6.6.7.190`。Apex 主机填前者 | `Test-NetConnection 6.6.7.100 -Port 8080` 成功 |
| D435 | USB 3 接 Orin，选彩色 V4L2 节点 | `v4l2-ctl --list-devices` 和 `--list-formats-ext` 可见 1920×1080、YUYV、30 fps |
| 两只 CTAG2F90D | 各自插在臂末端 8 针，左 `ARM0/485A`、右 `ARM1/485A`。按厂家手册核对线束和供电 | Robot 日志中两侧各有 `passed read-only preflight` |

## 2. 建立 PICO 的 USB 链路

Windows 安装 [Node.js](https://nodejs.org/en/download) 后解压本项目。在 PowerShell 中执行。首次安装官方 gnirehtet APK 时保持头显已解锁。

```powershell
cd vr\windows
.\Install-UsbTools.ps1
.\tools\platform-tools\adb.exe devices -l
.\tools\platform-tools\adb.exe -d install -r .\tools\gnirehtet-v2.5.1\gnirehtet-rust-win64\gnirehtet.apk
.\Start-PICO-USB-Sharing.ps1 -LunaEthernet
```

首次在 PICO 允许 VPN，然后再运行一次启动脚本。可以用 `-DeviceSerial` 指定已授权的 USB 设备。脚本为 `6.6.7.100/32` 建立隧道，启动反馈转发。退出时运行 `.\Stop-PICO-USB-Sharing.ps1`。首次安装的 PICO 应从[依赖清单](SOURCES.md)下载 **pxr** 版 Apex APK，通过相同的 `adb -d install -r <APK>` 安装。

```powershell
.\tools\platform-tools\adb.exe -d shell ip -4 addr show tun0
.\tools\platform-tools\adb.exe -d reverse --list
Get-Content .\pico-feedback-state.json
```

期望 `tun0` 为 `10.0.0.2/32`，反向映射包含 `gnirehtet`，反馈状态为 `connected`。头显打开 Apex，选择控制器模式，主机 `6.6.7.100`。PC→USB→头显的反馈由本项目将 Luna 的 UDP `9002/9003` 经 ADB TCP `39002` 送到头显本地；控制 TCP `9010`、视频信令 `8554`、WebRTC 媒体 UDP 分别诊断。每次 PC 或 PICO 重启后重建隧道。

## 3. 恢复 1080p 彩色视频

将整个 `vr` 目录复制到 Orin，如在项目根目录执行 `scp -r vr nvidia@6.6.7.100:~/luna-vr-release`（首次目标目录不存在）。下列命令以 `~/luna-vr-release` 为根目录。第一次 SSH 登录应在现场核实设备指纹。先在 Apex 退出相机采集，释放其他 D435 预览程序。

```bash
cd ~/luna-vr-release
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video12 --list-formats-ext
mkdir -p /home/nvidia/d435-preview
sudo install -d /etc/systemd/system/apex-camera.service.d
cfg=/etc/systemd/system/apex-camera.service.d/60-d435.conf
backup=$(mktemp -d "$HOME/camera-before-vr.XXXXXXXX")
test ! -f "$cfg" || cp "$cfg" "$backup/60-d435.conf"
test ! -f ~/d435-preview/apex-color1080.yaml || cp ~/d435-preview/apex-color1080.yaml "$backup/"
echo "$backup"
install -m 0644 d435-preview/apex-color1080.yaml ~/d435-preview/
sudo install -m 0644 d435-preview/apex-color1080-override.conf "$cfg"
sudo systemctl daemon-reload
```

如果彩色节点不是 `/dev/video12`，把已安装 YAML 中两处 `v4l2:///dev/video12` 改为实测节点。用 Apex 启动 Camera，再在 PICO 打开空间视频，选择 Mono。房间 `10` 给头显，`11` 给桌面。每个房间用一个接收端。画面为 H.264 四宫格 3840×2160，左上为原生彩色 1920×1080，其他三格为空。深度采集未启用。编码目标 8 Mbps、30 fps。

```bash
source /etc/apex/apex_ros_env.sh
python3 ~/luna-vr-release/d435-preview/check_apex_frame.py
journalctl -u apex-camera -n 50 --no-pager
```

该探针验证 Orin 本地 ROS 压缩图像。确认 PICO 媒体走线时，在 PICO 用 `adb -d shell ip -4 addr show wlan0` 读出当前无线地址，在 Orin 用 `ip -br addr` 确认接口。佩戴头显播放至少 30 秒，分别观察有线口发往 PC 的视频 UDP 和无线口发往 PICO 的视频 UDP，例如 `sudo timeout 30 tcpdump -ni eth0 'udp and dst host 6.6.7.101'`，另一终端对实际无线接口与头显地址做同样采样，按视频流的端口和流量区分反馈。信令握手成功不能判定媒体接口。

如媒体仍选无线，核实该地址确属头显，记录 `systemctl show apex-camera.service -p IPAddressDeny` 原值后，在现场无线地址为 `10.42.0.141` 的示例中执行 `sudo systemctl set-property --runtime apex-camera.service IPAddressDeny=10.42.0.141/32`，重新进入视频并复测。此设置在重启后失效；修改前已有其他策略时保留其条目。头显继续连接 Wi‑Fi，投屏接收端在头显内选择。

## 4. 构建、检查和安装腕部夹爪

使用厂家 Orin 镜像和本版本依赖。在 Orin 新终端执行以下命令。`C_SDK` 为固定版本的厂家源码，下载方式见[环境表](SOURCES.md)。`factory.sha256` 的两个文件必须匹配。工具链目录中有 `usr/bin/aarch64-linux-gnu-g++-11` 等文件；若板载编译器已匹配，可省略 `build_sdk.sh` 的最后一个参数。

```bash
source /etc/apex/apex_ros_env.sh
set -e
cd ~/luna-vr-release/wrist-grippers
sha256sum -c apex/factory.sha256
work=$(mktemp -d "$HOME/luna-vr-build.XXXXXXXX")
cmake -S verify -B "$work/tests" -DCMAKE_BUILD_TYPE=Release
cmake --build "$work/tests" --target protocol_test manual_test ros_adapter_test wrist_manual -j2
ctest --test-dir "$work/tests" -j3 --output-on-failure
sdk="$HOME/Gento-SDK/SDK_CT040404_SDK040402_260617/SDK/00040402/C_SDK"
bash apex/build_sdk.sh "$sdk" "$work/sdk" "$HOME/luna-gcc11"
cmake -S apex -B "$work/sdk/cmake" \
  "-DCMAKE_CXX_FLAGS=-ffile-prefix-map=$(pwd -P)=/home/nvidia/apex-wrist-recovery.hhe03VYK"
cmake --build "$work/sdk/cmake" -j2
echo "2dc21fc7b1e9ab53c83b854d8ecdb23fb041e5c8d8e23e77edad82f36683f6c9  $work/sdk/cmake/libGentoSDK.so" | sha256sum -c -
```

应看到 5 项测试通过，构建日志出现 `Matched stock SDK code, constant data, initialized data and exported symbol sizes`。最后的路径映射固定 ROS 日志中嵌入的源码路径，可逐字节复现现场库。发布时在 Orin 的独立目录验证了这一结果。三个 ROS 测试使用本地隔离域 93。安装前按正常现场流程退出遥操、释放控制器并停用 Robot、Teleop、Tool。首次安装在同一终端执行。

```bash
sudo bash apex/install.sh "$work/sdk/cmake"
install -D -m 0755 "$work/tests/wrist_manual" ~/luna-wrist-manual/wrist_manual
systemctl cat apex-robot apex-tool
```

已有本项目安装的设备先核对已加载库。如果需要重新安装，停用上述三个模块，备份 `/usr/local/lib/luna-apex-wrist` 与两个 `70-luna-ctag.conf` 到带时间的目录，再将这些旧路径移出安装位置并 `daemon-reload`，保留备份以便回退。安装器会拒绝覆盖已有安装。正常运行时 Robot 独占控制器会话，Tool 的租约启停适配器；夹爪收发均在同一 Robot 进程。不要再启动独立 SDK 客户端抢占控制器。

## 5. 启动和验收

1. 头显连上 `6.6.7.100`，确认跟踪正常。按厂家流程完成机器人准备和 Home，在 Apex 正常启动 Robot、Teleop、Tool 和 Camera。保持手柄靠近当前腕位，侧握同时启用该侧机械臂跟随。
2. 用 `journalctl -fu apex-robot` 等待左右两条 `passed read-only preflight`。握左侧握并完全松开食指扳机，按过 80% 闭合，松至 20% 张开。右手同样操作。速度和扭矩默认各 60%，张开目标 `0`，闭合目标 `9000`。
3. 在加载 Apex ROS 环境的终端运行 `ros2 topic echo /info/wrist485_L` 或 `_R`，依次是实测位置、速度、状态字、故障 A、报警 B。日志 `requested` 表示收到手势，`command acknowledged` 表示驱动回执。重复两侧开合并观察实物位置。退出、重进遥操后，先松开扳机再按。
4. 复测视频时记录帧数增量、接收字节增量和采样时长，按 `帧差/秒`、`字节差×8/秒/10^6` 计算。历史 PICO 有线实测 820 帧 / 30.81 秒，26.61 fps、7.55 Mbps。[头显采样](outputs/20261120-pico-usb-native1080.json)

单侧故障立即暂停该侧，在第 3、6、9、12、15 秒只读复查，任一轮健康就恢复接收新手势。第五轮仍失败才锁定该侧。可能仍在运动时先请求停止。仅报警 B 掉落位 `0x1000` 单独出现时继续，其他故障和停止确认继续生效。详见[回读和故障时间线](HISTORY.md)。

## 维护和回退

- 夹爪回退见[服务附加配置的命令](wrist-grippers/README.md#回退)。需要手动维护时先释放控制器。可在 Orin 执行 `source /etc/apex/apex_ros_env.sh` 后运行 `~/luna-wrist-manual/wrist_manual status both 60 60`。PC 可选 `python -m pip install paramiko`，以已核实的 `known_hosts` 和本人 SSH 凭据运行 `python control_grippers.py status`。
- 相机回退时先退出采集，将备份的 YAML 与 `60-d435.conf` 放回原处。原先没有此附加配置时，将新增 `.conf` 移到备份目录；执行 `sudo systemctl daemon-reload`，再用 Apex 启动 Camera。如果使用临时无线过滤，恢复到设置前记录的值。
- PC 视频接收器诊断可选 `python -m pip install playwright` 和 `python -m playwright install chromium`。关闭占用同房间的头显客户端，在 `vr` 目录执行 `python d435-preview/check_vr_receiver.py "<Apex安装目录>/dist/fullscreen.html" 6.6.7.100 frame.jpg --room 10 --seconds 30`。这项检查测的是 PC 接收端。
