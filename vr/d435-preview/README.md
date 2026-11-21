# Luna D435 独立预览

本页按开发顺序记录 8761 预览、RGB-D、60 FPS 和 1080p 实验。交付配置为最后一节的原生 1080p 彩色，请按[整套系统安装指南](../README.md)部署。原始截图留存在所有者私有归档。

地址：http://6.6.7.100:8761

直接调用机器现有环境中的 pyrealsense2，不调用或修改 8760 的授权逻辑，不调用机器人运动接口。

## 使用

- 网页显示彩色与对齐到彩色坐标的深度图，640×480，目标 30 FPS。
- “停止相机”释放相机采集资源；“启动相机”重新连接 D435。
- 另一个程序需要使用 D435 时，先在本页面停止相机，等待状态显示“已停止”。
- 关闭浏览器不会停止采集。
- 深度伪彩色显示范围为 0–4 米，黑色表示无效深度。
- depth.png 是 16 位原始深度图，乘以 /api/status 中的 depth_scale_m 得到米。独立下载的彩色与深度文件不保证来自同一帧。

## 接口

| 路径 | 内容 |
| --- | --- |
| GET /api/status | 状态、帧数、帧龄、FPS、深度单位 |
| GET /color.mjpg | 彩色视频 |
| GET /depth.mjpg | 深度伪彩色视频 |
| GET /color.jpg | 当前彩色图 |
| GET /depth.jpg | 当前深度伪彩色图 |
| GET /depth.png | 当前 16 位深度图 |
| POST /api/start | 启动采集 |
| POST /api/stop | 停止采集 |

POST 需带 X-Camera-Control: 1；浏览器请求限制同源。此服务无登录认证，仅部署在当前机器人局域网，不应直接暴露公网。

## 部署与运行

机器人目录：/home/nvidia/d435-preview

用户级服务：luna-d435-preview.service

```sh
systemctl --user start luna-d435-preview.service
systemctl --user stop luna-d435-preview.service
systemctl --user status luna-d435-preview.service
```

目前已启动，没有设置开机自动启动。

## 验证

2026-11-11：SDK 识别 Intel RealSense D435；采集稳定时约 29.8 FPS。电脑直连 HTTP 获取状态、彩色 JPEG、深度 JPEG、16 位深度 PNG 成功；停止与重新采集状态验证通过；人工查看下载彩色图确认真实画面。Codex 内置浏览器报访问失败，未完成该浏览器的页面布局验证。

## 第一阶段：接入 Apex 彩色

已按用户要求切换为 Apex 原生采集。独立 8761 服务目前停止，避免争用 D435。

- Apex Camera 原生程序通过 V4L2 采集 /dev/video12 的 D435 彩色通道，640×480、30 FPS。
- 参数文件：/home/nvidia/d435-preview/apex-d435.yaml。
- systemd 附加配置：/etc/systemd/system/apex-camera.service.d/60-d435.conf；原厂 service 和原厂 YAML 未改。
- Apex 正常启停 Camera 会使用此配置。未开启 Camera 开机自启。
- 当前保留原生四宫格布局，左上角是彩色画面，其他三路为空。没有套用原 CSI 相机的畸变参数。未接入深度画面或构造双目彩色画面。
- 实测 /api/v1/modules/camera/status 返回 active/running、healthy=true；/quad_tile/compressed 连续接收 30 帧，约 30.09 FPS；已查看保存帧确认真实画面。
- WebRTC 房间 10、11 均已发布；尚未在 Apex 桌面客户端或 PICO 内完成接收端显示验证。
- 当前 /dev/video12 是实机验证过的彩色节点；若重启或调整 USB 设备后编号变化，需要重新核对该节点。

采集路径：D435 彩色 → V4L2 → Apex quad_csi_quickview → ROS 压缩图像 / 原生 WebRTC。

恢复原 CSI 配置时，应先停止 apex-camera.service，仅移除上述 60-d435.conf 附加配置，再执行 systemctl daemon-reload。要使用 8761 独立 RGB-D 预览，先停止 Apex Camera，再启动 luna-d435-preview.service；不要让两者同时打开彩色设备。

## 2026-11-11 Apex 彩色和深度

用户要求补入深度后，已切换为一个 RealSense SDK 管线同时采集 RGB 和深度，并将深度对齐到彩色。当前 60-d435.conf 启动 apex-rgbd.launch.py，由 Apex Camera 服务统一管理 SDK 采集、两路编码、原生信令服务及原生视频程序。

- 左上：D435 RGB；右上：D435 DEPTH 0-4m；下方两路保留空白。
- 当前彩色采集 1920×1080、深度原生采集 1280×720，SDK 配置帧率均为 30 FPS。对齐后的深度可视化尺寸为 1920×1080，不代表新增原生深度细节。两路画面叠加同一 SDK 帧组的本地计数，便于比较。传输后不保证严格同时显示。
- 深度显示为伪彩色，0–4 米固定映射，超过 4 米饱和，无效深度显示黑色。此视频是深度可视化，不能作为 16 位原始测距数据使用。
- rgbd_bridge.py 是唯一相机采集者，通过本机 RTP 5600/5602 送入 Apex；独立 8761 预览保持停止。
- 原生 WebRTC 房间 10/11 继续发布四宫格；左右图为 RGB 与深度，不是左右眼双目彩色画面。
- 当前配置文件为 apex-rgbd.yaml，启动文件为 apex-rgbd.launch.py。原彩色配置保留，远端附加配置备份为 apex-color-only.backup。
- 运行统计保存到 /home/nvidia/d435-preview/rgbd-status.json；读取时同时检查 updated_unix 的新鲜度。

验证：已实际查看 Apex 输出的两路画面；连续接收 30 帧约 29.64 FPS；相机服务重启后再次收到 30 帧约 30.68 FPS，API 返回 healthy=true。测试时有效深度像素约 99%，该比例随场景变化。没有发送运动指令。尚未验证桌面 Apex/PICO 的最终显示或端到端延迟。

上述约 30 FPS 是此前 640×480 档位的结果。按用户要求切换最高分辨率后，SDK 实际启用档位已读回确认：彩色 1920×1080@30，深度 1280×720@30。Apex 四宫格输出 JPEG 实测为 3840×2160；两次连续 30 帧测试分别约 11.07、10.05 FPS，桥接近期处理约 17.6 FPS。配置帧率不等于处理链路实测帧率。已目视核对最高分辨率输出，服务保持 healthy=true。当前保持最高分辨率，未声称达到 30 FPS 的最终输出。

最高分辨率档位使用两路各 10 Mbps 的本机中间编码、20 Mbps 原生输出配置。远端 rgbd_bridge.py.vga-backup、apex-rgbd.yaml.vga-backup 保存了此前低分辨率版本。

恢复彩色单路可将 apex-camera-override.conf 安装回 /etc/systemd/system/apex-camera.service.d/60-d435.conf，执行 systemctl daemon-reload 并重启 apex-camera.service。不要同时启动独立预览与 Apex 采集。

## 2026-11-20 PICO 热点视频

PICO 连接 `现场无线热点` 时，Apex 主机填 `10.42.0.1`。在头显主面板打开「空间视频」，这台 D435 的彩色和深度适合用 Mono 查看。视频信令端口为 8554，PICO 使用房间 10，桌面用房间 11。每个房间支持一个接收者。

原先的 3840×2160、20 Mbps 配置下，电脑从房间 10 收到约 9.8 MB，测试期间解码 0 帧。调低 `apex-rgbd.yaml` 中的输出参数后，两房间合成视频均为 1920×1080，编码目标 4 Mbps。D435 彩色仍采集 1920×1080，深度仍采集 1280×720。电脑从同一房间解码 23 帧，保存的现场画面位于 `../outputs/20261120-vr-room10-received.jpg`。本次验证终点为电脑，PICO 内的最终显示需要佩戴确认。

头显退出空间视频后可用电脑复查房间 10。在有 Python Playwright 和 Chromium 的电脑上运行 `python check_vr_receiver.py "<Apex 安装目录>/resources/app/dist/fullscreen.html" 10.42.0.1 room10.jpg`。运行结束后该接收端释放房间。机器人上调整前的配置保存在 `/home/nvidia/d435-preview/apex-rgbd.yaml.pre-vr-20261120-1795158893`。

两只实体夹爪使用手臂末端 8 针 RS-485，左爪 `ARM0/485A`、右爪 `ARM1/485A`，与本机旧 USB 接线记录不同。依据是 `MdZenus/gentoluna-box-mime` 提交 `37a7068` 中的 `deliverables/gentoluna-gripper-ui-delivery.zip`。现场 Apex Tool 使用 DM 驱动。XR 驱动订阅两个手腕位姿，发往头显的 `PoseGripMsg` 包含位姿、振动和底盘坐标，没有夹爪实际开度。两个腕部相机槽位均为 `none`。夹爪通信和腕部视频需要各自的输入。

## 2026-11-20 单路彩色高帧率

当前机器人使用 `apex-color60.launch.py` 和 `apex-color60.yaml`。GStreamer 在 D435 彩色 USB 接口协商 `960×540 YUY2 @ 60 FPS`，硬件编码后通过本机 RTP 送入 Apex。相机参数已读回验证。深度采集、深度对齐和深度编码均已关闭。Apex 合成帧为 1920×1080、目标 4 Mbps，左上为彩色，其余槽位为空。

同一电脑通过热点订阅 PICO 使用的房间 10。Apex 原生 V4L2 输入只协商到 30 FPS，30.36 秒解码 912 帧，平均 30.04 FPS。切换到显式 60 FPS 彩色采集后，30.37 秒解码 1,318 帧，平均 43.40 FPS。接收器统计窗口内有一次冻结，解码丢帧计数增量为零。头显本机显示帧率待佩戴确认。

复测时在 `check_vr_receiver.py` 原命令末尾加 `--seconds 30`。本次高帧率测试的截图和逐秒记录分别为 `../outputs/20261120-vr-room10-color60.jpg` 和 `../outputs/20261120-vr-room10-color60-rate.txt`。

需要恢复时，将 `/home/nvidia/d435-preview/apex-camera-override.conf`（单路彩色 30 FPS）或 `/home/nvidia/d435-preview/dropin-rgbd-before-color-20261120-153122.conf`（本次切换前的彩色和深度配置）安装到 `/etc/systemd/system/apex-camera.service.d/60-d435.conf`，执行 `systemctl daemon-reload` 并重启 `apex-camera.service`。切换会中断视频。

## 头显连接期间的延迟诊断

PICO 正在订阅房间 10 时，同一热点上的电脑用房间 11 做了 20.23 秒对照测试。电脑解码平均 0.3 ms，WebRTC 抖动缓冲平均 305.3 ms，并出现两次冻结。对齐两机系统时钟后，末尾一次电脑截图保存时距离画面叠加的机器人时间戳约 3.28 秒。这是双接收端场景下的一次卡顿样本。同期机器人到 PICO 的 16 次网络往返平均 120.8 ms，最高 303.1 ms。PICO 屏幕端的呈现延迟尚未测量。

上述双接收端测试使用 1920×1080 合成输出、4 Mbps。原始数据保存在 `../outputs/20261120-latency-before.txt`。

PICO 断开后已启用 `apex-color60.yaml` 的 1280×720、2 Mbps 合成输出，彩色单格为 640×360。相机采集仍为 960×540、60 FPS。只重启了 `apex-camera.service`。切换前的活动参数备份在机器人 `/home/nvidia/d435-preview/apex-color60.yaml.before-2mbps-20261120-155142`。恢复时将该文件复制为同目录 `apex-color60.yaml`，再重启相机服务。

电脑通过同一热点和房间 10 先后测量，PICO 全程离线。第一轮开始前桌面端还有一个视频信令连接，重启后新一轮只看到测试接收端，两个窗口的并发负载可能不同。

| 实测指标 | 1080p / 4 Mbps | 720p / 2 Mbps |
| --- | ---: | ---: |
| 采样时长 | 20.29 秒 | 30.39 秒 |
| 解码帧率 | 44.61 FPS | 46.17 FPS |
| 平均抖动缓冲 | 43.4 ms | 40.6 ms |
| 收到帧至预计显示的中位时间 | 61.8 ms | 49.3 ms |
| 接收器冻结次数 | 2 | 1 |

两次末帧截图分别抽查了从机器人叠加时间戳到电脑保存截图的耗时。用 SSH 往返样本估算机器人时钟领先电脑约 887.58 秒，两张截图的画面年龄分别约 120 ms 和 210 ms。单张截图、播放缓冲和接收帧率均不能代替 PICO 内的端到端延迟分布。原始数据和截图在 `../outputs/20261120-room10-4m-ab.*`、`../outputs/20261120-room10-2m-ab.*`。

## 2026-11-20 有线与 USB 下的原生 1080p

当前相机服务的 `60-d435.conf` 使用 `apex-color1080-override.conf`，加载 `apex-color1080.yaml`。D435 彩色 V4L2 实际采集 1920×1080、30 FPS。Apex 四宫格为 3840×2160，彩色占左上 1920×1080，编码目标 8 Mbps。上一档 720p 配置保留在机器人 `/home/nvidia/d435-preview/apex-color60-override.conf` 和 `apex-color60.yaml`。

电脑有线临时地址为 `6.6.7.200/24`，Luna 为 `6.6.7.100`。此前电脑地址 `6.6.7.101` 被 Windows 标记为重复。恢复的 `6.6.7.200` 写入 Windows ActiveStore，重启后需核对。`Start-PICO-USB-Sharing.ps1 -LunaEthernet` 使用原 ADB/Gnirehtet 工具，通过 USB 为 `6.6.7.100/32` 建立路由，同时启动 9002/9003 反馈转发。关闭 PICO Wi-Fi 后，实测头显只有回环和 `tun0` 两个 IPv4 接口，机器人有线口也观察到发往电脑的视频 UDP 数据。

PICO 原生 Apex 房间 10 在 30.81 秒内解码 820 帧，平均 26.61 FPS。接收视频码率平均 7.55 Mbps，采样范围 5.40～12.46 Mbps，窗口内丢帧与丢包增量均为零；头显最后一次报告的抖动缓冲为 34.6 ms。另用两次 8 MiB HTTP 下载测得经 USB 隧道的有效吞吐约 157 和 174 Mbps。原生头显样本见 `../outputs/20261120-pico-usb-native1080.json`。先前 720p 测试期间信令走 USB、视频仍走 Wi-Fi，其记录为 `../outputs/20261120-pico-720p-wifi-media.json`。

### 同时使用 PICO 无线投屏

现场要同时使用无线投屏和 USB 视频。重连时保持头显 Wi-Fi 开启，Apex 的主机填 `6.6.7.100`。此前关闭 Wi-Fi 是为了验证传输路径，单独关闭 Wi-Fi 会中断无线投屏。

在 Orin 确认 `10.42.0.141` 属于这台 PICO 的当前 Wi-Fi 地址后，给 `apex-camera.service` 设置了运行时 `IPAddressDeny=10.42.0.141/32`。这只限制相机服务与该无线地址通信。PICO 在两个接口同时在线时已连上 `现场无线热点`，经 USB 可连接 `6.6.7.100:8554`，经无线连接 `10.42.0.1:8554` 超时。投屏接收设备由操作者在头显内选择，本次未验证接收端实际画面。

机器人重启后临时规则失效。头显无线地址改变时应先复核地址归属。恢复 Apex 纯无线视频时，在确认没有新增其他拒绝规则后执行 `sudo systemctl set-property --runtime apex-camera.service IPAddressDeny=`。无需重启相机服务。
