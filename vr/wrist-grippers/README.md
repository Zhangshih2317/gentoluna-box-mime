# Luna 8 针腕部夹爪适配

2026-11-21 操作者确认 PICO 控制实体夹爪可用。首次还原请从[系统安装指南](../README.md)开始。下文按当时记录保留各轮试验，机器人上的临时目录名供原机查档使用。Windows 手动入口为 `control_grippers.py`，机器人端为 `/home/nvidia/luna-wrist-manual/wrist_manual`。动作端到端时延仍待专门测量。

## 在 Apex 中使用

现场目视确认模式沿用左右手柄各自的侧握使能。启动预检后，扳机越过对应阈值会产生一次开合请求，持续按住不会重发。控制日志分别记录 `trigger ... requested` 和 `command acknowledged`，后者表示驱动应答了指令。普通位置寄存器 `0x0418` 丢回包时可继续处理新手势；驱动状态、目标参数读回、断连和保护停止仍单独校验。只有实测位置才会发布到腕部反馈话题。

按现场正常流程启动 **Robot、Teleop、Tool**，PICO 以控制器模式连接主机并进入遥操。首次试用时保持手柄在当前腕部对应位置，周围留出开合空间。侧握键也会启用该侧机械臂跟随。

1. 握住要控制一侧的侧握键，先完全松开食指扳机。
2. 食指扳机按过 80% 时闭合该侧，松到 20% 以下时张开。左手对应 `ARM0/485A`，右手对应 `ARM1/485A`。两侧独立。
3. 速度、扭矩均为 60%。仅报警 B 的 `0x1000` 单独出现时继续工作，其他故障依旧处理。重新连接或重新启用 Tool 后首先松开扳机。中途松开侧握会撤销该侧输入，对未完成的动作请求停止。

Apex 的原厂机械臂节点继续持有控制器会话。`apex-robot.service` 的 `70-luna-ctag.conf` 只为该服务选择专用 Gento SDK 4.4.2 构建，系统库和原厂机械臂二进制保留原文件。`apex-tool.service` 的同名附加配置通过本机文件锁启停腕部适配。专用 SDK 中的 485 接收函数供夹爪独占，原厂虚拟 CAN 的轮询不消费腕部回包。独立手动程序仍只在控制器释放后使用。

正常启动后，`journalctl -u apex-robot` 的 `ARM0/485A passed read-only preflight` 和 `ARM1/485A passed read-only preflight` 表示两侧预检通过。在加载 `/etc/apex/apex_ros_env.sh` 的环境中，`/info/wrist485_L`、`/info/wrist485_R` 提供真实位置、速度、状态字、故障 A、报警 B。

**故障后按侧恢复。** 触发控制错误时暂停该侧输入。如果该侧可能正在运动，立即请求一次停止。从检测到错误起，在第 3、6、9、12、15 秒开始只读复查。每轮确认驱动模式、实际位置、速度、静止位和故障/报警；动作结果不明确时需在同一轮取得两条健康、静止且位置一致的反馈。启动预检失败的侧会重新验证完整配置。检查成功即允许该侧接受新手势，先松开扳机再按。等待期间的手势会被丢弃。左右腕独立计时，同时复用 Apex 的控制器会话。

日志中的 `check N/5 failed` 表示该轮失败，读取超时会延后该轮结果。第五轮仍失败时，只锁定出错侧，直至人工排查后重新启动 Tool。重新启动 Tool 本身会对两侧重新预检。普通位置遥测暂缺而驱动状态有效时仍按目视模式处理。

2026-11-20 现场记录：18:31 和 18:36 的两次锁定分别记录了无效 RTU 回包和空回包。PICO 休眠期间也曾断开；重连后的采样窗口内，左右 `control/gripperValueL/R` 都为 `0.0`。排查时可同步观察该侧 `control/enableL/R`、扳机值和 `info/wrist485_L/R` 的位置。只看 Tool 服务是否运行无法判断夹爪适配器是否锁定。

11 月 20 日版在双爪均无进行中动作时，对空回包或损坏的只读帧最多尝试三次。机器人端当日测试日志保存在 `/home/nvidia/apex-wrist-readfix.l86lbibf/`。

11 月 21 日早期版本在运动中反馈丢失时，会先向正在执行动作的夹爪发出停止，再分别读取速度和静止位；未能确认静止或发现驱动故障曾使整个适配器锁定。现行按侧恢复策略见上文。速度、扭矩默认各 60%。

现场只读抓包共记录左腕 75 次请求、70 个回包，右腕 72 次请求、68 个回包。138 个回包 CRC 和长度均正确。两侧各取得 30 次有效反馈，抓包期间无写请求。协议、手动控制和 ROS 手势模拟回归全部通过。验证环境、`tests.log`、`actual.log` 和 `readonly.pcapng` 位于 Luna 的 `/home/nvidia/apex-ctag-verify.20YJNbWr/`。上版专用 SDK 保存在 `/usr/local/lib/luna-apex-wrist/sdk/libGentoSDK.before-stop-confirm.so`。

11 月 21 日 09:08 至 09:10 的 PICO 实操由同目录 `pico-retry.jsonl` 记录。左侧确认两次、右侧三次从张开到闭合再张开（位置 0 → 9000 → 0）。右爪第三次张开期间丢了一次位置回包，09:09:25 确认在位置 0 停止。有效反馈中故障和报警均为 0。09:09:35 右侧参数写入（功能码 `0x10`、寄存器 `0x0102`）未收到回执，适配器锁定。之后仍有扳机输入，已无新的位置回读。

09:15:14 重启的 Robot 已加载参数写入回执修复。09:16 的复测中，右侧确认两轮完整开合，左侧确认一轮并再次闭合。09:16:31 左腕读取 `0x0401` 时收到零字节，09:16:33 停止确认超时，适配器锁定。09:16:34 才读到左爪位置 9000、速度 0、静止位有效、故障和报警均为 0；右爪最后一次有效位置为 0。随后两侧仍有扳机输入，但适配器未再发布位置。该轮未记录到 `0x0102` 参数写入错误。

09:28:32 重启的 Robot 已加载迟到停止反馈修复。以同一次 TCP 握手对齐 PICO 和 Luna 的时钟，约 09:30:30 两只手柄同时丢失追踪，头显将两侧侧握置零，TCP 心跳仍有应答。09:30:47 Luna 重新记录到双手接管。09:30:56 右腕 `0x06/0x0108` 收到零字节后锁定，两侧随后均确认停在位置 0。此次日志没有命令值，无法判定缺少的是启动还是停止回执。旧的 ROS 记录在 09:28:39 已结束，本轮事件由系统和头显日志确认。09:34:08 单独重启 Tool，09:34:13 两侧预检通过，实际反馈均为 `[0,0,21,0,0]`。09:34 稍后头显应用挂起，TCP 会话随后断开。后续限时采集位于机器人 `/home/nvidia/apex-stop-readback.wGI6hOSY/` 的 `pico-after-command-ack.jsonl` 和 `wrist-0934.pcapng`。

### 构建与现场验证

11 月 21 日 09:58:42 的停机发生在左腕写入闭合目标 `9000` 的准备阶段。`wrist-0938.pcapng` 中，09:58:41.934 发出 `0x10/0x0102` 参数写入，收到的两段为 `01 10 01` 和 `06 e0 37`；完整回执应为 `01 10 01 02 00 06 e0 37`。Luna 未发出本次动作的 `0x06/0x0108` 启动指令。右爪此前已确认停在位置 0。PICO 当时仍在线，锁定之后仍有新扳机输入。该抓包无法区分是夹爪端、腕部串口还是控制器转发导致回包缺字节。

10:06:05 左腕型号寄存器 `0x0b00` 返回了损坏的 35 字节，预期为 37 字节。启动预检在初始化右腕前退出。10:10 单独重启 Tool，双腕均通过预检。随后右爪完成开合，10:14:08 左腕动作寄存器 `0x0108` 无回执，锁定后确认左爪静止在 9000。此轮未采集该指令的线上命令值。ROS 记录保存在 Luna `/home/nvidia/apex-short-ack.NA3K2vsQ/startup-retry.jsonl`。

10:41:29 左手松开扳机后，左腕写目标参数 `0x10/0x0102` 的回执只有 3 字节，期望 8 字节，适配器在启动张开动作前锁定。随后仍有扳机请求，头显在 10:42:50 断开。此轮日志只记录长度，未取得该回执的原始字节。10:44 单独重启 Tool 后恢复两侧反馈，左爪位置 9000、右爪位置 0，两侧速度、故障和报警均为 0。触发记录在 Luna `/home/nvidia/apex-trigger-edges.o6soEksD/pico-trigger.jsonl`。

使用厂家 `Gento-Teleoperation-Apex/Gento-SDK` 提交 `5a5f24332aba242a99b6c39d3f4ffde312d705ee` 的 `SDK_CT040404_SDK040402_260617/SDK/00040402/C_SDK`。`apex/build_sdk.sh` 先比较未修改基线和现场 SDK 的 `.text`、`.rodata`、`.data` 和导出符号尺寸，再仅重编修改的 L1 翻译单元并加入腕部模块。源码、编译器和此次构建结果保存在 Orin `/home/nvidia/apex-wrist-sdk.wv3zUx2i`。安装目录为 `/usr/local/lib/luna-apex-wrist`。启动器核对 `apex/factory.sha256`；后续厂家升级导致哈希改变时，将使用原厂 Robot 启动入口。

2026-11-20：真实 Apex Tool 服务启停已验证。隔离 ROS 域 93 的只读验证宿主通过已安装的专用库持有唯一控制器会话，持续执行与原厂 CAN 线程相同的两侧轮询。每侧均取得三条真实夹爪反馈，位置、速度、故障和报警均为零。原厂 Robot 已验证可以解析全部动态依赖。没有启动真实机械臂节点或遥控机械臂。手势开合、停用、断连、重连及空闲读回失败的测试在模拟环境完成。现场记录在 `../outputs/20261120-apex-ctag-installed-check.txt`。

### 回退

在现场正常退出遥操并释放控制器、三个模块均已停止后，管理员可将下列两个专用附加配置改名为 `.disabled`，然后执行 `sudo systemctl daemon-reload`。下次启动即恢复厂家的 Robot 和 Tool 入口。原厂 `/etc/apex/apex.env` 仍保留原来的 `dm` 设置。

```sh
sudo mv /etc/systemd/system/apex-robot.service.d/70-luna-ctag.conf /etc/systemd/system/apex-robot.service.d/70-luna-ctag.disabled
sudo mv /etc/systemd/system/apex-tool.service.d/70-luna-ctag.conf /etc/systemd/system/apex-tool.service.d/70-luna-ctag.disabled
sudo systemctl daemon-reload
```

## 手动开合

在 VR 工作区的 PowerShell 中运行 `python .\control_grippers.py` 进入双爪菜单。命令行也支持单次动作：

```powershell
python .\control_grippers.py status
python .\control_grippers.py open
python .\control_grippers.py close --side right --speed 60 --torque 60
```

`--side` 可取 `left`、`right`、`both`，默认双爪。张开目标为 0，闭合目标为 9000。速度和扭矩都使用 60～100 的整数百分比，默认各 60%。程序使用维护会话，需要 Apex 释放控制器。启动动作前，所选夹爪的目标、速度、扭矩和加减速都通过寄存器读回校验。双爪按左右顺序发送触发命令。

本机型按现场确认的规则处理报警 B。单独的 `0x1000` 掉落报警允许继续动作，状态输出保留原始位。其他报警 B 位以及任意故障 A 会中止操作。已经触发动作后遇到异常时，程序请求停止并检查零速反馈。通信中断可能导致停止无法确认。到位条件为反馈位置与端点相差不超过 20、位置到达位有效且速度为零，连续满足三次。闭合途中被物体阻挡时可能触发到位超时。

## 直接修改 Robot 源码的早期方案

`owner.patch` 在持有控制器会话的 `marvin_robot_node` 中构造 `wrist::Adapter`。`wrist_485_mode` 默认 `off`，保留原有 CAN 桥。`inspect` 和 `vr` 互斥使用腕部 RS‑485A，跳过两路 CAN 收发线程。模块使用现场 `libGentoSDK.so` 的 `FX_L1_Terminal_ClearData/GetData/SetData`，由现有进程提供已建立的 SDK 连接。

`inspect` 读取两只 CTAG2F90D 的型号、通信模式、使能、行程、位置、速度和报警。两边预检都通过后，发布 `/info/wrist485_L` 和 `/info/wrist485_R`。消息类型为 `Int32MultiArray`，顺序为原始位置、速度、状态字、故障 A、报警 B。保留原始 `0x1000`，沿用现场交付包对这一位的例外。

`vr` 模式使用对应侧的 `control/gripperValueL/R` 和侧握使能 `control/enableL/R`。在头显连接、当前侧可动且侧握持续有效时，先松开食指扳机，再按到 80% 闭合，松到 20% 张开。左映射 `ARM0/485A`，右映射 `ARM1/485A`。目标为 0 或 9000，候选适配器共用手动程序的默认速度 60%、扭矩 60%。启动、重新握住使能或重连后首次松开只用于识别起始手势。工作线程串行执行带 CRC 和读回校验的 RTU 事务，动作异常时尝试对已触发的一侧发停止并读回静止。断连检查依赖软件采样和串行通信，实际停止时延尚未测量。

## 2026-11-20 验证

在 Orin 隔离目录 `/home/nvidia/luna-wrist-verify.sRfivF` 编译了适配器，`libwrist_link_check.so` 已解析现场 ROS 和 `/usr/local/lib/libGentoSDK.so` 的符号。`ctest` 协议测试通过，覆盖模拟双路、启动防误触、触发超时、CRC、目标读回及启动与停止报文。此 `.so` 是链接检查产物。

运行中的 `kernelmind-apex 1.0.6.81g` 节点 SHA-256 为 `e198ceecc0846ee0eaeddb6bb5e1a34934a23e5c363f66f20c14c957d5071a07`。节点包含 `pd_cmd_cycle_time_ms`、`publish_cmd_pos_debug`、`check_cmd_timestamp`。公开 `gento_ros2_packages` 基线 `e6696cae3d95d84d1c14d0028ee267620bad9053` 的节点源码缺少这些参数。补丁已应用到公开源码的隔离副本，并成功编译了节点翻译单元；完整目标在原 `marvin_robot.cpp` 编译失败。该文件使用 `FX_L1_Comm_Send()` 和旧版 `FX_L1_Runtime_SetSpeedRatio` 签名，现场 SDK 要求后者提供 `thread_id`。

## 早期方案的源码依赖

请厂家提供 Orin、ROS 2 Humble、`kernelmind-apex 1.0.6.81g` 的 `Apex_Deploy/robot_node/marvin_ros2_control` 源码和相配的构建配置。原二进制中记录的构建根为 `/home/marvin/apex_gento_devel/src/`。将两个头文件和 `owner.patch` 中的连接及退出逻辑合入匹配分支，保留其控制周期、指令时效、原有 SDK 线程与驱动配置。切换末端前将 DM Tool 与 485 适配置为互斥。现场验收先使用 `inspect`，记录两侧型号和位置；随后在现场可观察且空载条件下逐侧验证扳机、松握、掉线、重新连接和停止反馈。

单独复验适配模块时，在 Orin 加载 `/etc/apex/apex_ros_env.sh` 后，在本目录执行：

```sh
cmake -S verify -B build
cmake --build build --target wrist_manual wrist_link_check protocol_test manual_test ros_adapter_test -j2
ctest --test-dir build --output-on-failure
```

2026-11-20 补充验证使用现场 Orin 和同一份 `wrist_adapter.hpp`。`ros_adapter_test` 在仅本机可见的 ROS 域 93 和独立命名空间中发送合成输入，使用测试可执行文件内的三个模拟末端 SDK 函数。现场 Apex 使用域 24。测试端不链接硬件 SDK，生产模块另行链接 `/usr/local/lib/libGentoSDK.so`。

集成测试覆盖左右 `485A` 开合与反馈、启动时按住扳机、运动中断连、重新连接后保持按住、触发报文发出前取消、取消后下一次有效动作。测试暴露了在清缓存等待期间松开侧握会使整个适配器进入故障锁定的问题。修复后协议测试通过，ROS 集成测试连续通过三次。模拟停止应答不能代表实体驱动停止时间。

同期只读观察现场 14 秒，14 次 `/info/vr_connected` 均为 `false`，左右扳机和侧握话题各收到 0 条消息。控制器仍由 `marvin_robot_node` 持有。本轮未验证真实手势、真实 RS-485 回包或机械动作。实体通信可在控制器已交接的维护窗口用已配置的 8 针交付程序 `run_luna_dual_gripper_three_cycles.py` 默认只读预检；该原版预检还会检查初始张开位置。

## 2026-11-20 实体只读检查

操作者释放控制器后，确认 Apex Robot、Teleop、Tool 均为 inactive，UDP 3720 至 3724 空闲。左 `ARM0/485A` 和右 `ARM1/485A` 均从真实设备读回 CTAG2F90D、行程 0 至 9000、当前位置 0、速度 0、状态字 0x0015、故障 0、报警 0。Python 交付代码的检查没有要求起始位置为零，发送端只允许功能码 0x03。右腕读取 0x0418 时重试三次，随后成功。

复用候选适配器的 `SharedTerminal` 和 `Motor::inspect` 进行独立 C++ 硬件测试。原始节奏和额外 150 ms 读取间隔的对照均出现空回包。新增的只读预检对空回复最多尝试五次，非空错误帧仍交给正常校验。完成预检后，VR 动作模式恢复原有单次事务行为；准备目标、触发和停止写入均没有重试。最终第一轮两侧通过；第二轮左侧 `0x0401` 在五次读取后仍为零字节，右侧重试一次后通过。原因尚未定位。所有现场测试只读取寄存器，完成后已断开 SDK 会话。

记录位于 `../outputs/20261120-171149-wrist485-readonly.json`、`../outputs/20261120-cpp-wrist485-paced-comparison.txt` 和 `../outputs/20261120-cpp-wrist485-final.txt`。在已交接的维护窗口，加载机器人 ROS 环境、构建 `inspect_wrist` 后，使用 `bash verify/read_released.sh` 重做独立 C++ 检查。该入口核对服务与端口并持有两把交接锁。
