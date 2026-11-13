# Luna Orin 双夹爪 USB-RS485 检查

检查日期：2026-11-12（北京时间）。仅检查系统设备及日志，未打开串口、
未发送夹爪协议帧、未修改机器人模式或启动机械臂运动。

## 主机与网络

- SSH：192.0.2.10，主机名 vision。
- 硬件：NVIDIA Jetson AGX Orin Developer Kit。
- 内核：5.15.148-rt-tegra。
- 本机 WLAN 起初断开，已使用已有 Luna_5G 配置重新连接。

## 两只 USB 串口转换器

| USB 路径 | 芯片 | VID:PID | 标识 | 本次启动曾分配的串口 |
| --- | --- | --- | --- | --- |
| 5-1.1 | FTDI FT232RL | 0403:6001 | BG011JUB | ttyUSB4 |
| 5-1.4 | QinHeng CH340 | 1a86:7523 | 未报告独立序列号 | ttyUSB5 |

两者位于同一 Genesys Logic USB Hub 的端口 1 和端口 4。
仅凭 USB 枚举无法确认哪只接左夹爪、哪只接右夹爪，也不能证明 RS485
侧供电、A/B 接线、波特率、从站地址或夹爪响应正常。
ttyUSB 编号是历史观察值，不能作为当前可用端口或持久设备标识。

## 已定位问题

ftdi_sio、ch341、usbserial 模块均已加载，nvidia 用户在 dialout 组。
本次启动内核日志：

```text
16:15:03 FTDI USB Serial Device converter now attached to ttyUSB4
16:15:03 ch341-uart converter now attached to ttyUSB5
16:15:03 usb 5-1.4: usbfs: interface 0 claimed by ch341 while 'brltty' sets config #1
16:15:03 ch341-uart converter now disconnected from ttyUSB5
16:15:04 usb 5-1.1: usbfs: interface 0 claimed by ftdi_sio while 'brltty' sets config #1
16:15:04 FTDI USB Serial Device converter now disconnected from ttyUSB4
```

brltty.service 虽然 disabled/inactive，但 brltty-udev.service 为 active，
由 udev 规则自动拉起。`/usr/lib/udev/rules.d/85-brltty.rules:293`
将 `1a86/7523/*` 匹配为 Baum 盲文设备。CH340 的 udev 属性明确包含：

```text
BRLTTY_BRAILLE_DRIVER=bm
SYSTEMD_WANTS=brltty-udev.service
```

因此本次检查结束时，两只夹爪转换器没有可用的 ttyUSB 串口节点。
期间出现的 ttyUSB0～3 属于 MEIG LTE-A_Module，并且日志显示其反复枚举，
不能拿这些端口尝试夹爪通信。

## 后续接入

需要先排除这两只转换器的 brltty 抢占，再重新绑定串口驱动并验证节点。
本次未停服务、未改 udev 规则、未重绑定 USB。
后续固定设备名应使用 FTDI 的序列号，以及 CH340 的物理 USB 路径；
左右夹爪映射需现场确认后命名。
拿到夹爪型号、协议、波特率和地址后，才可选择明确的只读状态指令验证 RS485。

## 原始证据

本工作区 evidence 目录：

- gripper-usb-inventory-20261112.txt
- gripper-usb-driver-20261112.txt
- gripper-usb-identities-20261112.txt

## 用户提供的 SDK 与协议核对

来源目录：
`C:/Users/operator/Downloads/伺服步进使用资料-外发/伺服步进使用资料-外发/`

已读取 Python SDK 全文、C++ 压缩包的头文件及 README，并提取同目录协议
和操作手册相关页的文字。仅把资料作为接口说明，没有执行其中安装或运动示例。

- Python SDK：`sdk/changingtek_p_rtu_Servo.py`，说明标注
  CTAG2F90D（Servo）控制器，依赖 minimalmodbus 和 pyserial。
- 默认通信：Modbus RTU，115200 baud，8 数据位，无校验，1 停止位，从站地址 1。
  这是资料默认值，尚未通过实机响应确认设备是否改过参数。
- C++ SDK 包含 Linux/ARM 串口实现声明。
- `串口驱动CH341.zip` 内只有目录和 `CH341SER.EXE`，属于 Windows 驱动安装包；
  不用于 Orin 当前 Linux 内核。当前系统 ch341 与 ftdi_sio 驱动已经存在。

只读寄存器（Modbus 功能码 0x03）：

| 地址 | 含义 | 依据 |
| --- | --- | --- |
| 0x0401 | 电机运行状态位 | 协议 PDF 第 2 页 |
| 0x0402 / 0x0403 | 故障与报警 | 协议 PDF 第 2 页 |
| 0x0411 | 找零完成状态 | 协议 PDF 第 2 页 |
| 0x0418 / 0x0419 | 用户位置，高字在前，SDK 按有符号 32 位合并 | SDK、协议 PDF 第 2 页 |
| 0x041A | 用户速度反馈 | SDK、操作手册 PDF 第 24 页 |
| 0x041B | 用户电流反馈 | SDK、操作手册 PDF 第 24 页 |
| 0x0A00 / 0x0A01 / 0x0A02 | 地址、波特率编码、数据格式编码 | 协议 PDF 第 3 页 |

操作手册 PDF 第 23 页同时将用户位置描述为“行程千分比”，又举例
9000 对应 90mm；SDK 参数名 position_mm 也没有做单位转换。
因此实际接入先显示原始位置计数，不将示例数值或单位描述直接用于运动。
速度、电流手册示例均为百分比，不可将电流反馈当作安培。

Python 文件的主程序会在连接后执行五次 9000 与 0 的往复运动，并再次写入
0 触发运动。它不是只读通信测试。C++ README 也明确 main 为五次循环演示。
本次没有运行这两个示例，也未写入 0x0102～0x0108 等运动寄存器。

复查 Orin：两只转换器仍在 USB 枚举中，brltty-udev.service 仍为 active，
没有可用的夹爪串口节点；pyserial 3.5 已安装，minimalmodbus 未安装。
证据：`evidence/gripper-sdk-readiness-20261112.txt`。
本次只完成资料及环境核查，未安装软件或调整系统服务。

## 已执行抢占修复与只读验证（后续用户明确授权）

用户随后要求排除抢占、只读验证两组响应，最后验证 SDK 五组动作。
本节取代前面“未改服务、未安装依赖”的历史状态。

已完成：

- 在 Orin `/etc/udev/rules.d/85-brltty.rules` 创建厂商规则的本地覆盖，
  仅跳过序列号 BG011JUB 的 FTDI 及 USB 位置 5-1.4 的 CH340；
  清除这两只设备的 BRLTTY 自动启动属性，保留其他规则。
- 停止 brltty-udev.service，重新绑定 ftdi_sio/ch341 驱动。
  未卸载系统驱动、未重启机器人、未改变机械臂使能。
- 添加 `/etc/udev/rules.d/99-luna-grippers.rules`：
  `/dev/luna-gripper-ftdi` 当前指向 ttyUSB4；
  `/dev/luna-gripper-ch340` 当前指向 ttyUSB5。
  tty 节点使用 dialout 组、0660 权限。
- 备份：`/var/backups/luna-gripper-usb-20261112-162545`。
- 在 `/home/nvidia/luna-gripper-check/deps` 隔离安装 minimalmodbus 2.1.1
  与 pyserial 3.5，不修改系统 Python。因 Orin DNS 失败，通过本机下载 wheel、
  SFTP 传输后离线安装。
- 原厂 Python SDK 放在同目录 `vendor_gripper_sdk.py`，只导入类，不执行其主程序。

只读实测未通过：

| 转换器 | 实测结果 |
| --- | --- |
| FTDI | 串口可以打开，但默认 115200 / 8N1 / ID 1 的所有读取均无响应 |
| CH340 | 打开串口返回 EIO；内核报告 USB control message -110、modem status -110 |

FTDI 追加有限只读探测：115200 下地址 1～16，以及 9600、57600、38400、
19200、230400 下地址 1/2，读取 0x0401，均无有效回复。其间内核也记录
FTDI 设置波特率、数据格式及流控的 USB 传输失败，因此不能据此排除或认定
RS485 参数不符。

对 CH340 做了一次仅针对该转换器的 USBDEVFS_RESET（未重置父 Hub），
调用最终返回 ENODEV。内核随后自动重新枚举为 Bus005 Device005，
固定别名恢复且 brltty 保持 inactive，但再次打开串口仍报 EIO/-110。
没有继续循环复位。USB 设备电源状态均为 active，两个转换器的 power/control
均为 on。此时软件抢占已排除，但 USB 链路尚未恢复正常。

**五次往复动作未执行。** 尚无两组夹爪的有效反馈，无法核验使能、报警和
0/9000 的实际位置范围。本轮只发送功能码 0x03；没有发送 Modbus 写寄存器、
使能、复位或运动触发指令。只读脚本将两个 SDK 写接口直接禁止，
`--execute` 在连接前明确阻断，不能把失败的通信当作动作通过。

现场下一步：检查/重插 USB 转换器与 Hub，优先临时直连 Orin 排除 Hub/线缆问题；
确认夹爪独立 24V 供电及 A/B 对应。CH340 若更换物理 USB 位置，需要同步更新
其无序列号设备的固定路径规则。通信通过后，再继续用户已授权的 SDK 五次往复验证。

可重复只读检查：

```powershell
python tools/run_gripper_remote.py inspect
```

本机脚本：`tools/repair_gripper_usb.py`、`tools/check_gripper_sdk.py`、
`tools/run_gripper_remote.py`。四项离线检查覆盖禁止写寄存器、反馈读地址、
有符号位置解码和通信未验证时的动作阻断。

证据（文件名采用本机北京时间，JSON 事件采用 Orin UTC；两机时钟仍有偏差）：

- `evidence/gripper-repair-20261112-161220.jsonl`
- `evidence/gripper-install-20261112-161419.jsonl`
- `evidence/gripper-inspect-20261112-161432.jsonl`
- `evidence/gripper-reset-ch340-20261112-161527.jsonl`
- `evidence/gripper-scan-20261112-161641.jsonl`
- `evidence/gripper-inspect-20261112-161732.jsonl`
- `evidence/gripper-final-link-20261112.txt`
- `evidence/gripper-usb-power-20261112.txt`

## 用户重启后：两组只读通信恢复

用户报告重启后再次复查，Orin 可登录，两只固定串口均存在，
brltty-udev.service 仍为 inactive，说明排除抢占的配置跨重启保留。

最终记录 `evidence/gripper-inspect-20261112-163105.jsonl`：

| 项目 | FTDI 通道 | CH340 通道 |
| --- | --- | --- |
| 型号字符串 | CTAG2F90D | CTAG2F90D |
| 通信 | 115200 / 8N1 / ID 1 | 115200 / 8N1 / ID 1 |
| 使能寄存器 0x0100 | 1 | 1 |
| 位置模式 0x0101 | 0 | 0 |
| 用户单位选择 0x0119 | 1 | 1 |
| 用户位置映射范围 | 0～9000 | 0～9000 |
| 电机位置范围 | 0～20687 | 0～20184 |
| 当前用户位置 | 0 | 0 |
| 当前速度/电流原始值 | 0 / 0 | 0 / 0 |
| 故障/报警 | 0 / 0 | 0 / 0 |
| 状态 0x0401 | 0x15 | 0x15 |
| 找零完成 0x0411 | 0 | 0 |
| 欠压/过压阈值原始值 | 1800 / 2800 | 1800 / 2800 |

资料版本差异已核对：软件操作手册 V3.1 第 13～14 页规定
0x0500～0x0503 为电机位置范围，0x0504～0x0507 为用户映射范围。
第 11 页的 0x0301 实为 IO 消抖参数，因此已修正检查工具的读取地址和标签，
先前 0x0301 开始的一组值不能解释为位置限位。
V3.1 第 1 页以 0x0100 作为使能显示来源，第 2 页不再列出 0x0401 的
0x8000 使能位；故旧协议的该状态位为 0 不应直接判定未使能。

Orin 本机 INA3221 读到 VDD_GPU_SOC/VDD_CPU_CV 通道 in_input=19816，
VIN_SYS_5V0 通道 in_input=4984，按 hwmon 毫伏约定对应约 19.816V 与 4.984V。
这是板载通道读数，不是夹爪外部 24V 端子的实测电压。没有证明用户先前看到的
14V 从何而来，也不能由通信恢复推出带载供电已稳定。
原始主机检查：`evidence/gripper-after-reboot-20261112.txt`。

本轮每组反馈连续三次成功，均未写入寄存器或触发动作。SDK 五次 9000/0 往复
的端点已落在设备读回的用户范围内；但先前 14V 供电问题仍需确认测量位置和
当前夹爪端电压，不能拿 Orin 电压代替。动作验证尚未执行。

## 2026-11-13 恢复测试：USB 总线改号后再次抢占

用户在前一轮已确认供电恢复 24V，并授权 SDK 五次往复；随后关机导致
测试未执行。本日用户要求重新进行，已重新连接 Orin。

重启后两只转换器从 `5-1.1 / 5-1.4` 变为 `3-1.1 / 3-1.4`。
此前 CH340 排除规则使用内核总线编号，未匹配新设备，BRLTTY 再次启动
并抢占串口。因此上一节“跨重启保留”仅适用于当时编号未变的那次启动。

已修正 `tools/repair_gripper_usb.py`：FTDI 按序列号 BG011JUB 识别；
CH340 按稳定物理路径
`platform-14180000.pcie-pci-0000:01:00.0-usb-0:1.4` 识别。
修复脚本先发现并核验设备，不再假定 USB 总线编号。规则已部署，
原规则备份于 Orin `/var/backups/luna-gripper-usb-20261113-090634`。
BRLTTY 已停止，两只别名分别恢复到 ttyUSB4、ttyUSB5，未发现串口占用。

本次只读检查仍失败：

- FTDI：串口可打开，但寄存器请求无回复。
- CH340：打开串口报 EIO；内核同时记录 USB 控制传输超时 `-110`。
- 本次未得到有效夹爪状态，未发送运动或寄存器写入命令，五次动作仍未执行。

下一步需要现场将两只 USB 转接器拔插回原插口，使其按新规则重新枚举，
然后重新只读验证。不要直接启动动作脚本或清除报警来跳过通信检查。
已准备的执行模式会核验两组型号、使能、范围、起点和报警，执行五次
9000/0 往复（速度 50%、力 25%、加减速 60），逐端点验证到位；
异常时对两组发送停止。此处速度是夹爪 SDK 示例参数，与双臂 60% 无关。

证据：

- `evidence/gripper-repair-20261113-085302.jsonl`
- `evidence/gripper-inspect-20261113-085311.jsonl`

本机证据文件名与 Orin 日志时钟约相差 13 分钟，不应将时间差当作第二次运行。

### 同日重新插拔后的复查

用户确认重新插拔后，内核记录两只转换器断开并重新枚举，固定别名正确，
BRLTTY 保持 inactive，fuser 未发现占用。两串口均可打开，CH340 不再报 EIO，
重新枚举后的日志未见该转换器控制传输超时。

但两组在原通信参数 115200/8N1/ID 1 下读取配置和反馈均无回复。
补充只读探测：115200 下 ID 1～16；9600、57600、38400、19200、
230400 下 ID 1/2，均无响应。这是有限参数探测，不能据此认定设备损坏。
需要现场核实夹爪本体供电指示、夹爪端电压与 485 接线；USB 转接器被系统
识别不能证明夹爪已供电。本次没有写寄存器或触发运动，五次测试仍待通信恢复。

证据：`evidence/gripper-inspect-20261113-085621.jsonl`、
`evidence/gripper-scan-20261113-085710.jsonl`。

### 用户再次确认 24V 后：USB 底层超时复发

用户明确确认夹爪供电为 24V。继续对比前日成功参数，两组仍应为
115200/8N1/ID 1。新增 `diagnose` 模式仅发送功能码 03，记录 SDK 原始
请求/回复，超时延长为 1 秒，并尝试本地主机 UART 的 N1/E1/O1/N2 格式。
没有更改夹爪寄存器或触发运动。

`evidence/gripper-diagnose-20261113-090254.jsonl` 显示 FTDI 请求
`01 03 04 01 00 03 55 3B`（状态）和
`01 03 0A 00 00 03 06 13`（通信参数）均收到 0 字节。
CH340 再次在打开串口时 EIO。
随后内核日志证实 FTDI 在诊断初次打开阶段即发生串口格式、波特率设置
失败及 flow control 超时 -110，CH340 同样出现 USB 控制传输超时 -110。
因此此次无响应不能单独归因为 485 波特率或地址错误；软件返回发送完成
也不能证明请求已经到达夹爪。

此时 BRLTTY inactive、串口与两只 USB 设备未发现占用；
两只转换器 power/control=on、runtime_status=active、累计休眠时间为 0；
共同父 Hub 3-1 同样为 active、累计休眠时间为 0。
下一步是将一只转换器绕过该 Hub，直接接 Orin 空闲 USB 口作只读对比；
FTDI 有序列号，适合作为首选。尚不能断定具体是 Hub、线缆还是转换器故障。

内核与电源管理原始输出保存在本机
`C:/Users/operator/Documents/Codex/2026-11-03/he/work/robot-gripper-usb-recurrence.txt`。
九项既有离线测试通过；现场五次动作未执行。

### 再次重启并换口后

用户报告已重启、换 USB 口。本次 Orin 开机约 2 分钟，
CH340 位于 `5-1.1`，FTDI BG011JUB 位于 `5-1.3`，两者仍在同一个
`5-1` 四口 Hub 下。软件拓扑无法判断该 Hub 是盒内还是外置。
CH340 物理路径改变导致旧规则不匹配，BRLTTY 再次启动并抢占。
已将 CH340 的规则和只读/执行身份检查更新为
`platform-14180000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0`（tty 路径），
USB 设备路径去掉末尾 `:1.0`。FTDI 仍按序列号识别。
规则备份 `/var/backups/luna-gripper-usb-20261113-092316`。

修复后 BRLTTY inactive，两串口可打开，但均无 Modbus 回复。
随后在确认串口无人占用后，尝试对已核验转换器进行 USB 重新授权，
第一只 FTDI 即在恢复授权时超时，内核记录
`usb 5-1.3: can't set config #1, error -110`。
复查 FTDI 的 authorized=1，但未配置成功，ttyUSB4/FTDI 别名消失；
CH340 的 authorized=1、配置=1，串口别名仍存在，但单独打开也报 EIO。
脚本已退出，没有继续操作第二只设备的 USB 授权，无遗留测试进程。
没有写夹爪寄存器或启动动作，不能标记 SDK 五次验证完成。
下一步需要确认实际 Hub/USB 接线位置，作独立 USB 链路对比。

证据：

- `evidence/gripper-repair-20261113-090944.jsonl`
- `evidence/gripper-inspect-20261113-091003.jsonl`
- `evidence/gripper-reconnect-usb-20261113-091132.jsonl`
- `evidence/gripper-inspect-20261113-091240.jsonl`

### 本体接口确认及指定 USB 控制器恢复

用户确认上述接口是计算盒本体接口。系统显示两只转换器通过 Genesys
05e3:0610 Hub 接入 Renesas uPD720202（1912:0015），PCI 地址
`0000:01:00.0`。该控制器启动日志有 `xHCI HW not ready after 5 sec`，
但仅凭此警告不能断定故障根因。另一个控制器 0004:01:00.0 连接相机、
触摸设备和音频；网络和存储同样不在目标控制器下。

在检查目标 PCI 身份、全部六个 USB 节点（两个根 Hub、两个 Genesys
Hub 接口、两只指定转换器）及 USB/串口无人占用后，执行一次
`rebind-controller`，仅对 0000:01:00.0 解除并重新绑定 xhci_hcd。
BRLTTY 保持 inactive。操作成功，两串口恢复：

- `/dev/luna-gripper-ftdi` -> `/dev/ttyUSB5`
- `/dev/luna-gripper-ch340` -> `/dev/ttyUSB4`

随后两串口可立即打开，但在已知 115200/8N1/ID 1 下读取配置和反馈
仍全部无响应。重绑定后至本次检查结束未见新的 USB 控制传输超时，
最终无串口占用。不能将串口恢复视为夹爪通信或动作测试通过。

证据：`evidence/gripper-rebind-controller-20261113-091608.jsonl`、
`evidence/gripper-inspect-20261113-091620.jsonl`。内核原始记录在连接工具的
`robot-gripper-host-controller-20261113.txt`、
`robot-gripper-controller-warning.txt`、
`robot-gripper-final-controller-state.txt`。
启动早期内核日志显示 June 4，随后时间同步为 November 13，均属于本次
`journalctl -b` 启动记录，不代表该警告发生于另一次旧启动。

Windows 端已检测到 pyserial 3.5，当前无 Win32_SerialPort 设备。
下一步建议将一只转换器 USB 接入当前 Windows 电脑，保留夹爪供电与
485 接线，作独立主机只读对比。尚未发送任何夹爪运动指令。

### Windows 直连 FTDI 对比结果

用户将转换器直接接到当前 Windows 电脑。pyserial 枚举为 COM4，
VID/PID=0403:6001、序列号显示 BG011JUBA；对应此前 FTDI 通道。
新增 `tools/check_gripper_windows.py`，通过本地 minimalmodbus 2.1.1
wheel 加载依赖和用户提供的厂家 SDK，不执行 SDK 的运动示例。
子类禁止两个寄存器写接口，只调用功能码 03。

COM4 正常打开。115200/8N1/ID 1 下连续三次读取状态，
每次等待 1 秒，均收到 0 字节。随后有限探测 32 组参数，
全部为 NoResponseError，没有有效回复：

- 115200/8N1，地址 1～16。
- 115200/8E1、8O1、8N2，地址 1/2。
- 9600、19200、38400、57600、230400 的 8N1，地址 1/2。

未见串口打开或配置错误；检测结束 COM4 已关闭。
这证明 FTDI 通道的无回复现象可在另一主机复现，不能只归因于 Orin。
仍不能凭此断言夹爪或转接器损坏，也没有穷举所有通信参数。
下一步建议将另一只 CH340 接入 Windows 做相同只读对比，随后根据结果
核查转换器 485 侧、A/B 接线和夹爪通信端。供电采用用户明确确认的 24V。
本轮未写设备寄存器或触发运动，SDK 五次测试仍未执行。

证据：

- `evidence/gripper-windows-20261113-092324.jsonl`
- `evidence/gripper-windows-20261113-092343.jsonl`

### 厂家上位机成功连接：修正排查结论

用户提供截图并实际操作 UServoController 成功。已通过界面查看确认：
连接模式 Modbus RTU、COM4、115200、8 位、无校验、1 停止位、ID 1、
设备数量 1。软件显示连接、使能及持续更新的温度和位置反馈。
这些基本参数与 Python 脚本一致。不能继续仅凭早先无回复推定接线、
转换器或夹爪硬件故障；Orin 先前 USB 超时日志仍是独立的真实观测。

实际运行软件来自另一份目录：
`C:/Users/operator/Downloads/伺服步进使用资料-外发 (1)/伺服步进使用资料-外发/UServoController_1.0.0.8Beta/UServoController_1.0.0.8Beta`。
其中 `log/20261113/log.txt` 有用户操作产生的 `01 17 ...` 读写组合动作帧。
它们会写运动目标和触发寄存器，不能为了“测试连接”直接重放。
该日志并未证明周期性只读查询使用何种功能码，也不能由动作帧
使用 0x17 推断此前 0x03 无回复的原因。

本次只切换到连接页查看，没有断开厂家软件或触发动作。查看时
报警 A=0x0000，报警 B=0x1000（手册定义为掉落报警），没有清除。
准确失败原因仍未证实，需待用户暂停厂家手动操作并释放 COM4 后，
在同一连接状态下对比 Python 串口初始化、DTR/RTS、时序和读取请求。
此前脚本的 DTR/RTS 为 true；厂家软件这两项实际值尚未取得，
不应将其作为已经证实的原因。

### 厂家软件断开后：原 Python 脚本直接读取成功

用户明确报告已断开厂家软件，随后运行未修改的
`python tools/check_gripper_windows.py`，退出码 0。
COM4 / FTDI BG011JUBA，115200/8N1/ID 1，首次功能码 03 请求即收到有效回复：
请求 `01 03 04 01 00 03 55 3B`，
回复 `01 03 06 00 15 00 00 00 00 2C B6`。

读回型号 CTAG2F90D，通信寄存器 [1,6,1]，使能/位置模式 [1,0]，
用户范围 0～9000。连续三次反馈：位置原始值 2506、速度 0、
电流原始值 20/20/21、状态 0x15、故障 0、报警 0。
与上次界面出现的 0x1000 相比，目前报警已为 0；本次脚本没有清除报警。

此次没有修改脚本、DTR/RTS 或通信参数，证明现有 SDK 的 03 读取方式
能在此链路正常工作。厂家软件使用后恢复通信是时间顺序事实，
并不能据此证明某个初始化动作是必要条件，也不能把此前失败归因于
厂家软件占用（此前只读脚本成功打开串口但收不到回复）。
具体恢复原因尚未查实，不再以早先无响应推断硬件损坏。

本轮只读，COM4 已关闭，五次动作未执行；当前只验证了 FTDI 一路，
当前位置也不是此前五次动作脚本要求的起点 0。
证据：`evidence/gripper-windows-20261113-094501.jsonl`。

### 随后读取 Orin 本体 USB：CH340 通信成功

按用户要求只读检查机器人本体 USB。电脑 Wi-Fi 曾断开，重连 Luna_5G
后可 SSH 登录 192.0.2.10；中途一次 SSH 超时重试后恢复。
FTDI 仍在 Windows COM4，本体只剩 CH340（5-1.1，ttyUSB4）。
BRLTTY inactive，未发现串口占用。本轮没有重置 USB 控制器或改通信配置。

执行 `python tools/run_gripper_remote.py inspect --port /dev/luna-gripper-ch340`
退出码 0，读到 CTAG2F90D、版本 2060000000000000、
通信 [1,6,1]、使能/位置模式 [1,0]、用户范围 0～9000。
本次电机范围读回 0～20210（此前为 20184；本轮没有修改范围，
差异来源未确认）。连续三次反馈均为位置 0、速度 0、电流原始值 0、
状态 0x15、故障 0、报警 0。

证据 `evidence/gripper-inspect-20261113-095035.jsonl`。
注意现有工具末尾字段 `both_responded=true` 在指定单端口模式下实际表示
所选端口检查通过，本轮不代表两组均连接到 Orin。无写寄存器命令、
无运动指令，连接已关闭。此前恢复的具体原因仍未查实。

### 继续 SDK 动作测试：CH340 首段触发掉落报警并停止

用户要求继续测试。两路预检通信正常：Windows FTDI COM4 的位置 2506、
Orin CH340 的位置 0，均已使能、静止、故障/报警为 0。
因为两只夹爪分属两台主机，安排分路各五次 9000/0 往复。
新增明确的 Orin `execute-single --port` 入口及 Windows
`tools/run_gripper_windows_cycles.py --execute`，共享厂家 SDK 动作与
到位验证逻辑。Windows 入口允许经过完整预检的范围内静止非零起点，
最终目标仍为 0。默认双夹爪入口保留两端口和零起点要求。
11 项离线测试及 Python 语法检查通过。

实际先启动 CH340，使用速度 50%、力 25%、加减速参数 60。
目标参数读回一致，第一段 9000 目标触发后，反馈位置 4467、
速度原始值 32、状态 0x18（包含力矩到达）、故障 0、报警 B=4096。
程序立即发送 0x0108=3，收到停止写入确认；停止后首次读取仍在减速，
位置 5121、速度 19。随后独立只读复查连续三次均为位置 5161、
速度 0、零速状态 0x4、故障 0、报警 B=0x1000、电流原始值 10。
确认停止，未自动清警、回原位、重试或提高力矩。

软件手册 V3.1 第 3 页将 0x0403 的 0x1000 定义为掉落报警，
但没有足够条件说明本次触发原因。不能直接推定是实际掉落或夹持力不足。
需要现场确认夹爪是否空载、运动途中是否接触物体，再决定下一步。
五次循环完成数 0，首个 9000 端点未通过；Windows FTDI 尚未执行动作。
证据：

- `evidence/gripper-windows-20261113-095657.jsonl`
- `evidence/gripper-inspect-20261113-095658.jsonl`
- `evidence/gripper-execute-single-20261113-095841.jsonl`
- `evidence/gripper-inspect-20261113-095923.jsonl`

网络偏好：用户明确要求 Luna Wi-Fi 仅承担机器人内网，上网走有线。
WLAN IPv4 metric 已从 1 改为 500，公网路由经以太网 5，6.6.7.0/24
仍经 WLAN；已验证直连公网 HTTP 200 和机器人 SSH。勿恢复 WLAN 高优先级。

### 空载确认后的低速诊断：小行程通过，端点附近报警复现

用户明确确认 CH340 夹爪空载，上一段运动没有夹到或碰到东西。
补充只读结果：0x0800=0（无规划）、电流到达阈值/窗口 0x0536/37=[10,10]、
位置到达阈值/窗口 0x0508..0A=[0,500,10]、速度/零速阈值窗口
0x0525..28=[50,10,50,10]。厂商 V3.1 手册第 5 页说明
临时区 0x0106/07 是加减速千分比，示例的 60 不能解释为 60%。
没有修改这些阈值、曲线模式、限位或持久化配置。

新增显式 `recover-empty --port` 诊断入口：完整身份和静止配置检查，
仅允许当前确切为 0x1000 的报警复位一次（0x0110=1），随后三次
无报警预检，速度降至 10%、力矩保持 25%、斜坡原始值保持 60。
先做 500 用户单位小行程，成功后才进入五次端点往复；
所有新报警仍立即停止，无自动清警循环。停止函数增加静止确认。
14 项离线测试通过。

实际执行：

1. 复位前位置 5161；复位后三次预检无报警，读取位置 5177。
2. 小行程目标 4677，到位反馈 4676，速度 0、报警 0，保持监测 3 秒通过。
3. 随后的第一个 9000 目标：先读到位置 8954、速度 2、报警 0；
   下一次位置 8987、状态 0x1d（位置到达/零速/力矩到达）、报警 0x1000。
4. 立即发送停止并确认静止。独立复查三次均为位置 8985、速度 0、
   电流原始值 0、状态 0x15、故障 A=0、报警 B=0x1000。

五次往复仍未完成，不能将小行程日志的 `sdk_test_complete` 事件解释为
五次测试通过（该事件标明 cycles_per_gripper=1、final_target_raw=4677）。
Windows FTDI 没有执行动作。没有自动回零、再次复位或屏蔽报警。
低速未消除掉落报警，原因未确定，应请厂家确认空载运动中该报警的
触发/锁存条件、力矩到达与掉落判定关系及推荐测试参数，不能断言硬件损坏。

证据：

- `evidence/gripper-inspect-20261113-100236.jsonl`
- `evidence/gripper-recover-empty-20261113-100426.jsonl`
- `evidence/gripper-inspect-20261113-100448.jsonl`

可交厂家核查的问题（尚未发送）：
CTAG2F90D，版本字符串 2060000000000000，24V，用户确认空载无外部碰撞。
03/06/10 通信及参数读回正常。速度 50%、力矩 25%、斜坡 60/60 时
0->9000 在位置 4467 出现 B=0x1000；复位一次后以速度 10%、
同力矩/斜坡执行约 500 单位小行程成功，但再去 9000 于位置 8987
同时出现状态 0x1d 和 B=0x1000。A 始终 0。请确认该版本空载时
掉落报警判定条件、是否需要特定控制模式，以及厂家推荐的空载测试参数。

### 厂家反馈后的专项兼容：两只夹爪各五次往复通过

2026-11-13，用户转述厂家答复：该报警可以忽略，属于固件错误；
用户在原厂软件中手动验证力矩 20、速度 100 也会报警。
这是用户提供的厂家结论，尚未取得厂家固件修订说明。

增加显式 `--ignore-drop-alarm` 参数，仅在本次 SDK 测试的判断中允许
报警 B 的 0x1000 位，原始反馈仍完整记录。故障 A 任意位、B 的其他位、
通信错误、参数读回不一致、位置越界和到位超时仍终止动作并尝试停止。
未修改驱动器报警配置、阈值或持久化参数，未执行清警。
16 项离线测试通过，包含专项掩码不修改原始反馈及其他故障仍被拒绝。

本次实际参数：速度 10%、力矩 25%、加减速原始值 60/60。
两只夹爪分主机依次执行，每只五次 `9000 -> 0`，每个目标至少观察
3 秒，并连续三次确认位置误差不超过 20 用户单位、到位和零速。
用户提到的速度 100、力矩 20 是其手动复现条件，本次没有采用。

| 设备 | 完成往复 | 目标验证 | 最终位置 | 最终速度 | 最终故障 A | 最终报警 B |
| --- | --- | --- | --- | --- | --- | --- |
| Orin CH340 `/dev/luna-gripper-ch340` | 5 | 10/10 | 0 | 0 | 0 | 0 |
| Windows FTDI `COM4`，BG011JUBA | 5 | 10/10 | 0 | 0 | 0 | 0x1000 |

CH340 的五个 9000 端点反馈均为 9000；FTDI 分别为
8981、8980、8982、8983、8983，均在既定 20 用户单位容差内。
两者运行过程中均观察到 0x1000，但未出现其他故障位。
两次进程均正常退出，串口关闭，没有后台持续动作。
这验证的是当前参数下的 SDK 五次往复，不代表高速或带载验证通过。

执行入口：

```powershell
python tools/run_gripper_remote.py execute-single --port /dev/luna-gripper-ch340 --ignore-drop-alarm --speed 10
python tools/run_gripper_windows_cycles.py --execute --ignore-drop-alarm --speed 10
```

证据：

- `evidence/gripper-execute-single-20261113-101155.jsonl`
- `evidence/gripper-windows-execute-20261113-101522.jsonl`

时间说明：证据文件名采用 Windows 本机时间；Orin JSON 事件采用其
系统 UTC 时间，当前约比本机快 13 分 33 秒，不能直接跨主机按事件时间排序。

### 两只接回 Orin 后联合驱动通过

2026-11-13，用户确认两只夹爪都接在机器人上，并要求同时驱动。
只读检查确认 FTDI BG011JUB 为 ttyUSB5，CH340 为 ttyUSB4，
既有稳定别名和身份校验均通过，串口没有占用。起始位置 FTDI=8613、
CH340 约为 0，两只静止，只有已获用户确认可忽略的 B=0x1000。

执行既有双路入口：

```powershell
python tools/run_gripper_remote.py execute --ignore-drop-alarm --speed 10
```

速度 10%、力矩 25%、加减速原始值 60/60，两路参数读回一致后，
连续下发两路启动命令，每段等两只都到位且静止后进入下一段。
五次 `9000 -> 0` 往复全部完成，十个联合目标验证通过，
进程正常退出码 0。日志启动确认事件间隔为 8.009 至 9.723 毫秒；
这是软件近同时启动，不能视为硬件同步精度或实际机械启动时差。

最终 FTDI：位置 0、速度 0、故障 A=0、报警 B=0x1000；
CH340：位置 0、速度 0、故障 A=0、报警 B=0。
原始报警仍保留，无其他故障或通信异常，机械臂未参与动作。

证据：

- `evidence/gripper-inspect-20261113-102430.jsonl`
- `evidence/gripper-execute-20261113-102443.jsonl`

### 用户确认开合方向，双路力矩 30% 闭合完成

2026-11-13，用户明确确认两只均为：0=张开，9000=闭合。
要求两只力矩改为 30 并夹上，沿用速度 10%，结束保持闭合。
新增 `close` 入口和 `--force` 参数，闭合只运行一个 9000 目标，
没有返回 0；所有原有报警、参数读回、到位和异常停止检查保留。
默认往复测试仍为力矩 25%，本次显式指定 30%。18 项离线测试通过。

先前及本轮只读检查中 CH340 打开串口报 EIO，单设备 USB 复位曾失败；
没有在这些失败检查后下发动作。随后实际闭合入口的两路身份、
串口打开和通信预检均通过，才开始设置参数、读回并触发。
暂不能认定间歇性 USB 问题已根治。

```powershell
python tools/run_gripper_remote.py close --ignore-drop-alarm --speed 10 --force 30
```

两路临时目标参数均读回 `[0,9000,10,30,60,60]` 后触发，
启动确认事件间隔 8.376 毫秒。连续到位及静止检查通过，正常退出码 0。
最终反馈：

| 设备 | 位置 | 速度 | 故障 A | 报警 B | 力矩到达位 |
| --- | --- | --- | --- | --- | --- |
| FTDI | 8999 | 0 | 0 | 0 | true |
| CH340 | 9000 | 0 | 0 | 0x1000 | true |

到位容差仍为 20 用户单位；30% 是 SDK 力/力矩参数，
不是实测牛顿或牛米。保持闭合，没有执行张开、去使能或回零。
到位和力矩状态不等同于已验证夹住具体物体。

证据：

- `evidence/gripper-inspect-20261113-103406.jsonl`
- `evidence/gripper-close-20261113-103513.jsonl`

### 手动张开/闭合程序

用户要求自行运行，新增项目根目录 `run_grippers.py`。在 Windows 上通过
既有 SSH 连接控制机器人两路 USB，保留项目内 tools、厂家 SDK 和当前
连接配置依赖；不是脱离项目的独立部署文件。
直接运行显示菜单：1=双爪张开到 0，2=双爪闭合到 9000，
3=只读状态，0=退出。启动显示菜单本身不连接或运动。
每次动作完成保持当前位置，失败退出而不自动重复。

文件顶部 `SPEED_PERCENT = 10`、`FORCE_PERCENT = 30` 可修改；
当前速度范围 1～50、力矩范围 1～30。也支持命令行：

```powershell
python run_grippers.py
python run_grippers.py open
python run_grippers.py close
python run_grippers.py status
python run_grippers.py close --speed 10 --force 30
```

双路预检、参数读回、目标到位、故障停止和 Ctrl+C 远端停止逻辑沿用
已验证执行器。仅 B=0x1000 允许记录后继续；不自动复位或去使能。
本次交付验证为 24 项离线检查及 CLI 帮助检查，没有触发实际运动。
