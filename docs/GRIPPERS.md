# 双夹爪：安装、张开与闭合

本页面向首次下载用户。Windows/Linux 主机通过 SSH，在机器人 Linux 上运行厂家
Python SDK，经两根独立 USB-RS485 转接器控制两只夹爪。不是 Windows COM 直连版。
无需部署机械臂 SDK、碰撞库或 ROS 即可单独使用本功能。

## 1. 已验证设备与数值

| 项目 | 本次现场 |
|---|---|
| 夹爪型号 / 驱动版本读数 | CTAG2F90D / `2060000000000000` |
| 转接器 | FTDI USB-RS485（UT-890A-I）与 CH340，各一只 |
| 串口 | 115200，8N1，每根独立总线的从站地址均为 1 |
| 单位 / 映射 | 用户单位，0～9000；不是 SDK 函数名所暗示的毫米 |
| 方向 | 用户确认 0 张开，9000 闭合 |
| 默认速度 / 力矩 | 10% / 30%，不是角速度或实测牛米 |
| 加减速 | 寄存器原始值 60/60（千分比） |
| 到位判断 | 目标误差 ≤20 用户单位、到位位、零速位、速度反馈0，连续3次 |
| 观察 / 超时 | 每目标至少3秒，最多12秒 |

两根线独立，所以两个地址都为1；不能未经配置把它们并到同一 RS485 总线上。
已有成功证据只支持此型号、模式及映射，其他型号需适配而非绕过校验。

## 2. 主机准备

解压完整项目后，在根目录打开终端：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
Copy-Item config/robot.example.json config/robot.local.json
Copy-Item config/grippers.example.json config/grippers.local.json
```

PowerShell禁止激活脚本时，可直接使用 `.\.venv\Scripts\python.exe` 代替后续 `python`，
不必修改全局执行策略。Linux用 `source .venv/bin/activate` 和 `cp`。

`robot.local.json` 对夹爪仅要求：

```json
{
  "ssh_host": "192.0.2.10",
  "ssh_port": 22,
  "ssh_user": "robot",
  "ssh_key_file": "~/.ssh/id_ed25519"
}
```

把示例地址、用户名和密钥换成自己的；`192.0.2.*` 不允许连接。
如使用 SSH agent，`ssh_key_file` 可为 `null`。先使用系统 SSH 登录，
向设备维护方核对主机指纹后建立 `~/.ssh/known_hosts`；
程序拒绝未知主机密钥，不内置密码。原开发机器的个人 SSH 脚本不再是依赖。

## 3. 取得厂家 SDK

向夹爪厂家取得本型号匹配的 `changingtek_p_rtu_Servo.py`，放入
`vendor/`，或在 `grippers.local.json` 的 `sdk_file` 写自己的路径。
必须提供 `MotorController` 及现场版本使用的
`temp_move`、`trigger_motion`、`_read_register(s)`、`_write_register(s)` 接口。
程序会把该文件上传到自己的机器人运行目录；不调用 SDK 示例的主函数。

SDK、厂家手册、驱动安装包没有随仓库重新分发。不能从名称相似的
第三方包推定接口兼容。`vendor/` 和本机配置已被 Git 忽略。

## 4. 枚举并填写 USB 身份

只需 SSH 配置，即可运行以下只读命令；不打开串口、不发送 Modbus：

```powershell
python tools/run_gripper_remote.py inventory
```

它显示每个 `/dev/ttyUSB*` 的别名和 `udevadm` 属性。据此填写：

- `devices[0].port` / `devices[1].port`：两个不同的 `/dev/serial/by-id/...`、
  `/dev/serial/by-path/...`，或设备维护方创建的固定别名。
- 每个 `identity`：真实 `ID_VENDOR_ID`、`ID_MODEL_ID`，以及
  `ID_SERIAL_SHORT` 或 `ID_PATH`。没有序列号的 CH340 使用物理 `ID_PATH`。
- `remote_directory`：本 SSH 用户可写的独立目录，例如 `/home/robot/luna-grippers`。
- `open_close_mapping_confirmed`：核对两只均为 0 张开、9000 闭合后改为 `true`。

示例的 `REPLACE_ME` 必须全部替换。程序核验真实设备身份和最终解析的串口，
拒绝两条配置实际指向同一设备。`ttyUSB0/4/5` 等编号会随插拔变化，不代表左右手。
若需区分左右，现场标记两只转接器后在自己的记录里对应；公开样例不猜测左右。

换 USB 口后 CH340 的 `ID_PATH` 可能变化，应重新枚举更新，不能直接删身份校验。

## 5. 机器人端安装

机器人需要 Python 3、`python3-venv`、`udevadm`、`fuser`、`timeout` 和用户串口权限。
Debian/Ubuntu 上缺组件时由设备管理员安装 `python3-venv`、`psmisc`、`udev`、
`coreutils`，将操作用户加入实际设备的串口组（常见为 `dialout`）后重新登录。
Windows CH341 驱动 ZIP 不是机器人 Linux 的驱动安装方式。

```powershell
python tools/doctor.py --component grippers
python tools/run_gripper_remote.py install
```

`doctor` 只检查本机文件和配置，不连接机器人。
`install` 明确建立远端独立 `.venv`，按 `requirements-gripper.txt`
安装 minimalmodbus 2.1.1、pyserial 3.5。它不修改系统 Python，
不使能夹爪、不改寄存器、不重启控制器。
机器人需要能访问 Python 包源；无法联网时在联网电脑准备依赖轮子，
复制到机器人后手动安装：

```text
联网电脑：python -m pip download -r requirements-gripper.txt -d wheels
机器人：python3 -m venv /home/robot/luna-grippers/.venv
机器人：/home/robot/luna-grippers/.venv/bin/python -m pip install --no-index --find-links /path/to/wheels minimalmodbus==2.1.1 pyserial==3.5
```

替换路径为自己的配置。初始设备使能、地址和位置映射由原厂软件/设备维护方准备；
程序不会自动清警、重设地址、改限位或使能未知设备。

## 6. 先读状态，再手动运行

```powershell
python run_grippers.py status
python run_grippers.py
```

菜单：`1` 张开两只、`2` 闭合两只、`3` 读取状态、`0` 退出。
菜单启动本身不运动。命令行也可以：

```powershell
python run_grippers.py open
python run_grippers.py close
python run_grippers.py close --speed 10 --force 30
```

默认参数在 `run_grippers.py` 顶部：

```python
SPEED_PERCENT = 10
FORCE_PERCENT = 30
IGNORE_DROP_ALARM = False
```

当前允许速度1～50、力矩1～30。**旧双臂程序的60%/100%参数与夹爪无关。**
每次选择只执行一个端点，结束保持位置，不自动张开、回零或去使能。
两个目标参数都读回一致后连续触发，软件近同时启动，不提供硬件同步保证。

遇到物体阻挡时可能到不了9000，本工具仍按到位超时处理，
**不是依据夹持力判断抓取成功的闭环抓取器**。到位位和力矩到达位也不证明已夹住物体。

## 7. 固件报警例外

本现场用户转述厂家答复：B=0x1000 为特定固件误报，空载也可出现。
11月13日的现场程序获准允许此位，其他 A/B 故障仍停止。
这不是所有夹爪固件的通用结论，也不是我们独立验证固件源码所得。

公开版默认 `IGNORE_DROP_ALARM = False`。只有厂家确认自己的设备适用该例外后，
才将其设为 `True`。仅在本程序判断中允许该位，原始报警留在日志里，
不改驱动器保护配置、不重复清警。底层命令行等价选项是 `--ignore-drop-alarm`。

## 8. 过程、退出与日志

每次通过 SSH 上传独立运行目录，锁定程序和串口，校验型号、模式、
通信参数、位置范围、静止反馈，再设置并读回参数。
动作异常时请求停止两路，并读取速度确认；Ctrl+C 使用运行标识核对后通知本次远端进程。
通信完全中断时，主机可能无法发送停止或确认反馈，不能把终端退出当作机械已停止。
单目标仍有远端12秒到位超时，外层进程有150秒上限；它们不是硬件急停。

日志在本机 `evidence/gripper-*.jsonl`，成功需看到
`sdk_test_complete` 和 `remote_exit_code: 0`，并核对两只最终位置和速度。
失败不自动重试运动。安装/枚举不产生运动；其他进程占用时拒绝接管。
远端 `runs/` 留存每次部署，可在没有运行进程时由维护方清理；不要删除 `.venv`。

## 9. 实测与局限

11月13日：分别五次往复、两路联合五次往复、力矩30%闭合均有成功记录；
两路启动确认相差约8～10ms，是日志时间差，不是机械同步误差测量。
11月16日现存日志再次包含闭合、张开、闭合成功，随后CH340只读打开再次EIO失败。
USB故障仍间歇出现，不能宣称已彻底修复。

本次公开版去除了个人目录与固定USB身份，增加可配置部署和离线测试；
没有用公开改造版再次驱动机器人。查阅
[夹爪现场全记录](../archive/points/GRIPPER_USB_CHECK_20261112.md)、
[验证记录](VALIDATION.md)和[问题索引](TROUBLESHOOTING.md)。
