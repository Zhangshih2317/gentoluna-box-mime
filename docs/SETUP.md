# 安装与首次运行

## 1. 环境前提

主机：Python 3.10+、SSH连接能力；现场使用Windows/Python3.12。
机器人端：Linux、可用的 `GentoRobot` SDK及动态库、`collision_guard` exact后端、
完整机器人URDF/网格、`calibration_transforms`及设备标定数据。

这些现场附加包没有一起发布；详见 [第三方依赖](../THIRD_PARTY.md)。
第一次运行前确认设备上的实际安装位置，不要照抄开发机器的路径。

主机依赖：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
Copy-Item config/robot.example.json config/robot.local.json
```

Linux主机激活环境使用 `source .venv/bin/activate`，其余Python命令相同。

## 2. 连接配置

编辑 `config/robot.local.json`：

- `ssh_host`、`ssh_port`、`ssh_user`：机器人端Linux的SSH连接。
- `ssh_key_file`：本机私钥路径；也可设为 `null` 使用SSH agent或默认密钥。
- `controller_ip`：SDK控制器地址，不是Linux主机地址。
- `remote_env_script`：远端bash环境脚本，设置SDK需要的动态库/Python环境。
- `sdk_root`：包含 `PYTHON_SDK/` 和 `robot/` 的目录。
- `model_path`：全身URDF绝对路径，其引用的网格应可访问。

配置文件不含密码字段。将密钥加入SSH agent，使用正常SSH首次连接并按自己的设备信息核对主机指纹。
Python适配器采用 `RejectPolicy`，不会自动信任未知主机。
Windows OpenSSH通常写入 `~/.ssh/known_hosts`，Paramiko会读取该文件。
可用环境变量 `LUNA_CONFIG` 指向项目外的配置。

代码将控制器IP、SDK目录和URDF路径嵌入远端进程环境，之后才加载执行模块。
不会修改机器人服务配置或安装软件。程序仍需要当前客户端拥有控制权；
FXPlatform、网关或另一个演示进程未释放时，先由操作者协调断连。

## 3. 获取自己设备的运动学组件

```powershell
python tools/fetch_frame_reference.py
```

仅SFTP读取指定的9个文件，不连接运动控制器、不更改远端文件。
读取到 `reference/robot_frames/calibration_transforms/`，Git默认忽略该目录。
源文件哈希记录在新生成的 `evidence/ground-frame-source-manifest.json`。
缺少这些文件时请向设备维护方取得对应版本；不要伪造标定映射或删除哈希检查。

## 4. 核对示教点与数字参数

公开点位是原设备的样例，不能自动当作另一台机器的已验证轨迹。

主入口使用：

- `points/BOX_START_BEFORE_INCIDENT.json` 的 `pose` 作为 HOME。
- `points/taught/SQUAT_TAUGHT-20261111-150201.json` 作为深蹲分支。
- `points/taught/WAIST_TAUGHT_01_UI-20261111-153805.json` 作为转腰示教。
- `points/SDK_FRAME_PARAMETERS.json` 作为离线缓存的SDK法兰数字参数。
- 独立HOME使用 `points/HOME_RECOVERY_PREFLIGHT_REFERENCE.json`。

`tools/teach_pose.py` 只读反馈，支持保存新的站姿/蹲姿文件；新文件不会自动替换上面入口的引用。
换自己的点位要同时审查入口、规划约束和恢复参考，而不是只换文件名。
原设备的数字法兰参数来自 `20261111-162613` 只读FK核对。
新的设备可用 `tools/read_frame_parameters.py` 只读采集自身参数；
不匹配时启动预检会拒绝，不要直接修改缓存里的哈希。

## 5. 缓存与只读预检

公开副本修改了部署源码，旧设备缓存只留在归档中；先生成新的缓存：

```powershell
python run_box_mime_taught.py --prepare-cache
python run_box_mime_six_cycles.py --prepare-cache
python run_box_mime_taught.py --preflight
python run_box_mime_six_cycles.py --preflight
```

准备缓存通过SSH运行模型计算，不建立运动SDK连接。
预检会连接SDK读取实际状态和厂家FK，但不启用运动、切换模式或改倍率。
若头部保持值与样例有微小差异，可按实际观测使用 `--prepare-cache --cache-head YAW PITCH ROLL`；
该选项限离线缓存、且与保存HOME头姿相差不超过0.05°。
它不是重新示教头部或放宽实际姿态差异的工具。

## 6. 手动执行

确认当前姿态属于检查通过的起点，调整入口顶部的速度/加速度后：

```powershell
python run_box_mime_taught.py --execute
# 或：
python run_box_mime_six_cycles.py --execute
```

独立回原点：

```powershell
python run_return_home.py --preflight
python run_return_home.py --execute
```

前台终端保持打开。SSH输入心跳断开、通信或硬故障仍会触发原异常处理。
正常完成保持Position；不能把程序退出等同于机器人已机械制动。
收到当前姿态不匹配时，读取实际点位再处理恢复，不自动重放历史参考。

## 7. 离线测试

无需机器人、SDK或标定文件：

```powershell
python tools/run_offline_tests.py
```

已提供本机合法标定组件后，测试高度求解、两个生成入口、完整六轮模拟：

```powershell
python tools/run_offline_tests.py --with-calibration
```

测试入口禁止网络连接，控制器调用均使用模拟对象。GitHub Actions只运行无需专有组件的测试。
CI通过不等于实机路径、载荷或环境已经验证。
