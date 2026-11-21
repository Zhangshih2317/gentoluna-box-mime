# Gento Luna 开发文档与可复用程序

**2026-11-21 · v0.3.0**　[PICO 有线 VR、D435 1080p 视频和双腕 8 针夹爪还原指南](vr/README.md) · [实机故障与开发记录](vr/HISTORY.md) · [环境与厂家资料](vr/SOURCES.md)。使用腕部夹爪的操作者从 `vr/` 开始。

更新：**2026-11-16，v0.2.0**。包含双臂/HOME/BODY 示教、搬箱模拟、
双 USB-RS485 夹爪控制、屏幕电量与语音工作的代码、说明、问题记录和脱敏证据。

**第一次下载从 [完整导航](docs/INDEX.md) 开始。**
下载完整 ZIP 后解压，在项目根目录执行命令。文档和离线测试不需要机器人；
实际控制需要填写本机配置、SSH 访问以及对应厂家 SDK。
本项目不包含密码、私钥、厂家 SDK 和专有模型，不承诺任意机器人零配置即动。

| 你要做什么 | 入口 |
|---|---|
| 先了解整个开发过程 | [开发时间线](docs/DEVELOPMENT.md)、[完整原始记录索引](docs/INDEX.md) |
| 两只夹爪张开、闭合 | [夹爪安装与使用](docs/GRIPPERS.md)，`run_grippers.py` |
| 单轮/六轮搬箱模拟、独立回 HOME | [运控安装](docs/SETUP.md) |
| 看电量屏、语音和网络处理记录 | [辅助功能](docs/AUXILIARY.md) |
| 遇到报错或想核对实测范围 | [故障排查](docs/TROUBLESHOOTING.md)、[验证记录](docs/VALIDATION.md) |

## 双夹爪快速入口

按照 [GRIPPERS.md](docs/GRIPPERS.md) 完成一次性配置和依赖安装后：

```powershell
python run_grippers.py
```

菜单：`1` 同时张开，`2` 同时闭合，`3` 读取状态，`0` 退出。
也可使用 `python run_grippers.py open` / `close` / `status`。
默认速度 **10%**、力矩 **30%**，文件顶部有中文参数说明。
现场已确认 **0=张开、9000=闭合**；新设备须核对映射后才能运动。
每次只执行选定动作并保持，不自动循环。夹爪控制独立于搬箱模拟。

## 搬箱模拟

从人工示教、下蹲运动学、回原点、调速排查，到两个完整搬箱模拟程序的开发归档。
这两个流程模拟“抱箱、起身、转腰、张臂放开”；**搬箱流程本身不包含抓取检测、
夹爪联动或真实箱子搬运任务规划**。

## 已完成的两个程序

| 程序 | 动作 | 现场结果 |
|---|---|---|
| `run_box_mime_taught.py` | 完整搬箱一次，站起后相对 HOME 转腰 −45°，放开后回 HOME | 2026-11-13 最近三次完整成功；最后一次最大 HOME 误差 0.001° |
| `run_box_mime_six_cycles.py` | 完整搬箱六次，−45° / +45° 交替，左右各三次，每次回 HOME | 2026-11-13 六轮、48 阶段全部完成；最大 HOME 误差 0.002° |
| `run_return_home.py` | 独立恢复到保存的 HOME，不接续搬箱 | 已有实机成功记录；速度 30%，加速度 10% |

六轮演示的每轮都是：

```text
张臂 → 下蹲 → 合抱 → 站起 → 转腰45° → 站立张臂 → 双臂回HOME → 身体回HOME
```

六轮不是原地连续摆腰，也不是直接从 −45° 转到 +45° 的 90° 跨越。
最新原始事件、逐周期反馈及核对结果见 [验证记录](docs/VALIDATION.md)。
这里的实机成绩来自发布前的现场版本；公开版增加了部署配置，未再次驱动机器人。

## 从哪里读

- [开发大纲与阶段成果](docs/DEVELOPMENT.md)
- [全部已归类的故障、原因及处理](docs/TROUBLESHOOTING.md)
- [安装、依赖、连接与首次运行](docs/SETUP.md)
- [速度、下蹲、到位判定和模块结构](docs/ARCHITECTURE.md)
- [程序与资料清单](docs/PROGRAMS.md)
- [日志说明、脱敏及历史记录边界](records/README.md)
- [第三方依赖与许可](THIRD_PARTY.md)

## 快速开始

需要 Python 3.10+（现场 Windows 使用 3.12），以及带匹配 SDK、碰撞包、标定包和模型的机器人端 Linux 环境。
缺少这些专有/现场组件时，可阅读代码和运行纯离线测试，但不能直接连接运动。

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
Copy-Item config/robot.example.json config/robot.local.json
```

编辑 `config/robot.local.json`，填写自己的 SSH、控制器地址和远端路径。
示例地址 `192.0.2.*` 是文档占位地址，程序拒绝将它用于连接。
SSH 使用本机密钥或 agent，并核验已有主机密钥；参照 [安装说明](docs/SETUP.md) 准备依赖。

```powershell
# 只读拉取自己设备上的标定库；不发运动命令
python tools/fetch_frame_reference.py

# 在机器人端进行模型计算，不连接运动控制器
python run_box_mime_taught.py --prepare-cache
python run_box_mime_six_cycles.py --prepare-cache

# SDK 只读预检：默认不运动
python run_box_mime_taught.py --preflight
python run_box_mime_six_cycles.py --preflight

# 操作者显式启动，二选一；勿并发控制
python run_box_mime_taught.py --execute
python run_box_mime_six_cycles.py --execute
```

本仓库保留**已成功运行的示教点和 100% 速度 / 50% 加速度配置**，用于复现记录，
不是新设备的默认调试建议。每个入口顶部的 ARM/BODY 参数独立修改；百分比不等于实际关节角速度。
替换标定、模型、点位或运动源码后，必须为该设备重新生成缓存。

下蹲为相对 HOME 的躯干下降 **30 cm**；全身默认动作到位要求两次不同反馈均在各轴 **2°** 内，
两次间隔 **0.1～1 秒**。正常完成保持 Position，不切 IDLE。
独立回 HOME 与旧分步兼容分支保留各自检查，不能用 2° 规则接受任意未知起点。

模型检查是离散裸臂模型检查，不包含外部障碍和现场加装夹爪。
软件普通等待与硬故障处理不同：本仓库仍存在硬故障急停路径，不应当作已验证的带载保持方案。

## 目录与复用

```text
run_box_mime_taught.py       单轮入口
run_box_mime_six_cycles.py   六轮入口
run_return_home.py           独立回 HOME
run_grippers.py              双夹爪手动开合
config/                     可移植连接配置与 SSH 适配器
tools/                      规划、执行、反馈、测试
demos/box_mime_six_cycles/   六轮独立模块快照
points/                     示教点和数字参数；现场样例
docs/                       本次开发的完整说明
records/                    脱敏事件日志、压缩反馈、索引、验证摘要
archive/                    旧文档、旧点位、历史源码（.py.txt）
```

历史内容会有旧参数和阶段性结论，以本 README 和 `docs/` 为当前说明。
旧缓存仅保留在 `archive/` 供审计；不会被公开版入口用于启动。
本仓库代码采用 MIT 许可；厂家及第三方组件不包含在内。

## 下载后离线验收

```powershell
python -m pip install -r requirements.txt
python tools/run_offline_tests.py
python run_grippers.py --help
```

离线入口禁止网络连接。机器人现场成绩与公开部署改造后的离线测试分开记录。
源码 ZIP 包含所有公开文档、源码和归档；私有组件取得方式见 [THIRD_PARTY.md](THIRD_PARTY.md)。
