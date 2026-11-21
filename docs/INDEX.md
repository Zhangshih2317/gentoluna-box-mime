# 完整开发导航

2026-11-21 新增 [整套 VR 系统还原](../vr/README.md)、[排障与开发记录](../vr/HISTORY.md) 和 [环境版本及资料入口](../vr/SOURCES.md)。

范围：用户提供的2026-11-10交接包、11-11至11-16当前工作目录中的代码、
说明与证据。不是所有历史聊天的逐字稿；原交接包未包含的源码和安装包不能凭记录还原。

## 按用途阅读

| 顺序 | 文档 | 能回答的问题 |
|---|---|---|
| 1 | [项目首页](../README.md) | 有哪些功能、哪些可运行 |
| 2 | [双夹爪使用](GRIPPERS.md) | 从下载、配置、SDK到同时开合 |
| 3 | [运控安装](SETUP.md) | SDK、模型、标定、缓存、首次运行 |
| 4 | [开发时间线](DEVELOPMENT.md) | 需求如何变化、每阶段做了什么 |
| 5 | [结构与参数](ARCHITECTURE.md) | 规划、反馈、到位、速度与停止 |
| 6 | [程序清单](PROGRAMS.md) | 每个入口和模块的责任 |
| 7 | [问题索引](TROUBLESHOOTING.md) | 报错、证据、处理及未解决点 |
| 8 | [实测与发布验证](VALIDATION.md) | 哪些真机成功、哪些只是离线通过 |
| 9 | [电量屏、语音和网络](AUXILIARY.md) | 辅助工作的部署边界 |
| 10 | [日志说明](../records/README.md) | 如何读成功/失败、脱敏和时间差 |
| 11 | [第三方依赖](../THIRD_PARTY.md) | 哪些组件要向厂家/维护方取得 |

## 原始开发记录

以下内容按当时状态保留，可能包含旧参数、过期“下一步”和本机路径。
历史命令不是让下载者自动执行的指令；当前用法以首页和上述安装文档为准。

| 阶段 | 原始记录 |
|---|---|
| 11月10日 VR 还原 | [开发摘要](../archive/handoff-20261110/开发摘要.md)、[本机还原记录](../archive/handoff-20261110/本机还原记录.md) |
| 首次SDK/控制权接入 | [原README](../archive/README-original.md) |
| HOME及双臂五组 | [HOME](../archive/points/HOME.md)、[被替换的模型候选](../archive/points/HOME-model-candidate-superseded.md) |
| BODY事故与示教 | [事故记录](../archive/points/BODY_INCIDENT_AND_TEACHING.md)、[下蹲示教](../archive/points/taught/SQUAT_TAUGHT.md)、[角度分析](../archive/points/taught/SQUAT_REASONING-20261111.md) |
| 转腰与站起 | [转腰示教](../archive/points/taught/WAIST_TEACHING-20261111.md)、[站起100mm](../archive/points/STAND_UP_100MM.md) |
| 早期动作与废弃方案 | [初版抱箱](../archive/points/BOX_MIME.md)、[抱箱转腰放开](../archive/points/BOX_PICK_TURN_RELEASE.md) |
| 正式示教流程 | [新版搬箱](../archive/points/BOX_MIME_TAUGHT.md)、[协调测试](../archive/points/COORDINATED_MIME_TEST.md) |
| 独立恢复 | [回HOME](../archive/points/RETURN_HOME.md)、[BODY低速回位](../archive/points/BODY_HOME_5PERCENT-20261111.md) |
| 带载停止与调速 | [停止审查](../archive/points/LOADED_TOOL_STOP_REVIEW.md)、[手臂速度审计](../archive/points/ARM_SPEED_AUDIT-20261113.md)、[BODY协调](../archive/points/BODY_SPEED_COORDINATION_REVIEW-20261113.md) |
| 只读核验 | [只读验证](../archive/points/READONLY_VERIFICATION-20261113.md) |
| 双夹爪 | [完整USB/通信/动作开发记录](../archive/points/GRIPPER_USB_CHECK_20261112.md) |
| 屏幕与语音 | [电量屏记录](../archive/points/BATTERY_SCREEN.md) |

## 代码和证据放在哪里

- `run_*.py`：首页列出的可运行入口；默认行为见程序清单。
- `tools/`：公开适配后的模块与测试。
- `config/`：示例配置和SSH连接器；`*.local.json`仅保存在使用者本机。
- `points/`：数字点位样例，不能当作新机器已验证的原点。
- `demos/box_mime_six_cycles/`：六轮独立源码快照。
- `archive/source/`：历史代码文本，`.py.txt`防止被误当现行入口运行。
- `archive/points/`：全部阶段性Markdown/JSON记录及历史缓存。
- `archive/handoff-20261110/`：初始交接文档与验证证据的脱敏副本。
- `records/evidence/`：现场事件和反馈；大文件是gzip压缩文本。
- `records/media/`：已审核图片。
- `records/manifest.json`、`gripper-export.json`、`handoff-export.json`：来源与导出哈希。
- `CHANGELOG.md`：公开发布版变化；实际提交以Git历史为准。

## 能否下载即用

可以直接阅读全部文档、查看历史记录、安装Python依赖后运行离线测试。
连接真实设备前，首次必须提供自己的地址、密钥、USB身份和厂家SDK。
搬箱额外需要匹配的标定、碰撞库、URDF/网格和重新生成的缓存。
这些信息不应也不能用开发者个人设备的数据替使用者填好。

发布ZIP不要求电脑预先存在原开发路径；归档里的旧路径仅供读史。
公开运行入口不会自动运行网络修复、USB控制器重绑定、升级固件或清警。
