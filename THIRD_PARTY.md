# 第三方依赖与来源

MIT 许可覆盖本仓库自行编写的动作编排、诊断、测试、文档和脱敏记录。
它不授予厂家 SDK、二进制库、URDF、网格、驱动、手册、语音素材或第三方软件的许可。
这些文件没有随本仓库发布，请从厂家或设备中按适用授权自行安装。

## 必须自行提供

- 现场使用的 GentoRobot Python SDK 及其动态库。
- `robot/collision_guard`：本项目依赖的现场碰撞检查包及其 exact backend。
  **不能仅安装一个普通 Python SDK 就假定具备此包。**
- 与设备一致的全身 URDF、网格和碰撞后端。
- `robot/calibration_transforms`：本项目所用 BODY/HEAD 标定与运动学包。
  普通厂家 SDK 分发包未必包含这个现场附加组件；缺失时须先取得并验证，
  不要下载名称相似的包替代。
- 远端 Linux Python 环境脚本及必要动态库路径。使用原有设备部署或自建等价环境，
  示例配置中的路径不是通用的厂家安装路径。

`tools/fetch_frame_reference.py` 仅将用户自己设备上的指定文件通过 SFTP 读回本机，
保存在 Git 忽略的 `reference/`。它不下载或再分发厂家软件。

## Python 第三方库

NumPy、Paramiko 按各自上游许可证使用；本仓库不包含这些库的代码。
安装版本以运行时环境记录为准，`requirements.txt` 是兼容范围，不是厂家认证清单。

## 历史参考

开发时参考过 `wangbo20261102/20261111Gentoluna-testing` 的本地副本，
固定提交 `62a4405cb918c8d38774352bf8bbd29414d053a2`。
该参考仓库、现场 SDK 源码和原始厂家手册均未复制到这里。
SDK 源码核对结论保留为开发记录，不能等同于对固件内部行为的验证。
