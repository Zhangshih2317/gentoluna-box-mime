# 已被用户示教替代的模型候选点

历史记录：当前有效 HOME 已由用户示教确认，见 `HOME.json` 和 `HOME.md`。
本文下述数据只对应 `HOME-model-candidate-superseded.json`，不得用作当前原点。

点位保存在 `HOME.json`，未下发机器人，也未覆盖控制器、Apex 或网关的 HOME 配置。
`execution_enabled=false`。本次只读取状态、调用正运动学和离线计算。

## 目标关节角

单位：度。左右臂均按各自 SDK 关节正方向记录。

| 轴 | 左臂 ARM0 | 右臂 ARM1 | 实测反馈限位 |
| --- | ---: | ---: | --- |
| J1 | 90 | 90 | -173～173 |
| J2 | 90 | 90 | -118～118 |
| J3 | 90 | 90 | -173～173 |
| J4 | -90 | -90 | -145～45 |
| J5 | 90 | 90 | -173～173 |
| J6 | 0 | 0 | -60～60 |
| J7 | 2.596357 | -2.596356 | -90～90 |

J4 的 +90°超限，采用 -90°；J6 不能达到 ±90°，保留 0°。
在前五轴取上述目标时，J7=90°会使法兰轴线接近竖直，偏离水平约 87.174°，
且只调整 J6 无法纠正。因此优先满足水平要求，用 J7 作左右臂各自的补偿。
全部目标在反馈限位内，最小限位余量为 28°。

## 水平定义及验证范围

这里约束的是 SDK 末端/法兰局部 Z 轴与地面平行，未约束手掌面或绕法兰轴的转角。
地面暂以厂家 `Underpan_Base_link` 的 XY 平面表示：假设底盘与现场地面平行。
尚未通过地面测量或姿态传感器核验这一假设，因此不能称为实测地面水平。

换算使用机器人已有的 `calibration_transforms` 身体/肩部坐标链，读取身体六轴反馈，
将 SDK 正运动学结果变换到底盘坐标系。该坐标链加载时检查厂家 URDF 校验值及身体轴映射；
没有使用相机外参。两个最终目标分别再次调用厂商 SDK 正运动学复核，
法兰轴线相对上述底盘水平面的计算倾角均小于 0.001°。
该数值是数学核算残差，不代表实体机械精度。

两侧目标法兰轴线在底盘系中约为 `[-0.999591, 0.028584, 0]`，
即主要指向底盘 -X 方向。身体、自身双臂、环境碰撞及到达路径均未验证。
旋转矩阵用于水平判断，未依赖欧拉角奇异位置的 ABC 数值。

此点依赖身体六轴姿态（度）：

```text
[-0.744, 9.281, -17.364, 5.456, -0.296, -1.653]
```

身体姿态或底盘倾斜变化后应重新计算，不能继续沿用固定 J7 补偿。
本记录没有向身体或头部下发目标。

## 本次姿态和反馈

采样开始时间：2026-11-11 11:21:17，北京时间。以下为当次快照。

| 项目 | 左臂 ARM0 | 右臂 ARM1 |
| --- | --- | --- |
| 状态 | IDLE | Position |
| 错误码 | 0 | 0 |
| 速度 / 加速度比例 | 5% / 5% | 20% / 20% |
| J1～J7，度 | 57.710, -61.271, -55.492, -114.509, -4.068, 2.437, -0.986 | -59.490, -60.847, 60.365, -113.859, 23.839, 35.713, -61.368 |

`HOME.json` 保留完整原始反馈：角度、速度、限位、状态、倍率、力矩、末端位姿、
工具参数和身体状态。力矩标记为 `SDK_RAW_UNVERIFIED`，不能当作已标定的 N·m 数据。
短租约已交还，网关 SDK 连接保持。

## 证据及重算

- 当前反馈：`evidence/robot-status-20261111-112117.json`
- 腕部 SDK 探测：`evidence/home-ground-probes.json`
- 坐标源文件清单及校验值：`evidence/ground-frame-source-manifest.json`
- 离线求解：`evidence/home-ground-solution.json`
- 最终 SDK 复核：`evidence/home-ground-vendor-verified.json`
- 原先只相对 SDK 坐标系水平的候选：`HOME-sdk-frame-only.json`，不用于地面 HOME。

本地重算和复核流程：

```powershell
python tools/build_ground_home.py --feedback evidence/robot-status-20261111-112117.json
python tools/home_kinematics.py --jobs points/home-ground-validation-jobs.json --output evidence/home-ground-vendor-verified.json
python tools/build_ground_home.py --feedback evidence/robot-status-20261111-112117.json --verify-fk evidence/home-ground-vendor-verified.json
```

第一步仅求解；第二步请求只计算的 SDK 正运动学接口；第三步验证通过后写入候选点。
使用新的身体姿态时，应先通过 `tools/robot_status.py` 获取新反馈并替换上述反馈文件参数。
依赖本机 NumPy、SciPy 及已核对的 `reference/robot_frames` 坐标库。
