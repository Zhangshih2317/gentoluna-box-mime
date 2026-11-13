# 程序与交付清单

## 当前可复用入口

| 路径 | 用途 | 默认行为 |
|---|---|---|
| `run_box_mime_taught.py` | 一次完整搬箱模拟 | 只读预检 |
| `run_box_mime_six_cycles.py` | 交替左右45°，六次完整搬箱 | 只读预检 |
| `run_return_home.py` | 单独回HOME | 只读预检 |
| `tools/fetch_frame_reference.py` | 获取自己的标定文件 | SFTP只读 |
| `tools/teach_pose.py` | 采集当前位置/保存人工示教点 | SDK只读 |
| `tools/read_frame_parameters.py` | 读取本机法兰参数供缓存计算 | SDK只读，写本机JSON |
| `tools/run_offline_tests.py` | 离线回归 | 网络调用被禁止 |
| `tools/summarize_records.py` | 分析公开归档日志 | 本机文件读取 |

## 核心模块

| 模块 | 责任 |
|---|---|
| `box_mime_taught_plan.py` | 八阶段流程、速度组、深度绑定 |
| `box_mime_taught_robot.py` | 运行、阶段流、保持/等待、日志 |
| `native_arm_motion.py` | 原生手臂完整目标、读回、独立组合采样 |
| `coordinated_joint_timing.py` | 共享进度及速度/加速度/jerk限时 |
| `mime_squat_height.py` | 根据示教分支解物理下降高度 |
| `position_acceptance.py` | 全身2°、两帧及时间窗 |
| `feedback_observation.py` | 反馈读取时序和新鲜帧 |
| `motion_wait_policy.py` | 普通等待、恢复条件和重新规划 |
| `box_mime_cache.py` | 源码、模型、点位、模式绑定的缓存 |
| `return_home_robot.py` | 独立恢复分支 |
| `box_motion_robot.py` | 几何检查与旧基类依赖，非推荐独立入口 |
| `luna_config.py` | 公开版配置校验与远端环境 |

六轮的同名模块位于独立快照目录。`box_mime_six_cycles.py` 扩展方向与循环，
通过每轮完成后回HOME衔接，缓存必须同时包含正负方向。

## 历史完成程序和辅助工作

这些源码以 `.py.txt` 放在 `archive/source/`，供查阅，不作为当前部署入口：

- `run_box_motion.py`：早期实现，事故后禁用，不能拿来绕过现行预检。
- `run_stand_up_10cm.py`、`stand_up_plan.py`：示教深蹲基础上的100mm站起。
- `run_body_home_5percent.py`：早期BODY低速回位。
- `run_dual_arm_large.py`、`dual_arm_five_moves.py`：双臂五组测试与同步轨迹。
- `run_grippers.py`、夹爪诊断与USB修复脚本：独立夹爪项目，未并入搬箱动作。
- `battery_screen.py`、`battery_audio.py`等：屏幕电量和语音提醒。
- `build_home.py`、`home_kinematics.py`等：坐标系、厂家FK与HOME候选研究。
- `analyze_taught_squat.py`、`analyze_body_speed_response.py`：下蹲及速度响应分析。

历史连接脚本可能引用旧路径或外部包，因此不宣称所有归档工具直接可运行。
厂商夹爪SDK、语音资源、参考代码仓库及私有连接函数没有发布。
辅助工作的现场结果与限制保留于 `archive/points/` 和 `records/evidence/`。

## 不覆盖现场程序

公开仓库在独立目录整理。当前机器人工作目录中的入口、参数、几何缓存和历史日志未被替换。
公开版因加入配置、改变部署源码哈希，需要重新生成本地设备的缓存。
发布时的差异记录见 `records/public-packaging.patch`。
