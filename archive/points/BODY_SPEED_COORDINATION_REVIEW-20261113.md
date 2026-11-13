# Luna 下蹲调速规律与 SDK 核对

日期：2026-11-13。对象：相对 HOME 下降 300 mm 的搬箱模拟动作。

后续更新：用户要求接入后，协调时间规划和等待恢复重规划已接入搬箱入口，
默认 10% 供手动测试，详见 `COORDINATED_MIME_TEST.md`。
以下保留接入前的研究记录；动态延迟补偿仍未标定或启用。

本次只分析已有实机日志、读取 Jetson 上的 SDK 文件并做离线计算。
没有连接机器人 SDK、发送运动、切换控制模式、修改控制器参数或更改现有执行入口。
入口当前由用户设置的 80% 保持原样。

## 结论

1. 可以制定固定的协调计算算法，但现有记录不足以标定一张可靠的各轴动态补偿表。
2. 相同的控制器百分比不代表相同的关节物理速度。BODY 的运行时速度、加速度比例各只有一个标量。
3. 当前指令已经按不同关节位移分配速度；真正缺的是实际响应差异的测量、轨迹限加速度/限 jerk，以及等待恢复时的平滑过渡。
4. 记录显示 B3 的进度比 B2、B4 落后。下蹲和站起的误差符号相反、规律相似，支持“动态跟随差异”的解释；不能据此排除其他机械因素。
5. Position 模式内部还会处理目标轨迹。外部 50 Hz 插值不是直接伺服透传。不能把外部规划的速度、加速度当成实际关节速度、加速度。

## SDK 证据

来源是 `/home/nvidia/tianji-robot-SDK` 的现场安装源码快照。
副本在 `reference/sdk_motion_review/`，逐文件 SHA-256 记录在
`evidence/body-speed-response-20261113.json`。
公网检索未取得可独立核验的厂家发布说明，以下不宣称为厂家最新发布版本。
现场固件内部代码、已安装二进制与源码完全一致性尚未验证。

| 项目 | 源码位置 | 核对结果 |
|---|---|---|
| BODY 比例配置 | `PYTHON_SDK/GentoRobot.py:354` | `BODY_SET` 中速度、加速度各一个标量；PD 的 K/D 才是六轴数组 |
| 运行时比例 | `PYTHON_SDK/GentoRobot.py:2123` | `runtime_set_speed_ratio(obj_type, vel_ratio, acc_ratio)`，范围 1–100，按部件设置 |
| 位置指令含义 | `VENDOR_SDK_README.md:440` | 单目标由内部控制器在速度/加速度限制下平滑逼近 |
| Position 示例 | `PYTHON_EXAMPLE/example_basic_Position.py:91` | 发一个目标，等待反馈到位，再发下一目标 |
| 逐轴上限 | `C_SDK/Kinematics/MotionPlanner/FXAxisPln.cpp:2684` | 关节速度上限乘速度比例，关节加速度上限乘加速度比例 |
| 时间同步 | 同文件 `:2759` | 找最大完成时间，把其他轴时间缩放到同一终点时刻 |
| 参数来源 | `C_SDK/L1Robot/L1Robot.cpp:6017` | 手臂读取 `R.A*.L*.BASIC.VelMax` 和 `AccMax`；未取得 BODY 对应真实配置 |
| MoveJ 适用范围 | `C_SDK/Kinematics/MotionPlanner/FXMotionPlanner.cpp:109` | 此接口限制机械臂序号 0/1，不能直接当 Luna BODY MoveJ 接口 |
| 同步双臂 | `C_SDK/Kinematics/MotionPlanner/FXAxisPln.h:65` | 明确是固定 BODY 的双臂规划，不是下蹲三轴动态补偿 |
| L1 缓存轨迹 | `C_SDK/L1Robot/L1Robot.cpp:4369` | `Config_SetTraj` 只有 ARM0/ARM1 分支，BODY 落入不支持分支 |
| PD 直通 | `PYTHON_EXAMPLE/example_basic_PD.py:15` | 文档描述为周期位置命令，不进行插值 |
| BODY PD 支持 | `C_SDK/L1Robot/L1Robot.cpp:2355` | Luna BODY 分支存在，需六轴 K/D 与周期配置；本次没有启用 |
| 可记录反馈 | `PYTHON_SDK/GentoRobot.py:1166` | BODY 包含命令位置、实际位置、速度、传感器力矩、gyro 字段 |

L0 存在 BODY 轨迹缓冲函数，不等于当前 L1 Python 接口支持完整 BODY 缓冲执行流程。
PD 在这里的文档名称为 Position-Direct，同时暴露 K/D；不能未经辨识把它视为 Position 模式的等价替换。
gyro 字段的物理分量、单位和安装坐标还需要厂家定义，不能直接按某一单位解释。

### 发现的源码疑点

`C_SDK/Kinematics/MotionPlanner/FXAxisPln.cpp:2655`：

```cpp
FX_DOUBLE ar = vel_ratio;
```

后面夹紧了 `acc_ratio`，但使用 `m_AccLmt[i] * ar`，该局部实现没有采用传入的加速度比例。
外层 `OnMovJoint` 虽然正确把两个参数传入，内层仍存在这个疑点。
这是现场 SDK 的离线 MoveJ 规划器代码问题，**不能据此推断控制器固件相同，也不是当前程序摆动的已证实原因**。
当前搬箱程序走 Position + `runtime_set_joint_pos_cmd`，没有调用该 MoveJ 规划器。
厂商源码副本未被修改。

还要区分两个比例单位：运行时比例接口使用 1–100；此 MoveJ 内部规划器使用 0.01–1。

## 三次实机记录

全部是 30 cm、控制器加速度比例 5%，均完整返回 HOME。
下表等待仅统计运动中 `torso_sum_deviation`，不包含正常终点等待。
“记录最大值”不是连续采样得到的真实峰值。

| 速度 | 日志后缀 | 下蹲/站起时间 | 下蹲/站起姿态等待次数 | 记录角度和偏差最大绝对值 |
|---|---|---|---|---|
| 30% | `132948` | 18.08 / 18.08 s | 0 / 0 | 1.417° |
| 40% | `133244` | 14.14 / 14.12 s | 2 / 2 | 2.007° |
| 80% | `133502` | 11.36 / 11.34 s | 4 / 4 | 2.062° |

完整文件名为 `evidence/box-mime-taught-20261113-<后缀>.jsonl`。
80% 时每次运动中冻结约 0.34–0.62 s，下蹲累计约 2.02 s，站起约 2.00 s。
软件冻结位置后从原进度恢复，未对恢复过程重新安排连续的速度/加速度。

### 表观滞后分析

只选取每段第一次等待前的数据，反解原五次插值进度，避免把等待时间误拟合成关节延迟。
该值包含内部规划、关节响应、采样时序的共同影响，不是直接测得的纯通信延迟。

| 速度与样本 | B2 表观滞后 | B3 表观滞后 | B4 表观滞后 |
|---|---:|---:|---:|
| 30%，下蹲约 10.02 s | 0.373 s | 0.492 s | 0.344 s |
| 40%，下蹲第一次等待前 | 0.389 s | 0.518 s | 0.354 s |
| 80%，下蹲第一次等待前 | 0.400 s | 0.508 s | 0.368 s |

运动中段 B3 比 B2 约多滞后 0.10–0.13 s；但同一次 30% 运行不同位置的 B3 表观滞后约在 0.34–0.49 s 变化。
因此不能简单写死“B3 永远提前 0.12 s”，更不能把这张表直接作为实机前馈参数。
对目标单独作时间超前，还会改变已检查的关节空间路径和瞬时姿态约束。

## 固定算法应分成两层

### 一、几何协调和时间参数化：可以固定

本次下蹲 B2/B3/B4 位移：

```text
Delta q = [+57.547449683, -103.774089593, +46.226639909] degrees
比例     = [+0.554545455, -1, +0.445454545]
```

所有轴共享一个进度 `s(t)`：

```text
q_i(t) = q_i(0) + Delta q_i * s(t)
v_i(t) = Delta q_i * s'(t)
a_i(t) = Delta q_i * s''(t)
```

这样位置、速度、加速度之和都满足 B2+B3+B4 的姿态配合要求。
不能只要求各轴同时结束：各自使用不同形状的速度曲线，即使结束时间一致，途中也可能改变角度和。

对已标定的每轴参数，用户只输入速度百分比 `p`：

```text
V_i(p) = min(V_i_reference * p / p_reference, V_i_calibrated_max)
A_i    = A_i_calibrated
J_i    = J_i_calibrated
```

加速度和 jerk（加速度变化率）不再无条件随速度百分比放大。
使用现有五次曲线时，统一时间取：

```text
T = max over all moving axes of:
    1.875 * abs(Delta q_i) / V_i(p)
    sqrt((10/sqrt(3)) * abs(Delta q_i) / A_i)
    cbrt(60 * abs(Delta q_i) / J_i)
```

将 T 向上取整到控制周期。较弱的轴决定速度上限，其余轴按同一进度配合。
当加速度、jerk 或实际能力成为瓶颈，继续增大速度百分比不会无限缩短时间。

已提供离线实现 `tools/coordinated_joint_timing.py`，六项测试通过，包括不同轴上限、姿态约束、速度饱和、加速度/jerk 上限和无效输入。
测试使用合成参数，**没有把它们作为机器人参数**。缺少运动轴标定上限时算法明确报错，不填猜测值。
该模块未接入当前执行入口。

### 二、实际跟随补偿：参数尚不能固定

现有日志通常每五秒或状态改变才记录一次，不足以辨识动态模型。
应在同一控制进程按每周期或至少 50 Hz 同时记录：

- 单调时间、实际发送间隔、RT frame serial、新鲜帧标志；
- 外部发送目标、SDK 回显位置、实际位置和速度；
- BODY 力矩/gyro 原始字段、B2–B4 角度和误差；
- 阶段、进度、冻结/恢复状态、控制器速度与加速度比例；
- 对双臂分别保存输入目标和内部 `Joint_Cmd`，避免把两者当成同一信号。

先分别识别下蹲和站起、不同姿态区间的响应，再使用独立的一次运行验证结果。
只有交叉验证通过的参数才能进入固定配置；不能用同一次数据拟合又宣称验证成功。
当前没有各轴 BODY 真实 `VelMax/AccMax`、有效 jerk 上限和高频响应记录，所以报告标记 `calibration_valid=false`。

实施方向是优先保持原几何点位和共同进度，限制加速度/jerk，并在误差接近阈值时平滑调整共同进度速度；
保留原硬故障和偏差保护，同时消除恢复时直接接回原速度的行为。
如果评估 PD 直通，需要先核对厂家 BODY 参数和周期语义，并单独验证，不能通过改一个模式号直接替换。

## 复查

```powershell
python "C:\Users\operator\Documents\ChatGPT\luna\tools\analyze_body_speed_response.py"
python -m unittest discover -s "C:\Users\operator\Documents\ChatGPT\luna\tools" -p test_coordinated_joint_timing.py
```

生成数据：`evidence/body-speed-response-20261113.json`。
本次没有更改 `run_box_mime_taught.py`、运动模块、当前速度/深度或几何缓存。
