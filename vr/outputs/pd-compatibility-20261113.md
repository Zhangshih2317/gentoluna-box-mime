# Luna PD 模式兼容性与切换条件核对

日期：2026-11-13。此次仅只读检查，不调整倍率、不切换模式、不修改控制器配置或固件。

## 已确认的故障

此前单次 Apex mode=3 测试中，实际调用 FX_L1_State_SwitchToPDMode。双臂均进入模块状态 100，模块错误 106（ERR_RequestTorqueMode，请求力矩模式失败），SDK 随后返回 -10（等待超时）。躯干同次切换成功。

详见 [切换测试记录](mode3-test-20261113.md)。FXPlatform 的 JointImp 与 Apex 调用的 PD 不是同一个高层模式，不能直接将两者的结果互换。

## 控制器与 SDK

- 控制器版本：4.4.4（0x00040404）。
- Apex 实际加载 /usr/local/lib/libGentoSDK.so，报告 SDK 4.4.2（0x00040402）。
- Apex 后端包：kernelmind-apex 1.0.6.81g。
- 本地原厂资料的连接兼容规则要求 MAJOR、MINOR 一致；不兼容连接返回 -4。本次连接成功，通信延迟 2 ms。
- 因此基础连接兼容检查通过，不能仅因补丁版本 2 与 4 不同就认定不兼容；这也不证明所有 PD 行为完全兼容。
- 机器上三份 SDK 库均自报 4.4.2，但 SHA256 不同。不能凭版本号认为二进制完全一致，也没有依据直接替换运行库。
- 阅读的 SDK 源码与已加载二进制尚未建立可复现构建对应关系，源码分析作为辅助证据。

## 切换条件

本机 SDK 源码允许双臂从 IDLE 等状态切入 PD，检查 low_speed_flag=1，然后设置参数并请求底层 TORQUE 状态。该函数未见强制 Home 检查。未满足低速状态条件时应返回 INVALID_CONDITION，而此前实际结果是模块 106 后 SDK 超时。

此次只读反馈：双臂 cur=0、cmd=0、err=0，low_speed_flag=1，traj_state=0。此前切换开始时也处于 IDLE 且模块无错误。这些证据不支持把故障简单归结为“必须先 Home”或“未处于 IDLE”；无法据此排除控制器内部未公开的切换条件。

SDK 源码的等待循环只判断目标 PD 状态，未见在模块进入 ERROR 时立即返回具体模块错误。这可以解释 Apex 最终只展示超时，而底层已有 106。

## 伺服读取结果

| 对象 | 关节 | 读取到的版本 |
| --- | --- | --- |
| ARM0、ARM1 | J1–J4 | V3.0.2.23 |
| ARM0、ARM1 | J5–J7 | V3.6.0.12 |
| BODY | J1–J6 | V3.0.2.23 |

此次读取各伺服报警均为 0，物理状态为 REAL。报警为零不等于已证明支持力矩模式，亦不否定此前模块错误 106。躯干同版本的部分伺服切换成功，也不能据此证明双臂每个驱动器支持当前请求，或认定腕部版本就是根因。

未找到能将上述固件版本与支持模式逐一对应的原厂能力表；现有读取结果不能确认具体哪个轴拒绝切换。

## 配置核对与误判纠正

旧版 SDK 教程提到 robot.ini 中 JointPIDCtlType=1。

最初按 R.A0.BASIC.JointPIDCtlType / R.A1.BASIC.JointPIDCtlType 查询，接口都返回成功和 0。但负向对照发现，查询不存在的短参数名同样返回成功和 0，因此这些返回值不能证明参数存在或配置为零。此前“已确认配置不符”的判断已撤回。

随后通过官方文件导出接口，只读取得控制器 /home/FUSION/Config/cfg/robot.ini，保存为 实际配置备份（原件归档：`controller-robot-20261113.ini`）。

实际文件没有 JointPIDCtlType，双臂 BASIC 节均含 JointImpPIDType=1；用实际存在的完整参数名查询也均返回成功和 1。两个名字是否在不同固件中表示等价功能，仍需原厂确认，不能自行添加旧键或认定 PD 配置关闭。

## 尚需原厂或控制器深层诊断确认

1. 4.4.4 控制器与实际加载的 4.4.2 SDK 二进制是否为该台 Luna 的推荐组合。
2. 两类伺服版本对该 PD/力矩控制请求的支持范围及所需配置。
3. 切换时逐轴请求模式、实际反馈模式、拒绝原因，以及模块 106 的具体触发轴和条件。
4. 当前固件中 JointImpPIDType 与旧教程 JointPIDCtlType 的准确关系。

目前没有证据支持盲目升级、替换 SDK、延长超时或修改该 PID 参数作为修复。问题已定位到双臂进入力矩模式这一层，根因尚未完全确认。

## 结束状态与证据

最后检查时 apex-robot、apex-teleop、gento-unified-control-8760 均 inactive。只读 SDK 会话已断开。

原始证据位于 camera-diagnostics-20261111：

- 20261113-111213-servo-firmware-readback.json：版本、逐轴报警及状态。
- 20261113-111037-pd-state-machine-versions.json：SDK 源码片段及库版本、哈希。
- 20261113-111249-compatibility-guard-review.json：兼容性与接口检查。
- 20261113-111355-pd-gate-final-validation.json：不存在参数的负向对照。
- 20261113-111446-controller-config-readback.json：实际配置导出。
- 20261113-111526-actual-pid-config-confirmation.json：实际参数名读回及服务状态。

## 2026-11-13 中午继续诊断

本段覆盖上述“结束状态”：再次检查时用户已重新启动 Robot 和 Teleop。进入本次诊断前，接口已报告双臂状态 100、current_mode=-1、control_ready=false，后台记录了一次新的 mode=3 失败。本轮未发送模式切换、复位、Ready、Home、倍率或配置写入指令；没有关闭用户已经启动的服务。

### 新取得的控制器证据

利用 SDK 单独的 FX_FileClient_RecvFile 文件下载接口导出控制器日志。该接口独立连接 TCP 10240，不调用 System_Link、不申请运动控制权。

- 当前系统日志（原件归档：`controller-syslog-20261113.txt`）
- 包含本次启动过程的前一系统日志（原件归档：`controller-syslog-previous-20261113.txt`）
- 内核日志（原件归档：`controller-kern-log-20261113.txt`）

启动日志确认 EtherCAT master 0、1、2 均出现 OP 状态及工作计数 24/24。该证据说明启动完成时总线正常，不代表此后每个时刻都正常。

当前系统日志中，双臂 obj_id=0/1 的七轴各有 __FX_ServoFsmSetPCtrl_10 记录；__FX_ServoFsmSetTCtrl_11 记录仅见 obj_id=2 的六轴，与躯干切换成功的现象一致。日志未给出双臂的逐轴拒绝码。这是后续核对控制器力矩切换分支的线索，不能仅凭缺少某条日志认定指令没有下发或伺服硬件不支持。

控制器日志的墙上时间明显不同于电脑与 Orin 时间：日志例如标记 18:42:52，不能直接对应 Apex 同日显示的切换时间。保留原始时间与内核启动秒数，不修改控制器时钟。

### 诊断能力限制

Windows 本地较新 SDK 源码提供 FX_L1_Fbk_GetSystemMsg，并在 UDP 3725 接收消息；Apex 实际加载的 /usr/local/lib/libGentoSDK.so 未导出该接口。对尚未被占用的 3725 端口进行了 8 秒被动监听，收到 0 包。它不证明控制器从不发送消息，只说明本次观察窗没有收到。没有为补齐诊断功能而替换实际控制库。

已有日志与公共资料搜索均未补齐伺服支持矩阵或错误 106 的逐轴触发条件。没有得到可以据此更改 CtrlType、PID 类型或伺服模式映射的可靠依据。

### 当前结论与原厂核对内容

尚未修复模式切换。实机故障仍定位于双臂请求力矩模式失败，缺少控制器内部状态机的具体拒绝条件。

可交给原厂的准确问题是：

> Luna 控制器 4.4.4，Apex 实际 SDK 4.4.2。两臂 IDLE 且低速标志为 1 时，PD 切换均出现模块 106，SDK 返回 -10，同次躯干 PD 成功。两臂 JointImpPIDType=1、CtrlType=0；J1–J4 固件 V3.0.2.23，J5–J7 V3.6.0.12。请核对错误 106 在当前固件内的触发条件，并提供失败时逐轴的请求模式、实际模式反馈、状态字及伺服配置版本。请同时确认 CtrlType=0 的具体含义、推荐配套 SDK 构建，以及旧教程 JointPIDCtlType 和当前配置 JointImpPIDType 的关系。

本轮末次状态仍为双臂 100、current_mode=-1；Robot、Teleop active。没有执行新的运动测试，也没有调整速度。对应原始诊断记录为 camera-diagnostics-20261111 中的 20261113-1205* 至 20261113-121030-* 文件。
