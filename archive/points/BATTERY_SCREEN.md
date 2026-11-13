# 机器人屏幕电量显示

2026-11-13 部署到域控 `192.0.2.10`，实体输出 DP-1，分辨率1520×720。
原GDM会话屏幕处于DPMS关闭状态；唤醒后，其合成显示仍输出黑画面。
现使用独立Xorg显示电量界面，替代本机屏幕的GDM登录界面。
没有修改GDM自动登录、机器人运动模式或控制租约。

## 行为

- 只订阅ROS Domain 0的 `/driver/battery/Data`，类型 `driver/msg/BatteriesData`。
- 不创建发布者，不调用运动、停止、复位、使能或充电控制接口。
- 两块电池分别显示百分比、充电状态、电压及温度，每秒刷新。
- 电量不高于20%显示低电量提示，不高于10%显示红色警告。
- 电池未连接、缺失、故障码非零或电量超出0～100会显示异常。
- 超过8秒没有新消息，或消息时间戳超过8秒未推进，隐藏电量并提示反馈中断。
- 保持屏幕常亮，程序异常退出后自动重启。
- 故障提示仅显示，不会自动停止机器人或执行回充。

## 语音提醒

2026-11-13 已接入自动语音。用户现场确认普通话女声、慢速及提高后的音量合适。
声音使用Windows本地 `Microsoft Huihui Desktop` 合成，Rate=-3，合成音量90。
播放时指定机器人板载模拟音频输出
`alsa_output.platform-sound.analog-stereo`，播放流音量100%，主音量按用户要求调至100%。
不使用此前语言和语速不合适的Speech Dispatcher播报路径。
本地音频无需联网，也不依赖机器人在线语音合成服务。

- 每块电池单独判断；不高于20%时播报“某号电池电量不足，请及时充电”，每5分钟一次。
- 不高于10%时播报“某号电池电量严重不足，请尽快充电”，每分钟一次。
- 连续3个不同时间戳的有效反馈确认后才触发，严重不足优先。
- 充电、缺失、未连接、故障、读数异常或反馈失联时不播报低电量；对应播报若正在播放则取消。
- 电量从普通不足升级为严重不足时，无需等待原5分钟间隔。
- 用23%和13%的恢复边界避免阈值附近反复触发；播放失败30秒后重试并记录错误。
- 音频以独立播放进程运行，不阻塞电量界面刷新；最长播放20秒。

充电状态变化也会提醒：任意一块电池进入充电时说“已开始充电”，
两块都停止充电时说“充电已停止”。状态需至少5个不同时间戳的有效反馈、
持续至少5秒确认，单次变化仅播报一次。程序启动只建立基线，不虚构插拔事件；
轮换充电和短暂接触抖动不触发，失联或缺失不解释成断开充电。
现有反馈只有每块电池的 `is_charging`，无法区分拔充电器和充满停充，
因此停止提醒不声称物理充电插头已经拔出。
未播出的过期事件会取消；播放失败30秒后重试；严重低电量优先于充电事件。

`tools/battery_audio.py` 实现提醒策略和播放，
`tools/generate_battery_audio.ps1` 可重新生成
`assets/battery-audio/` 中的6个中文音频。
机器人安装音频在 `/opt/luna-battery-screen/audio/`。
已在本机和机器人分别通过19项策略/播放测试，覆盖重复间隔、升级、
充电取消、失联、异常数据、阈值抖动、播放失败重试，以及开始/停止充电、
初始化、去抖动、双电池轮换、重复时间戳和过期事件取消。
测试未向真实电池话题发布模拟低电量数据。
未插拔实机充电器验证本次充电变化提醒。
语音接入前的界面备份位于机器人
`/opt/luna-battery-screen/backup-before-audio-20261113-092900/`。
反馈JSON新增 `audio` 字段，包含启用状态和最近播放错误；
日志事件以 `battery_audio_` 开头。

## 文件和服务

本地源文件：

- `tools/battery_screen.py`：普通nvidia用户运行的Tk界面和只读ROS订阅。
- `tools/battery_screen_prepare.py`：从实体Xorg会话复制认证到私有运行目录。
- `tools/battery_xserver_prepare.py`：创建仅本机使用的X认证。
- `tools/battery_audio.py`：独立判断和播放低电量语音提醒。
- `tools/luna-battery-screen.service`：电量显示服务。
- `tools/luna-battery-display.service`：独立Xorg显示服务，禁用X TCP监听。

机器人安装目录为 `/opt/luna-battery-screen/`。
两个服务文件位于 `/etc/systemd/system/`，均已启用。
`display-manager.service`别名指向 `luna-battery-display.service`。
运行时反馈为 `/run/luna-battery-screen/status.json`，该目录只允许nvidia用户和root访问。
原显示管理器别名记录为 `/opt/luna-battery-screen/original-display.json`；
原 `/etc/gdm3/custom.conf` 保持原样。

只读检查：

```bash
systemctl status luna-battery-display.service luna-battery-screen.service
cat /run/luna-battery-screen/status.json
journalctl -u luna-battery-screen.service -n 30 --no-pager
```

## 验证

已在机器人上验证8项电量状态检查，三个Python文件编译检查通过，
两个systemd服务文件校验通过。实体X显示输出截屏非黑屏，
已人工检查中文字形、电量、状态及布局。
部署后持续收到新反馈，1号电池从68%更新至69%，2号为77%。
原运控、底盘及后台的6个进程PID和启动时间保持不变。
当前实际运控进程为PID2532；
旧文档中的 `gento-unified-control-8760.service` 当时显示inactive，
不能仅以该旧服务名判断当前运控进程是否运行。
未重启机器人，不宣称已经完成整机重启验收。

截图为 `evidence/battery-screen-20261113.png`，
反馈快照为 `evidence/battery-screen-status-20261113.json`。

## 恢复原登录屏幕

这会关闭电量界面并恢复GDM登录显示。部署时保存的原别名目标为
`/lib/systemd/system/gdm.service`；恢复前应核对上述original-display.json。

```bash
sudo systemctl disable --now luna-battery-screen.service luna-battery-display.service
sudo ln -s /lib/systemd/system/gdm.service /etc/systemd/system/display-manager.service
sudo systemctl daemon-reload
sudo systemctl start gdm.service
```

`ln -s`不带强制覆盖参数，若别名已被其他工作修改，会拒绝覆盖。
上述命令不删除源文件、备份或电量反馈证据。
