# 辅助功能与网络

这些是完整开发史的一部分，但没有全部做成通用安装器。
不要因为文件出现在历史归档中，就将其当作当前设备适用的安装脚本。

## 电量屏和语音

现场工作包括两块电池电量、电压、温度、充电状态；
反馈过期隐藏旧电量，低电量分级语音，开始/停止充电播报；
显示服务与开机配置、原显示管理器恢复记录也已归档。

详细记录：[BATTERY_SCREEN.md](../archive/points/BATTERY_SCREEN.md)。
历史源码在 `archive/source/tools/battery_screen.py.txt`、
`battery_audio.py.txt`、`battery_screen_prepare.py.txt`、
`battery_xserver_prepare.py.txt`。这些依赖现场 ROS 主题、显示会话、
音频文件和设备权限，尚未改造成跨设备自动部署入口。
语音素材未重新分发；需要按自己的使用许可生成或准备。
原记录写“开机启动已配置”不等同于整机重启验证完成。

## 网络：连机器人，但互联网不走机器人 Wi-Fi

现场明确要求机器人 Wi-Fi 只访问机器人内网，上网走另一块有互联网的网卡。
采用独立网段路由和接口优先级处理，保留机器人直连路由，
不能简单禁用 Wi-Fi 后还期待 SSH 正常。

迁移到自己的电脑时，先查看接口、地址和路由，再确定互联网默认路由与
机器人网段的接口；不要照抄旧接口名称、索引或网关。
Windows可只读运行：

```powershell
Get-NetIPInterface -AddressFamily IPv4
Get-NetRoute -AddressFamily IPv4
```

归档 `archive/source/tools/set_luna_wifi_priority.ps1.txt` 记录了当时的修复；
公开入口不自动执行网络修改。
网络能 ping 通只说明网络路径，不证明 SSH认证、串口或SDK控制权正常。

## 原厂软件与手动交接

机械臂使用 FXPlatform，夹爪使用 UServoController；
两个软件分别服务不同控制器，不能用夹爪连接成功推定机械臂端口空闲。
断开原厂串口连接后再运行Python；软件窗口仍在不一定占用串口，
应以端口占用与实际读取为准。
SDK、上位机及驱动从厂家取得，仓库不重新提供安装包。

## 资料来源与可核查范围

原设备文档入口、同事代码固定提交与厂家资料的使用边界，
保留在 [原始接入记录](../archive/README-original.md) 和
[THIRD_PARTY.md](../THIRD_PARTY.md)。有些在线资料需要登录，
仓库没有凭据，也没有把未经完整阅读的嵌入资料标为已核查。
