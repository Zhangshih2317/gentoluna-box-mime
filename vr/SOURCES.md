# 环境、校验值和资料入口

## 经现场核对的组合

| 项目 | 版本与依赖 |
|---|---|
| 硬件 | Luna 双臂；Orin ARM64；Intel RealSense D435；PICO A9210、两只控制器；两只 CTAG2F90D 各接腕部 8 针 485A；PC 有线网卡、USB 数据线 |
| Orin | Ubuntu 22.04.5 LTS，`5.15.148-rt-tegra`，L4T 36.4.4，ROS 2 Humble。现场 ROS 域 24。环境入口 `/etc/apex/apex_ros_env.sh` |
| Apex | `kernelmind-apex`、`kernelmind-apex-tool` 均 `1.0.6.81g`，Humble/arm64。`APEX_ROBOT_PLATFORM=gento`、`APEX_ROBOT_MODEL=luna`，原厂工具配置 `dm` |
| 编译 | C++17、CMake 3.22.1、GNU binutils、Git、GCC/G++ 11。ROS 包 `ament_cmake rclcpp std_msgs sensor_msgs geometry_msgs std_srvs tf2 marvin_msgs`；SDK 头文件 `/usr/local/include/gentosdk` |
| 电脑 | Windows PowerShell、Node.js（现场 v24.18.0）、Android platform-tools 37.0.1-15733141 / ADB 1.0.41、gnirehtet Rust win64 v2.5.1。仅可选维护与诊断用 Python、Paramiko、Playwright |
| 头显/上位机 | Android 14，包 `com.KernalMind.Apex_Teleop`，PICO APK `1.0.7_pxr`。Windows 现场保留 Apex Teleop 1.0.33 和单独试装的 Apex-UI 1.1.7.1。复现从 1.0.33 开始 |

镜像须已有厂家 BSP、型号标定、ROS 工作区、原厂 SDK、设备网络和服务配置。安装器按原厂文件比对，异版设备应先取得匹配镜像。固件恢复与出厂标定需由设备所有者按厂家手册完成。

## 软件获取

按[固定文件名、上游下载地址与 SHA-256 清单](dependencies.json)获取安装包。后端和 Tool 对应同一版 **Orin Gento**。PICO 选 `pxr`。Windows 平台工具和 gnirehtet 可直接运行 `windows/Install-UsbTools.ps1` 下载校验，也可通过 `-PlatformZip`、`-GnirehtetZip` 指向已保存的原始 ZIP。

| 用途 | 发布入口 |
|---|---|
| Orin 后端与 Tool | [KernelMind Apex Deb](https://github.com/Gento-Teleoperation-Apex/KernelMind_Apex_Deb/releases) |
| 头显 APK | [KernelMind Apex VR APK](https://github.com/Gento-Teleoperation-Apex/KernelMind_Apex_VR_Apk/releases) |
| Windows 上位机 | [KernelMind Apex Web](https://github.com/Gento-Teleoperation-Apex/KernelMind_Apex_Web_Deb/releases) |
| 固定 SDK 源码 | [提交 5a5f2433](https://github.com/Gento-Teleoperation-Apex/Gento-SDK/tree/5a5f24332aba242a99b6c39d3f4ffde312d705ee/SDK_CT040404_SDK040402_260617/SDK/00040402/C_SDK) |
| 厂家平台和其他 SDK | [天机机器人 SDK](https://github.com/tianjizn/tianji-robot-SDK)、[厂家 Apex 资料中心](https://github.com/Gento-Teleoperation-Apex/Apex_doc_center) |
| 腕部交付参考 | [MdZenus 原始项目 37a7068](https://github.com/MdZenus/gentoluna-box-mime/tree/37a7068) |
| USB 网络 | [gnirehtet v2.5.1](https://github.com/Genymobile/gnirehtet/releases/tag/v2.5.1)、[Android Platform Tools](https://developer.android.com/tools/releases/platform-tools) |

```bash
cd ~
git clone https://github.com/Gento-Teleoperation-Apex/Gento-SDK.git
git -C Gento-SDK checkout --detach 5a5f24332aba242a99b6c39d3f4ffde312d705ee
```

现场系统 `g++-11` 的版本为 `11.4.0-1ubuntu1~22.04`。复现厂家 SDK 机器码使用独立提取的 `gcc-11`、`g++-11`、`cpp-11`，三个包的版本均为 `11.4.0-1ubuntu1~22.04.3`（Jammy arm64）。从 Ubuntu Jammy Updates 的 [GCC-11 源包历史](https://launchpad.net/ubuntu/+source/gcc-11)或有该版本的 Ubuntu 软件源取得。保留系统 C++ 头文件，按以下方式提取到 `build_sdk.sh` 的第三个参数目录。

```bash
mkdir -p ~/luna-gcc11
for pkg in gcc-11 g++-11 cpp-11; do
  apt-get download "${pkg}:arm64=11.4.0-1ubuntu1~22.04.3" || break
  dpkg-deb -x "${pkg}_11.4.0-1ubuntu1~22.04.3_arm64.deb" ~/luna-gcc11 || break
done
```

若软件源只保留新版，应按固定版本从 Ubuntu 归档取包，再执行 `dpkg-deb -x`。`sha256sum -c wrist-grippers/apex/factory.sha256` 核对原厂库和节点。现场最终定制库 SHA-256 为 `2dc21fc7b1e9ab53c83b854d8ecdb23fb041e5c8d8e23e77edad82f36683f6c9`；重新构建也需通过脚本的基线段比较、符号检查和回归测试。

## 天机 Gento 对外知识库

这些链接来自项目所有者提供的厂家入口。钉钉可能要求登录或由厂家授予文档权限。

- [Luna 用户手册](https://alidocs.dingtalk.com/i/p/WO4GjWeoDKJeQzDR/docs/vy20BglGWOq299NKUDzEQ2YeJA7depqY)
- [Skye 用户手册](https://alidocs.dingtalk.com/i/p/WO4GjWeoDKJeQzDR/docs/ydxXB52LJqKnGGjAC2eO95mpWqjMp697)
- [Luna / Skye 开箱使用教程](https://alidocs.dingtalk.com/i/p/WO4GjWeoDKJeQzDR/docs/KGZLxjv9VGYqPPKZFqRyrv6Y86EDybno)
- [Platform 上位机使用说明](https://alidocs.dingtalk.com/i/p/WO4GjWeoDKJeQzDR/docs/14lgGw3P8vgRGG2EFn3oRjExW5daZ90D)
- [运控 SDK（1004 版本）](https://alidocs.dingtalk.com/i/p/WO4GjWeoDKJeQzDR/docs/XPwkYGxZV3vZ11axUrb9BqgKWAgozOKL)
- [遥操相关资料](https://alidocs.dingtalk.com/i/p/WO4GjWeoDKJeQzDR/docs/ydxXB52LJqKnGGjAC24pZE49WqjMp697)
- [底盘相关资料](https://alidocs.dingtalk.com/i/p/WO4GjWeoDKJeQzDR/docs/N7dx2rn0JbnY33PRCbRym635JMGjLRb3)
