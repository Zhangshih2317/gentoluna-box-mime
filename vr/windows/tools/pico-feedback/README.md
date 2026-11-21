# PICO 反馈中继

`relay.cjs` 在 Windows 接收来自 Luna 的 UDP `9002/9003`，按端口和长度封装后送入 ADB 转发。`PicoFeedbackRelay.java` 在头显本地接收帧，将数据送到 Apex 的回环端口。`Start-PICO-Feedback.ps1` 会推送随项目提供的 `classes.dex` 并启动 `app_process`。日常使用只需 Node.js、ADB 和已授权的 USB 连接。

现场 DEX 由 Java 8 字节码、Eclipse ECJ 4.6.1 和 R8/D8 3.3.75 生成。需要重新编译时，从 [Maven Central](https://repo.maven.apache.org/maven2/org/eclipse/jdt/core/compiler/ecj/4.6.1/) 与 [Google Maven](https://dl.google.com/dl/android/maven2/com/android/tools/r8/3.3.75/) 获取两个 JAR，在本目录运行下列命令。ECJ 4.6.1 使用带 Java 8 标准库的 JDK。

```powershell
java -jar ecj-4.6.1.jar -1.8 -d . PicoFeedbackRelay.java
java -cp r8-3.3.75.jar com.android.tools.r8.D8 --min-api 26 --output . PicoFeedbackRelay.class
```
