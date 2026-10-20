在 Jetson 本机编译 ARM64 whisper.cpp，不能复制 macOS 的 whisper-cli 二进制。
把你使用的 whisper.cpp 源码放入此目录的 whisper.cpp/ 下：

    cmake -S third_party/whisper.cpp -B third_party/whisper.cpp/build -DWHISPER_BUILD_TESTS=OFF
    cmake --build third_party/whisper.cpp/build --target whisper-cli -j2

将多语言 ggml-base.bin 模型放到 third_party/whisper.cpp/models/ggml-base.bin。
可以复用原项目已经下载的这个模型文件。模型可以跨架构，二进制程序不可以。
确认安装 libopus（Ubuntu 包 libopus0）。
项目未打包第三方 SDK、模型或 macOS 编译产物。

也可以在目标设备执行 sh scripts/install-whisper.sh。脚本按 whisper.lock.json
固定源码提交与模型 SHA-256，在新目录内下载并本机编译，不更改旧项目或设备服务。
