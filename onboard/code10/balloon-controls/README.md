# code10 API 测试按钮与气球 PSDK 覆盖层

此目录保存 2026-11-02 通信盒已安装的 API 联网与功能测试按钮源码。`SOURCE_MANIFEST.json` 记录机载路径和 SHA-256；薄适配器、Widget C、两份布局及本地头文件均来自已核对的部署文件。

本目录用于现有 `psdk-balloon.service` 对应的气球控制 PSDK 工程。仓库根部的 `psdk/m300_runtime.c` 是另一套宿主集成；本次归档没有将两个 Widget/控制运行时合并，也没有在通信盒上安装、重启或启动任务。

## 文件和运行依赖

- `scripts/uav-api-functional-test`：机载 `/usr/local/bin/uav-api-functional-test` 的原样源码，分别调用既有 ASR、视觉和意图接口，不新增识别客户端。
- `psdk-overlay/samples/sample_c/module_sample/balloon_widget/`：Widget 实现、中文/英文布局及引用的项目头文件。目录结构对应设备完整 SDK 下的相对位置。
- `tests/`：现有 19 项 Python 适配测试、六项 C 隔离测试以及当前两语言布局与 C 处理项的一致性检查。

运行时仍需要设备已有的完整 DJI PSDK 3.9.2 工程、平台适配与链接库、`balloon_photo_detector` 实现和 `widget/test_widget_speaker.h`。它们没有随本覆盖层复制为一套完整 SDK。识别模块来源与依赖见 [识别代码说明](../recognition/README.md)，联网脚本见 [网络组件](../network/README.md)。

适配器保持设备路径：ASR 来自 `/home/uav/offline_balloon_strike_test/uav_voice_command.py::transcribe`；意图来自 `/opt/uav-microsoft-agent/app/uav_microsoft_agent.py::interpret_with_agent`；视觉来自 `/home/uav/m300-migration-v1/app/{vision,model}.py`。Python 环境、模型、API Key 和服务配置需由现场独立管理。

## 当前按钮

| 页面 | 编号 | 操作 |
| --- | --- | --- |
| 主界面 | 18–23 | 任意撞击、粉黄撞击、执行语音、停止、录像、跳过 |
| 配置页 | 24 | 检查网络连接 |
| 配置页 | 26 | API 联网 |
| 配置页 | 27 | 语音识别测试：等待下一段新录音并仅回显文字 |
| 配置页 | 28 | 画面识别测试：获取新实时帧并仅回显描述 |
| 配置页 | 29 | 任务意图测试：使用近期 ASR 文本或示例，仅回显理解结果 |
| 配置页 | 25 | 清除待执行语音任务 |

功能测试回显以“仅测试”开头，并包含“未执行任务”。测试使用独立进程和结果目录，保留超时、取消、相机资源互斥及测试录音隔离。测试 worker 禁用生产计划写入口；这是应用代码隔离，不是操作系统沙箱。停止按钮负责取消测试并清理资源。API 联网仅验证代理/HTTPS，成功不代表 API Key、模型权限或额度有效。

此版本的 26–29 编号专用于 API 功能，不能混用其他仓库将这些编号映射为任务快捷按钮的布局或二进制。完整 SDK 集成时，C 与中英文 JSON 必须成套构建和验证；本目录没有自动安装入口。原来的日期专用部署器绑定旧文件哈希和一次性暂存目录，未作为可重用部署工具收录。

## 本地验证

在此目录运行：

```sh
python3 -B -m unittest discover -s tests -v
python3 -B tests/run_widget_isolation.py
```

第二条需要 POSIX C 编译器和 pthread。它从本目录的真实 Widget 源码抽取六个被测函数，再使用 SDK 替代函数和故障注入验证取消、相机清理、线程/进程创建失败及录音隔离；不会连接设备或调用真实 API。临时 C 文件、二进制和 PCM 在系统临时目录中清理，不进入 Git。

这些结果不等同于完整 ARM64 SDK 链接、当前实机摄像头验收或飞行验证。凭据、录音、画面、运行结果、部署日志、原件备份和二进制均未收录。
