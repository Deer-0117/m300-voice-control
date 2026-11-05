# code10 当前通信盒代码

2026-11-05 本地视觉源码已整合上游 `75469fe8` 的 YOLO/HSV 可选检测器，默认 YOLO+HSV，纯像素模块保留为黄/浅绿场景选项。颜色范围、切换保护与测试见 [本地视觉说明](recognition/runtime/DETECTOR_SELECTION.md)；语音任务 API 与视觉检测分开。

以 2026-11-02 的 v4 快照为基础，已同步并重新通过 SSH 核对 2026-11-05 当前运行的 `flight-panel-20261105-v7`。包括气球 PSDK 覆盖层、本地视觉及任务保护、Agent/网络脚本、服务配置和回归测试；设备独立 M300 迁移目录的 38 个文件仍与根目录源码一致。

| 目录 | 内容 |
| --- | --- |
| [balloon-controls](balloon-controls/README.md) | 十项面板功能、维护开关、控制/预测/采样源文件、中英文布局、宿主入口、服务及核验清单、隔离测试 |
| [recognition](recognition/README.md) | API 语音候选/执行、任务握手、解码、YOLO/MIL、时间采样/分析、运行回归及 Agent |
| [network](network/README.md) | 现有代理、校时、网络分层检查与 API 联网工具 |

设置列表包含维护开关和九项任务/诊断操作；主界面六个快捷入口共享相同状态。维护默认开启，关闭仅允许随后显式启动实机任务；开关本身不发送运动指令。完整操作和门禁见 [面板说明](balloon-controls/README.md)。

部署仍对应 `/home/uav/offline_balloon_strike_test`、`/opt/uav-microsoft-agent`、`/home/uav/m300-migration-v1` 及设备完整 SDK。根部 `psdk/m300_runtime.c` 属于另一套独立宿主；不能把覆盖层放入根 `psdk/` 或同时注册两套 Widget。当前源码不是完整 DJI SDK或一键飞行部署包。

[SOURCE_MANIFEST.json](SOURCE_MANIFEST.json) 记录 148 项设备文件，147 项与仓库字节一致。`test_mission_buttons.py` 保留设备原哈希，并使用已通过 v7 隔离验证的仓库适配版本；其他新增测试和说明属于发布配套，不宣称都已安装到设备运行目录。

实际 Key、令牌、SSH 私钥、DJI 账户文件、模型、虚拟环境、运行数据和二进制不收录。此次仓库同步不改变设备模式或启动任务。构建及现场验收边界见 [同步记录](../../docs/CODE10_SYNC_20261105.md)。
