# code10 当前通信盒代码

2026-11-02 通过 SSH 读取当前项目源码，归档 `flight-panel-20261102-v4`。本次包括气球 PSDK 控制覆盖层、52 个运行与回归源码文件、现有 Agent/网络脚本和服务配置；另核对设备独立 M300 迁移目录的 38 个文件，与根目录源码一致。

| 目录 | 内容 |
| --- | --- |
| [balloon-controls](balloon-controls/README.md) | 十项面板功能、维护开关、控制/预测/采样源文件、中英文布局、宿主入口、服务及核验清单、隔离测试 |
| [recognition](recognition/README.md) | API 语音候选/执行、任务握手、解码、YOLO/MIL、时间采样/分析、运行回归及 Agent |
| [network](network/README.md) | 现有代理、校时、网络分层检查与 API 联网工具 |

设置列表包含维护开关和九项任务/诊断操作；主界面六个快捷入口共享相同状态。维护默认开启，关闭仅允许随后显式启动实机任务；开关本身不发送运动指令。完整操作和门禁见 [面板说明](balloon-controls/README.md)。

部署仍对应 `/home/uav/offline_balloon_strike_test`、`/opt/uav-microsoft-agent`、`/home/uav/m300-migration-v1` 及设备完整 SDK。根部 `psdk/m300_runtime.c` 属于另一套独立宿主；不能把覆盖层放入根 `psdk/` 或同时注册两套 Widget。当前源码不是完整 DJI SDK或一键飞行部署包。

[SOURCE_MANIFEST.json](SOURCE_MANIFEST.json) 记录 136 项设备文件的路径、设备哈希与仓库哈希。除一个测试文件按 v4 已验证版本和仓库路径适配外，导出文件字节与设备一致。新增面板测试和说明属于打包内容；不伪称设备上所有测试文件都已安装更新。

实际 Key、令牌、SSH 私钥、DJI 账户文件、模型、虚拟环境、运行数据和二进制不收录。此次仓库同步不改变设备模式或启动任务。构建及现场验收边界见 [同步记录](../../docs/CODE10_SYNC_20261102.md)。
