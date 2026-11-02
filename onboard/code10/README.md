# code10 通信盒新增代码

2026-11-02 从当前设备源码和已核验本地快照同步。这里收录现有气球 PSDK 链路上新增的 API 网络、遥控器测试按钮与受限识别/规划模块；不改变仓库根部 M300 独立宿主的控件和飞控接口。

| 目录 | 内容 |
| --- | --- |
| [network](network/README.md) | code10→code12 SSH 代理、校时、分层网络检查和 API 联网 helper，含安装/回滚与测试 |
| [balloon-controls](balloon-controls/README.md) | 当前 Widget 源码覆盖层、中英文布局、ASR/画面/意图测试适配器及隔离测试 |
| [recognition](recognition/README.md) | 测试实际复用的 ASR、能力网关、任务协调和 Microsoft Agent 源码及服务模板 |

主界面 18–23 保留当前任务操作，配置页 26–29 为 API 联网、语音识别测试、画面识别测试、任务意图测试。功能测试只回显识别结果，保留任务互斥、取消及录音隔离。它们不提交或执行测试计划。

部署位置仍对应现有 `/home/uav/offline_balloon_strike_test`、`/opt/uav-microsoft-agent`、`/home/uav/m300-migration-v1` 和现有完整 PSDK。网络安装器有显式安装入口；Widget 覆盖层需要在完整 SDK 内单独构建，不能整目录覆盖根部 `psdk/` 或与另一套 Widget 同时启动。

真实 API Key、鉴权令牌、SSH 私钥、设备账户配置、模型、虚拟环境、运行日志、音视频和二进制未收录。`SOURCE_MANIFEST.json` 记录此次核对的源码路径和哈希；源码同步不证明本次已重新部署或完成飞行验证。
