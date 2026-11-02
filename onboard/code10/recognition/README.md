# code10 既有识别与受限规划模块

这些源码是 API 功能测试按钮实际复用的机载模块。`runtime/` 来自 `/home/uav/offline_balloon_strike_test`，`agent/` 来自 `/opt/uav-microsoft-agent/app`，本次同步前已核对设备 SHA-256。根目录 `app/` 对应 `/home/uav/m300-migration-v1/app`，提供测试画面的 `LiveCamera` 和 `ModelClient`。

| 模块 | 作用 |
| --- | --- |
| `runtime/uav_voice_command.py` | 使用已有远端 ASR 服务转写 PCM；生产入口另有候选、确认与取消 |
| `runtime/uav_natural_language.py`、`uav_capabilities.py` | 本地解析、能力校验和确认后计划入口 |
| `runtime/balloon_mission.py`、`balloon_clock.py` | 气球任务协调与同次开机时钟协议 |
| `agent/uav_microsoft_agent.py`、`uav_capabilities.py` | Microsoft Agent Framework 受限颜色顺序规划，解释与提交分开 |
| `systemd/` | 服务模板，只引用设备外置环境文件 |

`agent/requirements.txt` 记录设备交接资料中的 Agent Framework 版本；已有服务使用 `/opt/uav-microsoft-agent/venv`，不要为源码归档重建其环境。按钮测试只调用 `transcribe`、`interpret_with_agent` 和根应用的画面识别接口，不调用生产候选/计划写入或提交入口。

配置模板 `config/uav-microsoft-agent.env.example` 的 Key 和鉴权令牌均为空。实际值应只存在设备私有 `/etc/uav-microsoft-agent.env` 或进程环境中，不能提交 Git。ASR 和本地能力网关鉴权令牌也须另行在设备外置配置中填写。本目录不包含这些真实配置、模型、录音、计划和 API 返回。

这是对既有 code10 气球控制链路的源码同步。文件依赖原有 SDK、服务和部署路径；本目录不会自动安装、启动服务或改写根目录另一套 `psdk/m300_runtime.c` 控件。本次没有重新部署或验证实飞。
