# code10 运行、识别与规划模块

组合测试由 `widget_test_options.py` 核对当前检测进程、开机时钟与配置。`balloon_mission.py` 在计划握手中携带 `detector` 和 `prediction`，执行端确认后继续使用 v7 的视觉预检。省略两个选项的旧任务调用仍使用原握手格式。导出的后台遥测可由 `analyze_background_telemetry.py` 分析。

2026-11-05 选择性整合 Conti `75469fe8` 的 [YOLO/HSV 可选检测模块](runtime/DETECTOR_SELECTION.md)：默认使用本地 YOLO+HSV；纯 HSV 像素模块仅对随包黄/浅绿场景可选。图片识别不调用 API，API 保留文本任务解析。YOLO 颜色目前仍是粉/红/橙、黄/绿、蓝/紫三组，未宣称七色独立识别。此次新增来源与适配见 [视觉来源清单](VISION_SOURCE_MANIFEST.json)。

`runtime/` 以 2026-11-02 的 52 文件快照为基础，新增 v7 本地视觉模块、任务绑定回执及回归测试；运行源来自 `/home/uav/offline_balloon_strike_test`，`agent/` 来自 `/opt/uav-microsoft-agent/app`。本轮重新通过 SSH 核对来源；`test_mission_buttons.py` 使用已验证的 v6 语音入口与 v7 视觉握手回归版本，保留设备旧测试哈希，差异记入上级来源清单。

| 模块 | 作用 |
| --- | --- |
| `uav_voice_command.py` | 普通录音经 ASR 和真实 API interpret 生成候选；确认时校验会话、开机周期、时效与任务令牌。新增任务首次仅 ASR，第二次点击才调用 API。 |
| `balloon_mission.py`、`balloon_clock.py` | 准备、执行端计划确认、录像回执、COMMIT 和同次开机时钟握手；取消使迟到结果失效。 |
| `uav_natural_language.py`、`uav_capabilities.py` | 既有解析、能力校验和本地计划入口；新 API 面板链路不以本地规则冒充 API 决策。 |
| `balloon_liveview_decoder.py`、`balloon_detector_worker.py`、`balloon_yolo_worker.py`、`balloon_tracker_worker.py` | 画面解码、统一检测入口、本地 YOLO/HSV 与 MIL 追踪输入；一个进程只加载一种检测器。 |
| `detector_selector.py`、`pixel_callable_module/`、`switch_detector.py` | 默认 YOLO 的显式配置、黄/浅绿纯像素检测、只读状态及双 SDK 服务停止后的切换。 |
| `capture_camera_timing.py`、`calibrate_camera_timing.py` 等 | 相机/姿态采样、时钟和相对时间分析，保留有效性门限。 |
| `flight_panel_tools.py` | 精确布局/部署核验、本次开机时间标定门禁和有界采样。 |
| `hardcase/`、相关工具 | 既有难例采集、协议和回放分析；不把数据回放结果当实飞证据。 |
| `agent/uav_microsoft_agent.py` | Microsoft Agent Framework 受限规划，解释与提交分开；新面板使用 interpret 后在本地验证/提交。 |
| `systemd/` | 现有服务配置，凭据继续从设备外置环境文件读取。 |

模式和控件见 [面板说明](../balloon-controls/README.md)。旧独立 API 诊断适配器仍保留源码，但新控件不再提供旧编号的三项诊断入口。根 `app/` 对应另一套 M300 独立宿主，没有与本目录的连续追踪控制混合运行。

`agent/requirements.txt` 保留此前设备交接的版本说明，不为了源码同步重建环境。生产视觉需要已有模型、NumPy/OpenCV/TensorRT/解码工具和设备环境；API/ASR 地址、鉴权信息由既有私有配置管理。`config/uav-microsoft-agent.env.example` 只有空 Key/令牌，不复制真实环境文件、任务计划、音视频或 API 返回。

本次同步不安装或重启服务，也不运行真实任务。源文件与仓库适配记录见 [来源清单](../SOURCE_MANIFEST.json)，测试和现场边界见 [同步记录](../../../docs/CODE10_SYNC_20261105.md)。
