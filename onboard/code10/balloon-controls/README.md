# code10 测试面板与 PSDK 控制覆盖层

本目录对应 2026-11-02 通信盒实际安装的 `flight-panel-20261102-v4`。`SOURCE_MANIFEST.json` 可逐项核对当前设备路径和 SHA-256。这里的 32 个 SDK 文件包括 Widget、控制/预测、相机采样、两语言布局/图标，以及 Linux 宿主入口与 CMake；不包含账户配置或平台库。

## 十项功能

| 设置列表 ID | 名称 | 行为 |
| --- | --- | --- |
| 69 | 仅显示/维护 | 默认开启；关闭进入实机测试待命，另点任务按钮才执行。 |
| 60 | 开始测试 | 任意气球任务，固定开启录像，按计划/录像确认后放行。 |
| 61 | API联网 | 原代理和校时链路的网络检查，不校验密钥或额度。 |
| 52 | 相机/姿态采样 | 地面静止确认后 30 秒采样，完成不代表时间标定有效。 |
| 53 | 部署/标定核验 | 分别报告文件、运行二进制及本次开机标定。 |
| 54 | Kalman预测模式 | 开启需通过严格标定检查，任务空闲时选择，下次任务生效。 |
| 65 | 执行API语音任务 | 确认本次 API 候选并执行；校验会话、时效及任务令牌。 |
| 66 | 新增API语音任务 | 首次停止旧任务并提示录音；识别后第二次点击才交 API 决策并执行新计划。 |
| 67 | 清除候选语音任务 | 取消本轮候选与识别；不停止已经运行的飞行任务。 |
| 68 | 停止测试 | 取消准备及控制任务，等待资源清理和控制权释放。 |

主界面 50、51、55、56、57、58 六个快捷入口分别映射同名功能。共 16 个唯一控件 ID，十个逻辑功能；旧 18–29 回调被拒绝。C 与中英文 JSON 必须成套使用。刷新界面请退出再进入 PSDK 设置列表并滚动查看。

维护开启时控制器初始化、任务和实机采样均被阻止。关闭开关仅改变本进程待命模式，不启动任务、申请控制权或转动云台；随后显式启动任务可以产生真实飞行/云台指令。两方向切换都要求任务、语音处理、采样、控制权及资源清理完成；忙时先按停止并等待。切换清候选、旧门及采样确认，预测复位 1；同值回调无副作用。每次进程启动都回到维护，未沿用旧 OFF 状态。

控制模式由显式运行时接口统一决定，服务里的 `BALLOON_NO_FLIGHT=1` 与 `GIMBAL_TRACKING=0` 仅为启动基线；维护关闭后的真实状态以开关回读为准。手动接管锁存和释放失败状态均会阻止自动重新申请/开启任务，停止不等于可恢复暂停。

## 文件与集成

- `psdk-overlay/`：相对于设备完整 SDK 的项目源文件；保留 PSDK 3.9.2 实际路径。
- `systemd/psdk-flight-panel-display.service`：现有面板服务配置，启动默认维护并沿用 liveview/视觉 worker 和控制参数。
- `deployment/flight_panel_manifest.json`：当前安装的源码和二进制哈希核验清单；用于运行时只读部署检查。
- `scripts/uav-api-functional-test`：保留设备已有的独立 API 诊断适配器源码；本版面板不再注册旧 27–29 诊断按钮。
- `tests/`：真实函数/控制适配层加 SDK 替身；没有链接 DJI 库或飞行输出。

运行模块位于 [recognition/runtime](../recognition/README.md)，网络工具见 [network](../network/README.md)。需要外置凭据、完整官方 SDK/平台适配与链接库、已有 Python 环境和模型。服务还要求旧 `psdk-balloon.service` 停用、原维护锁存在，避免双 SDK 宿主；这些前置条件不等于已经批准实机任务。本次不附带绑定一次性暂存目录的自动安装/解锁脚本。

只复制源码时仍需在设备 SDK 内完整重建并更新二进制核验清单，不能把此处 ARM64 二进制哈希用于另一套构建。当前无有效时间标定；采样和相对延迟估计不得直接写成有效曝光/姿态物理延迟。

## 本地验证

在本目录运行：

```sh
python3 -B -m unittest discover -s tests -v
TEST_PSDK_ROOT=/path/to/full-or-public-sdk python3 -B -m unittest discover -s tests -v
python3 -B tests/run_authority_stop.py --sdk-root /path/to/sdk
bash tests/run_native_no_aircraft.sh /path/to/sdk /tmp/panel-native-tests
PYTHONPATH=../recognition/runtime python3 -B -m unittest discover -s ../recognition/runtime -v
```

`TEST_PSDK_ROOT` 提供 `psdk_lib/include` 和 `samples/sample_c/module_sample/utils/cJSON.c`，未指定时仅控制适配层用例会明确跳过。其余面板/语音/核验测试直接使用本仓库源码。原生脚本面向 Linux GCC；macOS 可运行 Python 与编译替身测试。测试需要 Python 3.10+，运行回归另外需要 NumPy、Pillow，生产视觉还需设备现有 OpenCV/TensorRT 等依赖。

设备 v4 的 ARM64 构建、177 项 Python 和原生控制回归已通过；用户确认十项显示与维护 ON。实机来回切换、真实相机采样、任务执行和飞行均未完成验收。本次仓库同步不连接飞机执行控制，不增加实际云端请求。具体验证记录见 [本次同步记录](../../../docs/CODE10_SYNC_20261102.md)。
