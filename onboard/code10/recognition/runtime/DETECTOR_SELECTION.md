# code10 本地视觉识别

从 [Conti 仓库固定提交 `75469fe8`](https://github.com/Conti427/uav-balloon-collision/tree/75469fe8de98bc8e915f544e543010f49be8ee9a/offline_balloon_strike_test) 选择性整合 YOLO/HSV 检测模块。来源与本地适配哈希见 [VISION_SOURCE_MANIFEST.json](../VISION_SOURCE_MANIFEST.json)。这份文件描述源码和配置，不将上游测试或源码一致解释为本机实飞验收。

`balloon_detector_worker.py` 是共同入口，沿用 `balloon-yolo-worker.service`；公共任务门、目标文件、MIL 交接、计时和难例接口保留在 `balloon_yolo_worker.py`。一个进程只加载一种检测器，文件锁拒绝第二个检测进程占用同一输出通道。

```json
{"active_detector":"yolo","pixel_config":"pixel_callable_module/config.json"}
```

- 默认 `yolo`：本地 YOLO/TensorRT 检测通用 `balloon` 框，再对框内中央区域做 HSV 颜色分类；TensorRT 故障可以回退到本地 PyTorch。图片不发送给语言模型。
- 可选 `pixel`：在原图分辨率上做 HSV 阈值、连通域及轮廓形状过滤；不加载 Torch、Ultralytics、TensorRT 或神经网络权重，也不执行 YOLO 六区域补检。
- 配置缺失、损坏或模块名未知立即失败；不会静默换另一种算法。`BALLOON_PREDICTION_MODE` 是后续运动预测配置，与检测器选择独立。

## 颜色范围

YOLO 后的 HSV 分类保留上游三组：`pink` 对应粉/红/橙，`yellow` 对应黄/绿，`blue` 对应蓝/紫。紫色指令可以匹配蓝组气球，当前不能承诺蓝紫独立区分。`uav_capabilities.py` 的七种语音颜色名称不等于七种独立视觉类别。

纯像素模块当前**只输出 yellow**。随包参数是 2026-10-30 特定画面的浅黄/浅绿阈值：H 28–55、S 12–65、V 110–255，最小评分 0.98；没有紫色识别能力。该评分是形状与像素覆盖率的排序规则，不是模型识别概率。默认继续使用 YOLO，未对相机重新标定这些阈值。

## 配置与切换

YOLO 使用设备已有 `yolo_callable_module/weights/best.pt` 和可选 `best_fp16.engine`。模型和环境不入库。选择 TensorRT 的环境配置为 `BALLOON_YOLO_BACKEND=tensorrt`、`BALLOON_TRT_ENGINE=<engine路径>`；YOLO 配置为 640 像素输入、置信度 0.25。原有 `yolo_env` 路径名不代表像素模式会导入神经网络。

只读检查无需 sudo，也不操作服务：

```sh
cd /home/uav/offline_balloon_strike_test
/usr/bin/python3 -B switch_detector.py --status
```

切换仅用于地面维护。当前维护开关保存在面板进程内部，辅助程序无法可靠验证其状态，因此要求 `psdk-balloon.service` 和 `psdk-flight-panel-display.service` **都已停止**，并且 `balloon_strike.armed`、`balloon_strike.trigger`、`balloon_mission.token`、`balloon_mission.commit`、`balloon_mission.ready` 均不存在；不会代替操作者停止 SDK 服务或清除这些任务标记。

```sh
sudo /usr/bin/python3 -B switch_detector.py yolo
# 仅对匹配上述黄/浅绿参数的测试场景使用：
sudo /usr/bin/python3 -B switch_detector.py pixel
```

切换器归档旧检测请求/目标与配置，只重启切换前已经运行的检测和跟踪进程。原先停止的服务保持停止。失败时恢复旧配置，并尝试恢复原先运行的视觉服务；不会启动任何 SDK 服务。单独修改 JSON 不会热切换现有进程。

语音转文字与 API 文本任务解析仍保留。根目录 `app/model.py` 的视觉 API 是另一套独立宿主实现，不是本目录 YOLO/HSV 检测器的调用路径。

## 隔离验证

在装有 NumPy、OpenCV、Pillow 的环境中运行：

```sh
python -B -m unittest discover -s tests -p 'test_detector*.py' -v
python -B -m unittest discover -s tests -p 'test_pixel_interface.py' -v
```

测试覆盖单检测器分支、无效配置失败、纯像素进程禁止神经库导入、文件/帧接口、颜色与 TSV 输出、难例元数据，以及切换时两个 SDK 服务和任务门拦截。测试使用临时图像和服务替代函数，不调用 DJI 或执行飞控动作。真实相机颜色准确率、模型效果、飞行跟踪和碰撞效果需要现场独立验证。
