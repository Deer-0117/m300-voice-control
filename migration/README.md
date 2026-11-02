# 通信盒迁移与独立宿主工具

本目录同步当前 `/home/uav/m300-migration-v1/migration` 中的项目源码，不复制设备已安装的 Pillow/OpenCC、模型、私有 staging 树、构建输出或运行记录。Python 依赖由根部 `requirements.txt` 安装；已有设备可以继续使用 `migration/python-deps`、`migration/text-deps` 的离线依赖。

- `configure.py`：在 Jetson ARM64/Python 3.10 上交互写入源码目录外的私有配置，隐藏 Key 输入且不覆盖已有配置。
- `inspect-host.py`：检查原有宿主入口和 SDK 条件。
- `prepare-host.py`、`host/`：在独立私有目录构建新宿主，按固定锚点停用旧业务入口，保留原账户初始化。源布局不匹配时退出。
- `check-host-build.py`：校验新二进制，仅执行初始化 SDK 之前的 `--build-info` 分支。
- `probe-api.py`：需要操作者显式运行的真实 API 探测；本次同步验证未运行此命令。
- `test-on-jetson.sh`：在 Jetson 执行软件/SDK替代函数测试。
- `run-on-jetson.sh`：读取外置配置后只运行 Python 应用，不负责启动或切换 PSDK 服务。

新增宿主保持 `M300_ALLOW_FLIGHT=0`，CMake 拒绝关闭 ground-only。实际 RID 来自现场核验，不提供默认坐标。完整 SDK 和应用账户文件仍由设备本地提供。`prepare-host.py` 生成的私有副本可能含 SDK 账户头或 JSON，不能提交到 GitHub；本仓库只发布转换器和宿主项目代码。

本地验证入口：

```sh
python3 -m pip install -r requirements.txt
sh scripts/test.sh
```

需要完整 Jetson SDK 的构建和现场启动应按源码内的设备前提分别进行。当前 GitHub 更新没有执行真实 API 探测、宿主启动或飞行操作。
