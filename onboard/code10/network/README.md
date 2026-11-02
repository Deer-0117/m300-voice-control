# code10 API 网络组件

2026-11-02 整理通信盒现有代理、校时、网络分层检查和遥控器“API联网”辅助代码。网络检查、Agent 代理检查、wrapper 和 API 联网 helper 已按通信盒文件 SHA-256 核对。安装器本次增加统一管理 API 联网 helper；这一打包改动只通过本地回归，未在本次同步中重新安装到设备。

## 数据流与边界

```text
code10 应用 → 127.0.0.1:18080 → SSH 隧道 → code12 127.0.0.1:18080 → 既有上游代理 → HTTPS API
```

当前链路使用 code10 `192.168.168.155`、code12 `192.168.168.222`，USB 维护地址为 `192.168.55.1`。这些是现有设备网络的配置，迁移到其他设备前应核对地址和受限 SSH 授权。本包只配置应用代理，不修改网卡、MAC、默认网关或路由，也不修改 PSDK/飞控源码。

API 联网操作启动/启用既有隧道和校时服务，再用不带凭据的请求检查 `TCP → CONNECT → TLS → HTTP`。它不重启 Agent，不调用模型、不发送 Authorization。收到 `401` 可以证明 HTTPS 传输可达，不能证明 Key、额度或模型权限有效。界面显示“API网络已通；密钥和额度未验证”。

## 文件与安装目标

| 源码 | 设备目标/作用 |
|---|---|
| `systemd/uav-code12-proxy-tunnel.service` | 回环 SSH 代理隧道，失败后自动重连 |
| `systemd/uav-code12-time-sync.service`、`.timer` | 启动及约每十分钟通过受限 SSH 校时 |
| `systemd/zzzzzzzzzzzz-api-network.conf` | Agent 加载网络专用环境的 drop-in |
| `config/uav-code12-proxy.env` | `/etc/uav-code12-proxy.env`；只含代理与本地绕过地址 |
| `scripts/uav-code12-time-sync` | `/usr/local/sbin/uav-code12-time-sync` |
| `scripts/uav-api-with-proxy` | `/usr/local/bin/uav-api-with-proxy` |
| `check_network.py` | `/usr/local/bin/uav-api-network-check` |
| `check_agent_proxy.py` | `/usr/local/bin/uav-api-agent-proxy-check`，仅报告白名单代理字段是否匹配 |
| `scripts/uav-api-network-connect` | `/usr/local/bin/uav-api-network-connect`，遥控器 API 联网入口 |

Key、Bearer token、SSH 私钥、真实 API 返回和运行日志不包含在此目录。模型凭据继续由现有 `/etc/uav-microsoft-agent.env` 等设备私有配置加载；网络安装器不读取或覆盖该文件。

## 前置条件

设备必须是 Linux/systemd，已有 `uav` 用户及 `uav-microsoft-agent.service`。code12 已有可用上游代理，且校时所取的 code12 UTC 时间正确。通信盒需通过既有运维流程准备并核验以下受限 SSH 文件；本包不生成、不复制、不覆盖它们：

```text
/home/uav/.ssh/code10_code12_proxy_ed25519
/home/uav/.ssh/known_hosts_code12_proxy
/home/uav/.ssh/code10_code12_time_ed25519
/home/uav/.ssh/known_hosts_code12_time
```

代理身份在 code12 上只能转发到回环 `127.0.0.1:18080`，校时身份只能读取时间。不要改为自动密码登录或关闭主机密钥验证。

## 使用

从仓库根目录查看安装计划和运行不联网的测试：

```sh
python3 onboard/code10/network/install.py plan
python3 -B -m unittest discover -s onboard/code10/network/tests -v
```

将本目录部署到通信盒后，仅在需要安装/更新网络组件时显式运行：

```sh
cd /路径/network
sudo python3 install.py install
```

安装先保存现有目标文件及权限、所有者、相关服务状态，再原子替换文件、启用隧道和定时器、异步校时。原来运行的 Agent 会重启加载代理；原来停止的 Agent 保持停止。安装器不启动飞行任务，也不重启 PSDK。失败会尝试自动恢复。

手动验收既有链路：

```sh
systemctl is-active uav-code12-proxy-tunnel.service uav-code12-time-sync.timer
sudo /usr/local/bin/uav-api-agent-proxy-check
/usr/local/bin/uav-api-network-check --url https://api.deepseek.com --json
sudo /usr/local/bin/uav-api-network-connect
sudo /usr/local/bin/uav-api-network-connect --status
```

API 联网 helper 的端到端预算为 25 秒，单次子命令最多 6 秒；私有状态写入 `/run/uav-api-network`。并发请求不会覆盖活动状态，路径和锁不接受符号链接。网络检查不读取响应正文或跟随重定向；不要把凭据放进 URL。

手动程序复用代理：

```sh
/usr/local/bin/uav-api-with-proxy python3 /路径/应用.py
```

## 回滚

使用安装器打印的真实备份 ID：

```sh
sudo python3 install.py rollback 20261102T000000Z-01234567
```

回滚只恢复该次包管理的文件与服务状态，且拒绝覆盖后来修改/删除的目标。系统时间不会回拨。新版清单为版本 2，管理十个目标；原九目标版本 1 的备份仍可恢复，且不触碰后来安装的 API 联网 helper。回滚不需要上游网络或 SSH 密钥，但需要完好的本地备份和 systemd 权限。

## 验证范围

34 项本地测试通过临时目录和 mock systemd/socket 覆盖网络检查、并发状态、超时、安装失败回滚、旧备份兼容及不启动原先停止的 Agent。Python 与 shell 语法、`install.py plan` 已检查。测试不运行网络请求、真实模型或飞控；本次 GitHub 同步不构成新的设备部署或现场链路验收。
