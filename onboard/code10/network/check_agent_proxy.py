#!/usr/bin/env python3
"""Read only the running Agent's whitelisted proxy fields, never API tokens."""
import os
from pathlib import Path
import subprocess
import sys

EXPECTED_PROXY = "http://127.0.0.1:18080"
PROXY_NAMES = {"HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "http_proxy", "https_proxy", "all_proxy"}
NO_PROXY_NAMES = {"NO_PROXY", "no_proxy"}
REQUIRED_LOCAL = {"localhost", "127.0.0.1", "::1", "192.168.168.155", "192.168.168.222", "192.168.55.1"}


def main():
    if sys.platform != "linux" or os.geteuid() != 0:
        print("请在通信盒上用 sudo 运行；只读取 Agent 的代理变量。", file=sys.stderr)
        return 2
    proc = subprocess.run(["systemctl", "show", "uav-microsoft-agent.service", "--property=MainPID", "--value"], capture_output=True, text=True, timeout=10)
    if proc.returncode or not proc.stdout.strip().isdigit() or int(proc.stdout.strip()) <= 0:
        print("Agent 未运行，无法检查进程环境。", file=sys.stderr)
        return 1
    values = {}
    for item in (Path("/proc") / proc.stdout.strip() / "environ").read_bytes().split(b"\0"):
        name, separator, value = item.partition(b"=")
        key = name.decode("ascii", errors="ignore")
        if separator and key in PROXY_NAMES | NO_PROXY_NAMES:
            values[key] = value.decode("utf-8", errors="replace")
    ok = True
    for name in sorted(PROXY_NAMES):
        match = values.get(name) == EXPECTED_PROXY
        print(f"{name}: {'符合回环代理配置' if match else '缺失或被其他配置覆盖'}")
        ok &= match
    for name in sorted(NO_PROXY_NAMES):
        match = REQUIRED_LOCAL.issubset({part.strip() for part in values.get(name, "").split(",")})
        print(f"{name}: {'本地地址均绕过代理' if match else '本地绕过地址不完整'}")
        ok &= match
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, subprocess.TimeoutExpired):
        print("无法读取服务进程环境；请在通信盒用 sudo 重试。", file=sys.stderr)
        raise SystemExit(1)
