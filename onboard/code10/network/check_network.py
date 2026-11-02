#!/usr/bin/env python3
"""Check the code10 -> code12 HTTP proxy path without credentials or inference."""
from __future__ import annotations

import argparse
import datetime as dt
import http.client
import json
import socket
import ssl
import sys
from dataclasses import dataclass
from urllib.parse import urlsplit


@dataclass(frozen=True)
class Config:
    host: str
    port: int
    proxy_host: str
    proxy_port: int
    target_host: str
    target_port: int
    request_target: str
    timeout: float


def authority(host: str, port: int) -> str:
    return f"[{host}]:{port}" if ":" in host else f"{host}:{port}"


def valid_host(host: str | None) -> str:
    if not host or any(ord(c) <= 32 or ord(c) == 127 for c in host):
        raise ValueError("主机地址为空或含控制字符")
    return host.encode("idna").decode("ascii")


def configuration(args: argparse.Namespace) -> Config:
    target = urlsplit(args.url)
    proxy = urlsplit(args.proxy)
    if target.scheme != "https" or target.username is not None or target.password is not None or target.fragment:
        raise ValueError("--url 必须是 HTTPS 地址，不能带用户名、密码或片段")
    if proxy.scheme != "http" or proxy.username is not None or proxy.password is not None or proxy.query or proxy.fragment or proxy.path not in ("", "/"):
        raise ValueError("--proxy 必须是无认证信息、无路径的 HTTP 代理地址")
    if any(ord(c) <= 32 or ord(c) == 127 for c in args.url + args.proxy):
        raise ValueError("地址不能含控制字符")
    if not 1 <= args.port <= 65535 or not 0 < args.timeout <= 120:
        raise ValueError("端口必须为 1–65535，超时必须大于 0 且不超过 120 秒")
    # Accessing .port also rejects invalid and out-of-range URL ports.
    proxy_port = proxy.port if proxy.port is not None else 80
    target_port = target.port if target.port is not None else 443
    if proxy_port == 0 or target_port == 0:
        raise ValueError("URL 端口不能为 0")
    path = target.path or "/"
    if target.query:
        path += "?" + target.query
    try:
        path.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError("URL 路径须使用百分号编码") from exc
    return Config(valid_host(args.host), args.port,
                  valid_host(proxy.hostname), proxy_port,
                  valid_host(target.hostname), target_port,
                  path, args.timeout)


def failure_description(exc: BaseException) -> str:
    # Never print response bodies, credential-bearing URLs or raw exception text.
    if isinstance(exc, ssl.SSLCertVerificationError):
        return "TLS 证书验证失败；检查系统时间、可信 CA 和目标证书"
    if isinstance(exc, ssl.SSLError):
        return "TLS 握手失败"
    if isinstance(exc, (TimeoutError, socket.timeout)):
        return "连接或读取超时"
    if isinstance(exc, ConnectionRefusedError):
        return "连接被拒绝"
    if isinstance(exc, socket.gaierror):
        return "主机地址解析失败"
    if isinstance(exc, http.client.HTTPException):
        return "HTTP 响应格式异常"
    return "连接失败（" + type(exc).__name__ + "）"


def api_status_description(status: int) -> str:
    if status == 401:
        return "HTTPS 网络可达；未鉴权，尚未验证 API 密钥、模型权限或额度"
    if status == 402:
        return "HTTPS 网络可达；服务报告付费/账户状态问题，需单独确认账户"
    if status == 403:
        return "HTTPS 传输可达；请求被拒绝，不能据此认定 API 可调用"
    if status == 404:
        return "HTTPS 传输可达；此路径不存在，不能据此认定 API 可调用"
    if 300 <= status < 400:
        return "HTTPS 传输可达；收到重定向（未跟随），尚未验证 API 调用"
    if 500 <= status:
        return "HTTPS 传输可达；服务端返回错误，尚未验证 API 调用"
    return "HTTPS 传输可达；本检查未携带 API 密钥，未验证模型权限或额度"


def check(config: Config) -> dict:
    result: dict = {
        "network_reachable": False,
        "api_authenticated": False,
        "model_call_tested": False,
        "target_origin": "https://" + authority(config.target_host, config.target_port),
        "stages": [],
        "note": "退出成功仅表示网络传输可达；不代表 API 鉴权、模型权限或额度可用。",
    }

    def stage(name: str, ok: bool, detail: str, **extra: object) -> None:
        result["stages"].append({"stage": name, "ok": ok, "detail": detail, **extra})

    try:
        with socket.create_connection((config.host, config.port), config.timeout):
            pass
        stage("code12_tcp", True, authority(config.host, config.port) + " 可连接（对应遥控器按钮的首跳测试）")
    except (OSError, ValueError) as exc:
        stage("code12_tcp", False, failure_description(exc))
        return result

    raw_socket = None
    tls_socket = None
    response = None
    try:
        try:
            raw_socket = socket.create_connection((config.proxy_host, config.proxy_port), config.timeout)
            stage("proxy_tcp", True, authority(config.proxy_host, config.proxy_port) + " 可连接")
        except (OSError, ValueError) as exc:
            stage("proxy_tcp", False, failure_description(exc))
            return result

        try:
            connect_target = authority(config.target_host, config.target_port)
            raw_socket.sendall(("CONNECT " + connect_target + " HTTP/1.1\r\nHost: " + connect_target + "\r\n\r\n").encode("ascii"))
            response = http.client.HTTPResponse(raw_socket, method="CONNECT")
            response.begin()
            status = response.status
            response.close()
            response = None
            if status != 200:
                stage("http_connect", False, "代理 CONNECT 未成功", http_status=status)
                return result
            stage("http_connect", True, "代理已建立到目标 HTTPS 端口的隧道", http_status=status)
        except (OSError, http.client.HTTPException, ValueError) as exc:
            stage("http_connect", False, failure_description(exc))
            return result

        if dt.datetime.now(dt.timezone.utc).year < 2024:
            stage("tls", False, "通信盒时间无效；请先等待 uav-code12-time-sync.service 成功校时")
            return result
        try:
            context = ssl.create_default_context()
            tls_socket = context.wrap_socket(raw_socket, server_hostname=config.target_host)
            raw_socket = None
            stage("tls", True, "TLS 握手成功，已验证证书链和主机名")
        except (OSError, ValueError) as exc:
            stage("tls", False, failure_description(exc))
            return result

        try:
            host_header = config.target_host if config.target_port == 443 else authority(config.target_host, config.target_port)
            if ":" in config.target_host and config.target_port == 443:
                host_header = "[" + config.target_host + "]"
            request = ("GET " + config.request_target + " HTTP/1.1\r\nHost: " + host_header
                       + "\r\nUser-Agent: uav-api-network-check/1\r\nConnection: close\r\n\r\n")
            tls_socket.sendall(request.encode("ascii"))
            response = http.client.HTTPResponse(tls_socket, method="GET")
            response.begin()
            status = response.status
            stage("api_http", True, api_status_description(status), http_status=status)
            result["network_reachable"] = True
        except (OSError, http.client.HTTPException, ValueError) as exc:
            stage("api_http", False, failure_description(exc))
        return result
    finally:
        if response is not None:
            response.close()
        if tls_socket is not None:
            tls_socket.close()
        if raw_socket is not None:
            raw_socket.close()


def parser() -> argparse.ArgumentParser:
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--url", default="https://api.deepseek.com", help="无 API 密钥的 HTTPS GET 验收地址")
    cli.add_argument("--proxy", default="http://127.0.0.1:18080")
    cli.add_argument("--host", default="192.168.168.222", help="对应按钮的 code12 探测主机")
    cli.add_argument("--port", type=int, default=22)
    cli.add_argument("--timeout", type=float, default=8.0, help="各次网络操作的超时秒数")
    cli.add_argument("--json", action="store_true", help="输出不含密钥/响应正文的 JSON")
    return cli


def main(argv: list[str] | None = None) -> int:
    cli = parser()
    args = cli.parse_args(argv)
    try:
        config = configuration(args)
    except (ValueError, UnicodeError) as exc:
        # Error messages here are ours, except urlsplit/.port messages. Do not echo supplied URLs.
        cli.error("参数格式错误；--url 需为 HTTPS、--proxy 需为无认证 HTTP 地址，端口/超时须有效")
    report = check(config)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        for item in report["stages"]:
            status = "通过" if item["ok"] else "失败"
            code = " HTTP=" + str(item["http_status"]) if "http_status" in item else ""
            print("[" + status + "] " + item["stage"] + code + "：" + item["detail"])
        print(report["note"])
    return 0 if report["network_reachable"] else 1


if __name__ == "__main__":
    sys.exit(main())
