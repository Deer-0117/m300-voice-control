#!/usr/bin/env python3
"""Install/restore the Code10 API network layer; plan is safe on macOS."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import pwd
import re
import shutil
import stat
import subprocess
import sys
import uuid


PACKAGE = Path(__file__).resolve().parent
TARGETS = (
    ("systemd/uav-code12-proxy-tunnel.service", "/etc/systemd/system/uav-code12-proxy-tunnel.service", 0o644),
    ("systemd/uav-code12-time-sync.service", "/etc/systemd/system/uav-code12-time-sync.service", 0o644),
    ("systemd/uav-code12-time-sync.timer", "/etc/systemd/system/uav-code12-time-sync.timer", 0o644),
    ("systemd/zzzzzzzzzzzz-api-network.conf", "/etc/systemd/system/uav-microsoft-agent.service.d/zzzzzzzzzzzz-api-network.conf", 0o644),
    ("config/uav-code12-proxy.env", "/etc/uav-code12-proxy.env", 0o644),
    ("scripts/uav-code12-time-sync", "/usr/local/sbin/uav-code12-time-sync", 0o755),
    ("scripts/uav-api-with-proxy", "/usr/local/bin/uav-api-with-proxy", 0o755),
    ("check_network.py", "/usr/local/bin/uav-api-network-check", 0o755),
    ("check_agent_proxy.py", "/usr/local/bin/uav-api-agent-proxy-check", 0o755),
    ("scripts/uav-api-network-connect", "/usr/local/bin/uav-api-network-connect", 0o755),
)
LEGACY_TARGETS = {target for _, target, _ in TARGETS if target != "/usr/local/bin/uav-api-network-connect"}
TUNNEL = "uav-code12-proxy-tunnel.service"
TIMER = "uav-code12-time-sync.timer"
TIME_SERVICE = "uav-code12-time-sync.service"
AGENT = "uav-microsoft-agent.service"
UNITS = (TUNNEL, TIMER, TIME_SERVICE, AGENT)
BACKUP_BASE = "/var/lib/uav-api-network/backups"
SSH_FILES = (
    "/home/uav/.ssh/code10_code12_proxy_ed25519",
    "/home/uav/.ssh/known_hosts_code12_proxy",
    "/home/uav/.ssh/code10_code12_time_ed25519",
    "/home/uav/.ssh/known_hosts_code12_time",
)


class DeploymentError(RuntimeError):
    pass


def running(state: str) -> bool:
    return state in {"active", "activating", "reloading"}


class Deployment:
    def __init__(self, root: Path = Path("/"), runner=subprocess.run):
        # A separate root is used only by unit tests, never exposed as a CLI flag.
        self.root = root
        self.runner = runner

    def path(self, absolute: str) -> Path:
        return self.root / absolute.lstrip("/")

    def ctl(self, *args: str, check: bool = True):
        result = self.runner(["systemctl", *args], text=True, capture_output=True, timeout=40)
        if check and result.returncode:
            # systemctl diagnostics for these fixed service names do not include secrets.
            raise DeploymentError(f"systemctl {' '.join(args)}: {result.stderr.strip() or result.stdout.strip()}")
        return result

    def preflight(self, installing=True):
        if sys.platform != "linux" or os.geteuid() != 0:
            raise DeploymentError("安装/回滚只能在通信盒 Linux 上用 sudo 运行；本机可运行 plan。")
        if not shutil.which("systemctl"):
            raise DeploymentError("需要已安装的 systemd。")
        if not installing:
            return
        if not Path("/usr/bin/ssh").is_file():
            raise DeploymentError("需要已安装的 /usr/bin/ssh。")
        try:
            pwd.getpwnam("uav")
        except KeyError as exc:
            raise DeploymentError("通信盒必须已有 uav 用户。") from exc
        if self.ctl("cat", AGENT, check=False).returncode:
            raise DeploymentError("未发现既有 Agent 服务；本包只配置已部署的 Agent。")
        for name in SSH_FILES:
            if not self.path(name).is_file():
                raise DeploymentError(f"缺少既有受限 SSH 文件：{name}；本包不生成或覆盖密钥。")
        for source, target, _mode in TARGETS:
            if not (PACKAGE / source).is_file():
                raise DeploymentError(f"部署包不完整：{source}")
            p = self.path(target)
            if p.is_symlink() or (p.exists() and not p.is_file()):
                raise DeploymentError(f"目标不是普通文件，未修改：{target}")

    def state(self):
        return {
            unit: {
                "enabled": self.ctl("is-enabled", unit, check=False).stdout.strip(),
                "active": self.ctl("is-active", unit, check=False).stdout.strip(),
            }
            for unit in UNITS
        }

    def make_backup(self) -> Path:
        ident = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
        backup = self.path(BACKUP_BASE) / ident
        backup.mkdir(parents=True, mode=0o700)
        backup.chmod(0o700)
        (backup / "files").mkdir(mode=0o700)
        manifest = {"version": 2, "id": ident, "units": self.state(), "files": []}
        for source, target, mode in TARGETS:
            p = self.path(target)
            record = {"path": target, "existed": p.exists(), "installed_sha256": hashlib.sha256((PACKAGE / source).read_bytes()).hexdigest()}
            if p.exists():
                if p.is_symlink() or not p.is_file():
                    raise DeploymentError(f"目标不是普通文件：{target}")
                meta = p.stat()
                record.update(mode=stat.S_IMODE(meta.st_mode), uid=meta.st_uid, gid=meta.st_gid)
                saved = backup / "files" / target.lstrip("/")
                saved.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(p, saved)
                record["original_sha256"] = hashlib.sha256(saved.read_bytes()).hexdigest()
            manifest["files"].append(record)
        self.save_manifest(backup, manifest)
        return backup

    @staticmethod
    def save_manifest(backup: Path, manifest: dict):
        p = backup / "manifest.json"
        p.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        p.chmod(0o600)

    def write_targets(self):
        for source, target, mode in TARGETS:
            dest = self.path(target)
            dest.parent.mkdir(parents=True, exist_ok=True)
            # Atomic replacement keeps a partially copied unit out of service.
            temporary = dest.with_name(dest.name + ".uav-api-new-" + uuid.uuid4().hex[:8])
            try:
                temporary.write_bytes((PACKAGE / source).read_bytes())
                temporary.chmod(mode)
                if self.root == Path("/"):
                    os.chown(temporary, 0, 0)
                os.replace(temporary, dest)
            finally:
                temporary.unlink(missing_ok=True)

    def install(self):
        backup = self.make_backup()
        manifest = json.loads((backup / "manifest.json").read_text())
        print(f"已备份至：{backup}")
        try:
            # Do not race an already running previous clock-sync script.
            self.ctl("stop", TIME_SERVICE, check=False)
            self.write_targets()
            self.ctl("daemon-reload")
            self.ctl("enable", TUNNEL, TIMER)
            self.ctl("restart", TUNNEL)
            self.ctl("restart", TIMER)
            self.ctl("start", "--no-block", TIME_SERVICE)
            if running(manifest["units"][AGENT]["active"]):
                self.ctl("restart", AGENT)
        except Exception:
            print("安装失败，正在恢复本包修改的文件与服务状态。", file=sys.stderr)
            try:
                self.restore(backup, allow_partial=True)
            except Exception as rollback_error:
                print(f"自动回滚未完成：{rollback_error}；请保留备份目录。", file=sys.stderr)
            raise
        print("网络配置文件已安装；连通性、API 权限仍需按 README 验收。")
        print(f"回滚：sudo python3 install.py rollback {backup.name}")
        return backup

    def load_backup(self, backup: Path):
        if backup.resolve().parent != self.path(BACKUP_BASE).resolve():
            raise DeploymentError("只能回滚本包备份目录中的一个备份 ID。")
        manifest = json.loads((backup / "manifest.json").read_text(encoding="utf-8"))
        if manifest.get("version") not in (1, 2) or manifest.get("id") != backup.name:
            raise DeploymentError("备份清单版本或 ID 不匹配。")
        # The original package's v1 backups predate the API-connect helper.
        # Restore exactly their nine managed paths and leave any later helper.
        expected = LEGACY_TARGETS if manifest["version"] == 1 else {target for _, target, _ in TARGETS}
        actual = {record["path"] for record in manifest["files"]}
        if actual != expected or len(manifest["files"]) != len(expected):
            raise DeploymentError("备份目标清单不匹配，未执行回滚。")
        if set(manifest["units"]) != set(UNITS):
            raise DeploymentError("备份服务清单不匹配。")
        for record in manifest["files"]:
            if record["existed"]:
                saved = backup / "files" / record["path"].lstrip("/")
                if saved.is_symlink() or not saved.is_file():
                    raise DeploymentError(f"备份文件缺失：{record['path']}")
                if hashlib.sha256(saved.read_bytes()).hexdigest() != record["original_sha256"]:
                    raise DeploymentError(f"备份文件校验失败：{record['path']}")
        return manifest

    def restore(self, backup: Path, *, allow_partial=False):
        manifest = self.load_backup(backup)
        # Refuse to overwrite a later deployment or manually edited target.
        # Automatic rollback additionally allows files untouched by our failed install.
        for record in manifest["files"]:
            dest = self.path(record["path"])
            if dest.is_symlink() or (dest.exists() and not dest.is_file()):
                raise DeploymentError(f"回滚目标不再是普通文件：{record['path']}")
            if not dest.exists():
                if allow_partial and not record["existed"]:
                    continue
                raise DeploymentError(f"回滚目标已被移除，未覆盖后续状态：{record['path']}")
            allowed = {record["installed_sha256"]}
            if allow_partial and record["existed"]:
                allowed.add(record["original_sha256"])
            if hashlib.sha256(dest.read_bytes()).hexdigest() not in allowed:
                raise DeploymentError(f"回滚目标已被其他操作修改，未覆盖：{record['path']}")
        for unit in (TIMER, TIME_SERVICE, TUNNEL):
            self.ctl("stop", unit, check=False)
        self.ctl("disable", TUNNEL, TIMER, check=False)
        for record in manifest["files"]:
            dest = self.path(record["path"])
            if dest.is_symlink() or (dest.exists() and not dest.is_file()):
                raise DeploymentError(f"回滚目标不再是普通文件：{record['path']}")
            if record["existed"]:
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(backup / "files" / record["path"].lstrip("/"), dest)
                dest.chmod(record["mode"])
                if self.root == Path("/"):
                    os.chown(dest, record["uid"], record["gid"])
            else:
                dest.unlink(missing_ok=True)
        self.ctl("daemon-reload")
        for unit in (TUNNEL, TIMER):
            previous = manifest["units"][unit]
            if previous["enabled"] == "enabled":
                self.ctl("enable", unit)
            elif previous["enabled"] == "enabled-runtime":
                self.ctl("enable", "--runtime", unit)
            if running(previous["active"]):
                self.ctl("start", unit)
        if running(manifest["units"][TIME_SERVICE]["active"]):
            self.ctl("start", "--no-block", TIME_SERVICE)
        if running(manifest["units"][AGENT]["active"]):
            self.ctl("restart", AGENT)
        print(f"已恢复备份 {backup.name}；不会把系统时钟回拨到错误日期。")


def main(argv=None):
    parser = argparse.ArgumentParser(description="通信盒 API 网络部署；默认只展示计划")
    sub = parser.add_subparsers(dest="command")
    sub.add_parser("plan", help="本地查看目标文件，无副作用")
    sub.add_parser("install", help="在通信盒上安装，需要 sudo")
    rollback = sub.add_parser("rollback", help="在通信盒上恢复指定备份")
    rollback.add_argument("backup_id")
    args = parser.parse_args(argv)
    if args.command in (None, "plan"):
        print("本地部署计划：code10 → 192.168.168.222:22 → 回环 HTTP 代理 18080 → API")
        print("以下目标仅在通信盒执行 install 后写入：")
        for source, target, _mode in TARGETS:
            print(f"  {source} -> {target}")
        print("前置：code12 上游代理已配置；通信盒已有两把受限密钥和已核验 known_hosts。")
        print("不会更改网卡地址、默认路由、PSDK 或 API 密钥。")
        return 0
    deploy = Deployment()
    try:
        deploy.preflight(installing=args.command == "install")
        if args.command == "install":
            deploy.install()
        else:
            if not re.fullmatch(r"[0-9]{8}T[0-9]{6}Z-[0-9a-f]{8}", args.backup_id):
                raise DeploymentError("备份 ID 格式不正确。")
            deploy.restore(deploy.path(BACKUP_BASE) / args.backup_id)
        return 0
    except (OSError, ValueError, DeploymentError, subprocess.TimeoutExpired) as exc:
        print(f"未完成：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
