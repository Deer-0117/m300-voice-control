#!/usr/bin/env python3
"""Explicitly run only the new executable's no-SDK --build-info branch."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
def main():
    try:
        info=json.loads((ROOT/'migration/last-host-build.json').read_text())
        binary=Path(info['binary']).resolve(strict=True)
        allowed=Path('/home/uav/m300-host-work').resolve(strict=True)
        binary.relative_to(allowed)
        if binary.name!='m300_voice_psdk_host' or not info.get('ground_only'):
            raise ValueError('Unexpected host target')
        if hashlib.sha256(binary.read_bytes()).hexdigest()!=info['binary_sha256']:
            raise ValueError('Host binary checksum changed')
        result=subprocess.run([str(binary),'--build-info'],capture_output=True,text=True,timeout=15,check=True)
        payload=json.loads(result.stdout)
        if payload!={'host':'m300_voice_psdk_host','ground_only':True,'sdk_initialized':False,'flight_enabled':False}:
            raise ValueError('Unexpected build-info response')
        print(json.dumps(payload,ensure_ascii=False))
        return 0
    except (OSError,KeyError,ValueError,subprocess.SubprocessError):
        print('Build-info check failed. Do not start the real host; report the build/check output.',file=sys.stderr)
        return 1

if __name__=='__main__':raise SystemExit(main())
