#!/usr/bin/env python3
"""One text API request only; does not import the runtime or aircraft bridge."""
import json
import os
from pathlib import Path
import platform
import sys
ROOT=Path(__file__).resolve().parents[1]
if sys.platform!='linux' or platform.machine()!='aarch64':raise SystemExit('Run this on Jetson.')
sys.path.insert(0,str(ROOT))
os.environ['M300_ENV_FILE']=str(Path('/home/uav/.config/m300-voice-control/runtime.env'))
os.environ['UAV_ENABLE_FLIGHT']='0'
from app.config import Config
from app.mission import Mission
from app.model import ModelClient

def main():
    cfg=Config();cfg.initialize()
    try:
        mission=Mission();revision=mission.interrupt()
        result=ModelClient(cfg).task('先撞击红色气球，再撞击绿色气球',mission)
        mission.propose(result['answer'],revision)
        print(json.dumps({'ok':True,'flight_enabled':False,'candidate':result['answer'],
                          'summary':mission.summary(),'elapsed_ms':result['received_ms']-result['sent_ms']},ensure_ascii=False))
        return 0
    except (RuntimeError,ValueError,OSError) as exc:
        print(json.dumps({'ok':False,'flight_enabled':False,'error_type':type(exc).__name__,
                          'error':str(exc) if isinstance(exc,RuntimeError) else '检查本地配置或候选返回格式'},ensure_ascii=False))
        return 1

if __name__=='__main__':raise SystemExit(main())
