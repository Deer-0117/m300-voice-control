#!/usr/bin/env python3
"""Write a private external env file; no network, services or flight changes."""
import ast
import getpass
import os
from pathlib import Path
import platform
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
DEST=Path('/home/uav/.config/m300-voice-control/runtime.env')

def current_model():
    source=Path('/home/uav/offline_balloon_strike_test/uav_voice_command.py')
    try:
        for node in ast.walk(ast.parse(source.read_text(encoding='utf-8'))):
            if not isinstance(node,ast.List):continue
            try:values=ast.literal_eval(node)
            except (ValueError,TypeError):continue
            if '--model' in values:
                index=values.index('--model')+1
                if index<len(values) and isinstance(values[index],str) and Path(values[index]).is_file():return values[index]
    except (OSError,SyntaxError):pass
    small=Path('/opt/whisper.cpp/models/ggml-small.bin')
    return str(small if small.is_file() else Path('/opt/whisper.cpp/models/ggml-base.bin'))

def main():
    if sys.platform!='linux' or platform.machine()!='aarch64' or sys.version_info[:2]!=(3,10):
        raise SystemExit('Run on Jetson ARM64 with Python 3.10.')
    if DEST.exists():raise SystemExit('Private config already exists; it was not overwritten. Edit it locally if needed.')
    cli='/opt/whisper.cpp/build/bin/whisper-cli';model=current_model()
    if not os.access(cli,os.X_OK) or not Path(model).is_file():raise SystemExit('Existing Whisper CLI/model not available.')
    key=getpass.getpass('New DeepSeek API Key (hidden; do not reuse the exposed key): ').strip()
    if not key or any(c.isspace() for c in key):raise SystemExit('Invalid key; no configuration written.')
    values={'OPENAI_API_KEY':key,'OPENAI_BASE_URL':'https://api.deepseek.com',
            'OPENAI_CHAT_MODEL_ID':'deepseek-flash','UAV_API_TIMEOUT_S':'30',
            'UAV_MAX_VISUAL_DECISIONS':'100','UAV_ENABLE_FLIGHT':'0',
            'UAV_STATE_DIR':str(ROOT/'state'),'UAV_WHISPER_CLI':cli,'UAV_WHISPER_MODEL':model,
            'UAV_LIVEVIEW_METADATA':'inputs/liveview_latest.tsv','UAV_LIVEVIEW_FRAME_DIR':'inputs/liveview_frames',
            'UAV_CAMERA_DESCRIPTION':'Forward-facing level camera. Verify real mounting, gimbal and fixed zoom before movement.'}
    DEST.parent.mkdir(parents=True,exist_ok=True);os.chmod(DEST.parent,0o700)
    temporary=None
    try:
        with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=str(DEST.parent),delete=False) as stream:
            temporary=Path(stream.name)
            for name,value in values.items():stream.write(name+'='+value+'\n')
        os.chmod(temporary,0o600)
        os.link(temporary,DEST)
    finally:
        if temporary is not None:temporary.unlink(missing_ok=True)
    print('Saved external private config: '+str(DEST))
    print('UAV_ENABLE_FLIGHT=0. No API call or service changes. No key was written into the source package.')

if __name__=='__main__':main()
