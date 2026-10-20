import argparse,json,signal,shutil,sys
from .config import Config

def main():
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');a=p.parse_args();c=Config();c.initialize()
    if a.check:
        print(json.dumps({'api_key_configured':bool(c.key and c.key!='Your API Key'),'endpoint':c.base_url,'model':c.model,'flight_enabled':c.flight_enabled,'whisper_cli_exists':c.whisper.is_file(),'whisper_model_exists':c.whisper_model.is_file(),'liveview_metadata_exists':c.metadata.is_file(),'liveview_frame_dir_exists':c.frame_dir.is_dir(),'psdk_telemetry_exists':(c.state/'ipc/telemetry.json').is_file()},ensure_ascii=False,indent=2));return
    from .runtime import Runtime
    runtime=Runtime(c)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:runtime.stop.set())
    runtime.run()
if __name__=='__main__':main()
