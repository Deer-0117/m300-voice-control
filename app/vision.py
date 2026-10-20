"""Read the existing Jetson Liveview publisher; never open its H.264 FIFO."""
from __future__ import annotations
from collections import deque
from pathlib import Path
import io, threading
from PIL import Image
from .clock import clock_id, frame_timestamp, split_record
from .io import now_ms

class LiveCamera:
    def __init__(self,cfg):
        self.cfg=cfg;self.frames=deque(maxlen=60);self.lock=threading.Lock();self.stop=threading.Event();self.thread=None;self.error=None
        self.last_id=0

    def read_latest(self):
        c=self.cfg
        try:
            parts=split_record(c.metadata.read_text(),4)
            if parts is None:return None
            ident,stamp,encode=int(parts[0]),int(parts[1]),int(parts[2])
            if ident<=self.last_id or encode<0 or not 0<=now_ms()-stamp<=c.frame_max_age_ms:return None
            file=Path(parts[3])
            file=(file if file.is_absolute() else c.frame_dir/file).resolve()
            if file.parent!=c.frame_dir.resolve() or file.suffix.lower() not in ('.jpg','.bmp'):return None
            # Do not trust mtime or guess whether wall time resembles monotonic time.
            if frame_timestamp(file)!=stamp:return None
            data=file.read_bytes()
            if len(data)>8*1024*1024:return None
            with Image.open(io.BytesIO(data)) as im:
                im.load()
                if im.width<32 or im.height<32:return None
                if file.suffix.lower()=='.bmp':
                    out=io.BytesIO();im.convert('RGB').save(out,format='JPEG',quality=85);data=out.getvalue()
            self.last_id=ident
            return {'frame_id':ident,'time_ms':stamp,'clock_id':clock_id(),'encode_ms':encode,'jpeg':data}
        except (OSError,ValueError,TypeError):return None

    def start(self):
        self.thread=threading.Thread(target=self._read,daemon=True);self.thread.start()

    def _read(self):
        while not self.stop.wait(.02):
            try:
                frame=self.read_latest()
                if frame:
                    with self.lock:self.frames.append(frame)
            except Exception as exc:self.error=type(exc).__name__

    def sample(self):
        with self.lock:frames=list(self.frames)
        if len(frames)<3 or not 0<=now_ms()-frames[-1]['time_ms']<=self.cfg.frame_max_age_ms:return None
        recent=[f for f in frames if frames[-1]['time_ms']-f['time_ms']<=1000]
        if len(recent)<3:return None
        return [recent[0],recent[len(recent)//2],recent[-1]]

    def close(self):
        self.stop.set()
        if self.thread:self.thread.join(timeout=2)
