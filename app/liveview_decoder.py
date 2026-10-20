"""Optional dedicated producer using the supplied Jetson publisher's TSV protocol.
Run only if the existing decoder service is stopped. One reader per H.264 FIFO.
"""
from __future__ import annotations
import argparse, os, stat, time
from collections import deque
from pathlib import Path
from .clock import clock_id, now_ms
from .config import path

def atomic(file,data):
    temporary=file.with_name(file.name+'.tmp');temporary.write_bytes(data);temporary.replace(file)

def publish(frame, ident, directory, metadata):
    import cv2
    stamp=now_ms();start=time.perf_counter_ns();im=frame
    h,w=frame.shape[:2]
    if max(h,w)>960:
        scale=960/max(h,w);im=cv2.resize(frame,(round(w*scale),round(h*scale)))
    ok,buf=cv2.imencode('.jpg',im,[cv2.IMWRITE_JPEG_QUALITY,85])
    if not ok:raise RuntimeError('JPEG encode failed')
    file=directory/f'frame_{ident:012d}.jpg';atomic(file,buf.tobytes())
    atomic(file.with_suffix('.jpg.tsv'),f'{stamp}\t{file.name}\t{clock_id()}\n'.encode())
    encoded=(time.perf_counter_ns()-start)//1_000_000
    atomic(metadata,f'{ident}\t{stamp}\t{encoded}\t{file}\t{clock_id()}\n'.encode())
    return file

def main():
    p=argparse.ArgumentParser();p.add_argument('--fifo',default='state/ipc/liveview.h264');p.add_argument('--frame-dir',default='state/liveview/frames');p.add_argument('--metadata',default='state/liveview/latest.tsv');a=p.parse_args()
    fifo,frames,metadata=path(a.fifo),path(a.frame_dir),path(a.metadata)
    frames.mkdir(parents=True,exist_ok=True);metadata.parent.mkdir(parents=True,exist_ok=True);fifo.parent.mkdir(parents=True,exist_ok=True)
    try:os.mkfifo(fifo,0o600)
    except FileExistsError:
        if not stat.S_ISFIFO(fifo.stat().st_mode):raise ValueError('input is not a FIFO')
    import cv2
    # Same GStreamer software decoder as the supplied M300/H20 installation.
    pipeline=f'filesrc location="{fifo}" do-timestamp=true ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false'
    if cv2.getBuildInformation().find('GStreamer:                   YES')<0 and 'GStreamer' not in cv2.getBuildInformation():raise RuntimeError('GStreamer-enabled OpenCV required')
    retained=deque();ident=time.monotonic_ns()
    while True:
        cap=cv2.VideoCapture(pipeline,cv2.CAP_GSTREAMER)
        try:
            last=0
            while cap.isOpened():
                ok,frame=cap.read()
                if not ok:break
                if now_ms()-last<80:continue
                ident+=1;last=now_ms();retained.append(publish(frame,ident,frames,metadata))
                while len(retained)>48:
                    old=retained.popleft();old.unlink(missing_ok=True);old.with_suffix('.jpg.tsv').unlink(missing_ok=True)
        finally:cap.release()
        time.sleep(.5)
if __name__=='__main__':main()
