"""Bounded ground recording/pose capture; never starts a mission or commands motion."""
import argparse
import json
import math
import time
import uuid
from pathlib import Path
from balloon_clock import now_ms, clock_id

REQUEST = Path('/tmp/balloon_timing_capture.request')
RECORD = Path('/tmp/balloon_prediction_record.request')
ACK = Path('/tmp/balloon_prediction_record_ack.json')
POSE = Path('/home/uav/offline_balloon_strike_test/calibration_camera_pose.csv')
COMMIT = Path('/tmp/balloon_mission.commit')


def remove_owned(path, token):
    try:
        if path.read_text(encoding='ascii').split()[0] == token: path.unlink()
    except (FileNotFoundError,IndexError): pass


def read_ack(token, boot, started, current):
    """Only this capture's current-boot, causal recorder receipt is evidence."""
    try:
        ack=json.loads(ACK.read_text(encoding='utf-8'))
        if not isinstance(ack,dict): return None,'invalid receipt'
        if ack.get('token') != token: return None,'no matching recorder receipt'
        if ack.get('clock_id') != boot: return None,'recorder boot mismatch'
        stamp=ack.get('ts_ms')
        if type(stamp) is not int or not started <= stamp <= current:
            return None,'invalid recorder timestamp'
        if current-stamp > 1500: return None,'recorder receipt stale'
        return ack,''
    except (OSError,ValueError): return None,'recorder receipt unavailable'


def pose_status(token, boot, started, current):
    try:
        with POSE.open('rb') as f:
            f.seek(0,2); f.seek(max(0,f.tell()-4096)); tail=f.read().decode('ascii',errors='replace')
        matching=[line.split(',') for line in tail.splitlines()
                  if len(line.split(',')) == 9 and line.split(',')[3] == boot
                  and line.split(',')[-1] == token]
        if not matching: return False,'no matching pose rows'
        latest=matching[-1]
        stamp=int(latest[0])
        if not started <= stamp <= current: return False,'invalid pose timestamp'
        if current-stamp > 1000: return False,'pose rows stale'
        yaw,pitch=float(latest[1]),float(latest[2])
        if not (math.isfinite(yaw) and math.isfinite(pitch) and abs(yaw)<=360 and abs(pitch)<=180):
            return False,'invalid camera angles'
        begin,end=int(latest[4]),int(latest[5])
        if not started <= begin <= end == stamp:
            return False,'invalid pose read interval'
        raw=tuple(int(v) for v in latest[6:8])
        if any(v<0 or v>0xffffffff for v in raw): return False,'invalid SDK timestamp fields'
        if len(matching)<2: return False,'waiting for SDK sample progress'
        previous=matching[-2]
        if int(previous[0])>=stamp: return False,'pose timestamps not advancing'
        # A newly written host timestamp must not make an old SDK buffer look
        # fresh. Compare raw identity only; their clock mapping is still unknown.
        if raw==tuple(int(v) for v in previous[6:8]):
            return False,'SDK sample timestamp not advancing'
        return True,'ready'
    except (OSError,ValueError): return False,'pose log unavailable'


def capture(seconds):
    if not 15 <= seconds <= 60: raise ValueError('duration must be 15..60 seconds')
    if COMMIT.exists():
        raise ValueError('mission is committed; stop all tests before ground capture')
    token='timing-'+uuid.uuid4().hex
    started=now_ms(); deadline=started+int(seconds*1000); boot=clock_id()
    try:
        # Exclusive files: never overwrite a mission's recording or a prior capture.
        with REQUEST.open('x',encoding='ascii') as f: f.write(f'{token} {deadline}\n')
        with RECORD.open('x',encoding='ascii') as f: f.write(token+'\n')
        print('Recording only. Keep aircraft fixed; manually sweep gimbal left/right with pauses.',flush=True)
        ready=False; directory=None; last_ready=started
        last_frames=None; last_ack_stamp=None; last_progress=None; advancing=False
        last_diagnostic=None
        while now_ms() < deadline:
            try:
                own_requests=all(p.read_text(encoding='ascii').split()[0] == token for p in (REQUEST,RECORD))
            except (OSError,ValueError,IndexError): own_requests=False
            current=now_ms()
            ack,video_reason=read_ack(token,boot,started,current)
            if ack and ack.get('status') == 'failed':
                raise RuntimeError('video recorder failed: '+str(ack.get('error','unknown error')))
            if not own_requests: raise ValueError('capture stopped externally or request replaced')
            video_ready=False
            if ack:
                frames=ack.get('frames'); receipt_dir=ack.get('directory')
                if ack.get('status') != 'recording': video_reason='recorder is not recording'
                elif type(frames) is not int or frames < 1: video_reason='no recorded frames'
                elif not isinstance(receipt_dir,str) or not receipt_dir: video_reason='recorder directory missing'
                else:
                    if directory is not None and directory != receipt_dir:
                        raise RuntimeError('video recorder directory changed during capture')
                    directory=receipt_dir
                    if last_frames is not None and (frames < last_frames or ack['ts_ms'] < last_ack_stamp):
                        raise RuntimeError('video recorder frame count/timestamp reversed')
                    if last_frames is None:
                        last_frames=frames; last_ack_stamp=ack['ts_ms']; last_progress=current
                    elif frames > last_frames and ack['ts_ms'] > last_ack_stamp:
                        advancing=True; last_frames=frames; last_ack_stamp=ack['ts_ms']; last_progress=current
                    video_ready=advancing and current-last_progress <= 1500
                    video_reason='ready (frames advancing)' if video_ready else 'waiting for frame-count progress'
            pose_ready,pose_reason=pose_status(token,boot,started,current)
            diagnostic=(video_reason,pose_reason)
            if diagnostic != last_diagnostic:
                print(f'CAPTURE_STATUS video={video_reason}; pose={pose_reason}',flush=True)
                last_diagnostic=diagnostic
            if video_ready and pose_ready:
                if not ready: print('VIDEO_AND_POSE_READY: '+str(directory),flush=True)
                ready=True; last_ready=current
            if ready and now_ms()-last_ready > 1500:
                raise RuntimeError(f'capture stopped updating: video={video_reason}; pose={pose_reason}')
            if not ready and now_ms()-started > 8000:
                raise ValueError(f'capture not ready: video={video_reason}; pose={pose_reason}; '
                                 'check patched PSDK liveview/pose startup and decoder')
            time.sleep(.1)
    finally:
        # The writer can close immediately when RECORD is removed. Set the
        # boundary before cleanup so that receipt is not misclassified as old.
        stop_started=now_ms()
        try: remove_owned(REQUEST,token)
        finally: remove_owned(RECORD,token)
    stop_deadline=stop_started+10000
    while now_ms() < stop_deadline:
        ack,_=read_ack(token,boot,stop_started,now_ms())
        if ack and ack.get('status') == 'failed':
            raise RuntimeError('video recorder finalization failed: '+str(ack.get('error','unknown error')))
        if ack and ack.get('status') == 'stopped' and ack.get('directory') == directory:
            print('CAPTURE_SAVED: '+str(directory)); print('POSE_LOG: '+str(POSE));return
        time.sleep(.1)
    raise ValueError('recording finalization not confirmed; preserve files and inspect decoder log')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=int,default=30)
    parser.add_argument('--stationary-aircraft',action='store_true',required=True,
                        help='confirm aircraft stationary on ground and chase task stopped')
    args=parser.parse_args()
    capture(args.seconds)
