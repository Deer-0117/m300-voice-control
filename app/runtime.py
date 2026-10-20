from __future__ import annotations
import copy, json, math, signal, threading, time, uuid
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from .config import Config
from .io import atomic_json,read_json,now_ms,event_log
from .clock import clock_id
from .mission import Mission
from .model import ModelClient
from .asr import Recognizer
from .vision import LiveCamera
from .control import ZERO,validate_motion,healthy,stable

class Runtime:
    def __init__(self,cfg):
        self.cfg=cfg;cfg.initialize();self.mission=Mission();self.client=ModelClient(cfg);self.asr=Recognizer(cfg);self.camera=LiveCamera(cfg)
        self.session=uuid.uuid4().hex;self.stop=threading.Event();self.pool=ThreadPoolExecutor(max_workers=2)
        self.task_future=None;self.motion_future=None;self.voice_revision=None;self.segment=None;self.motion_id=0;self.stable_since=None
        self.recent_actions=deque(maxlen=6);self.green_history=deque(maxlen=3);self.last_summary=None;self.seen=set();self.boot=None
        self.visual_calls=0;self.integrated_hover=0.;self.query_yaw=0.;self.last_tick=now_ms();self.last_candidate=None;self.last_error=''

    def wait_for_stability(self, revision):
        deadline=time.monotonic()+10;since=None
        while not self.stop.wait(.05):
            if self.mission.revision!=revision:raise RuntimeError('task superseded')
            try:t=read_json(self.cfg.state/'ipc/telemetry.json')
            except (OSError,ValueError):t={}
            valid=t.get('clock_id')==clock_id() and t.get('bridge_boot')==self.boot and t.get('kinematics_fresh',t.get('fresh')) is True and 0<=now_ms()-t.get('time_ms',0)<=250
            if valid and stable(t):
                if since is None:since=time.monotonic()
                if time.monotonic()-since>=.5:return
            else:since=None
            if time.monotonic()>deadline:raise RuntimeError('cannot establish stable hover')
        raise RuntimeError('runtime stopped')

    def clear_motion(self):
        if self.segment:
            row=self.segment
            self.recent_actions.append({'command':row['velocity'],'executed_duration_s':max(0,(min(now_ms(),row['deadline'])-row['start'])/1000),'actual_mean_velocity_body_mps':[x/max(1,row['samples']) for x in row['actual_sum']],'call_id':row['call_id']})
        self.segment=None;self.stable_since=None;self.motion_id+=1
        if self.motion_future:self.motion_future.cancel();self.motion_future=None

    def publish(self,t):
        m=self.mission;now=now_ms();frames=self.camera.sample();v=dict(ZERO)
        if self.segment and m.status=='active' and healthy(t,self.cfg,now) and frames:
            if now<self.segment['deadline']:
                v=self.segment['velocity'];self.segment['samples']+=1
                yaw=math.radians(t['yaw_deg']);n,e,z=t['velocity_neu']
                actual=[n*math.cos(yaw)+e*math.sin(yaw),-n*math.sin(yaw)+e*math.cos(yaw),z]
                self.segment['actual_sum']=[x+y for x,y in zip(self.segment['actual_sum'],actual)]
            else:self.clear_motion()
        elif self.segment:self.clear_motion()
        atomic_json(self.cfg.state/'ipc/command.json',{'clock_id':clock_id(),'session':self.session,'bridge_boot':self.boot or '', 'sent_ms':now,'deadline_ms':self.segment['deadline'] if self.segment else now,'revision':m.revision,'target_index':len(m.completed),'plan_hash':m.approved_hash,'nonce':m.approved_nonce,'motion_id':self.motion_id,'enabled':self.cfg.flight_enabled and m.status=='active','velocity':v})
        p=m.pending
        # Repeated status publication is a liveness signal to the RC bridge.
        atomic_json(self.cfg.state/'ipc/status.json',{'clock_id':clock_id(),'session':self.session,'time_ms':now,'revision':m.revision,'target_index':len(m.completed),'flight_enabled':self.cfg.flight_enabled,'status':m.status,'summary':m.summary(),'plan_hash':p['plan_hash'] if p else m.approved_hash,'nonce':p['nonce'] if p else m.approved_nonce})

    def events(self):
        for file in sorted((self.cfg.state/'ipc/events').glob('*.json')):
            try:event=read_json(file)
            except (OSError,ValueError):continue
            file.unlink(missing_ok=True)
            if event.get('bridge_boot')!=self.boot or event.get('clock_id')!=clock_id() or not 0<=now_ms()-event.get('time_ms',0)<=3000:continue
            ident=event.get('id')
            if not ident or ident in self.seen:continue
            self.seen.add(ident)
            if len(self.seen)>4096:self.seen={ident}
            kind=event.get('kind');event_log(self.cfg.state,'rc_event',kind=kind,revision=event.get('revision'))
            if kind=='voice_start':
                self.clear_motion();rev=self.mission.interrupt();self.voice_revision=(event['audio_id'],rev)
                if self.task_future:self.task_future[0].cancel()
                self.task_future=None
            elif kind=='voice_finish' and self.voice_revision and event['audio_id']==self.voice_revision[0]:
                rev=self.voice_revision[1];snapshot=copy.deepcopy(self.mission)
                def work(ev=event,ms=snapshot):
                    text=self.asr.recognize_event(ev)
                    self.wait_for_stability(rev)
                    result=self.client.task(text,ms)
                    return text,result
                self.task_future=(self.pool.submit(work),rev)
                self.voice_revision=None
            elif kind=='confirm':
                if event.get('session')==self.session and self.mission.confirm(event):self.clear_motion();self.visual_calls=0;event_log(self.cfg.state,'human_confirmed',revision=self.mission.revision)
            elif kind in ('pause','reject','release','voice_abort'):
                self.clear_motion();self.mission.pause();self.voice_revision=None
            elif kind=='prepare':
                # Reacquiring authority must never resume old controls automatically.
                self.clear_motion()
                if self.mission.targets:
                    rev=self.mission.interrupt();self.mission.propose({'intent':'revise','targets':self.mission.targets[:]},rev)
            elif kind=='complete' and self.mission.mark_complete(event):self.clear_motion()

    def completed_requests(self,t,frames):
        if self.task_future and self.task_future[0].done():
            future,rev=self.task_future;self.task_future=None
            try:
                text,result=future.result()
                if rev==self.mission.revision:
                    self.mission.propose(result['answer'],rev);self.mission.history.append({'text':text,'response':result['answer']});event_log(self.cfg.state,'candidate_plan',revision=rev,call_id=result['call_id'],targets=result['answer']['targets'])
            except Exception as exc:
                if rev==self.mission.revision:self.mission.pause();self.last_error=type(exc).__name__
                event_log(self.cfg.state,'task_error',error=type(exc).__name__)
        if self.motion_future and self.motion_future['future'].done():
            request=self.motion_future;self.motion_future=None
            try:
                result=request['future'].result()
                if request['revision']!=self.mission.revision or self.mission.status!='active':return
                if not frames or not healthy(t,self.cfg) or not stable(t):raise ValueError('feedback invalid after model analysis')
                if self.integrated_hover>.12 or abs((t['yaw_deg']-self.query_yaw+180)%360-180)>1:raise ValueError('moved during analysis')
                v,ms=validate_motion(result['answer'],self.mission,self.cfg)
                # Only keep green images the API itself identified, labelled historical.
                if 'green' in result['answer'].get('visible',[]):self.green_history.append(request['frames'][-1])
                self.last_candidate={'call_id':result['call_id'],'decision':result['answer'],'execution_enabled':self.cfg.flight_enabled}
                atomic_json(self.cfg.state/'last-motion-candidate.json',self.last_candidate)
                if self.cfg.flight_enabled and any(v.values()):
                    self.motion_id+=1;start=now_ms();self.segment={'velocity':v,'start':start,'deadline':start+ms,'actual_sum':[0.,0.,0.],'samples':0,'call_id':result['call_id']}
                    event_log(self.cfg.state,'segment_accepted',call_id=result['call_id'],motion_id=self.motion_id,velocity=v,duration_ms=ms,revision=self.mission.revision)
                else:self.stable_since=now_ms()+1000
            except Exception as exc:
                self.clear_motion();self.mission.pause();self.last_error=type(exc).__name__;event_log(self.cfg.state,'motion_error',error=type(exc).__name__)

    def tick(self):
        now=now_ms()
        try:t=read_json(self.cfg.state/'ipc/telemetry.json')
        except (OSError,ValueError):t={}
        boot=t.get('bridge_boot')
        if self.boot and boot!=self.boot:
            self.clear_motion();self.mission.pause()
        self.boot=boot
        if t.get('clock_id')!=clock_id():t={}
        self.events();frames=self.camera.sample();self.completed_requests(t,frames)
        if self.motion_future and healthy(t,self.cfg):self.integrated_hover+=math.sqrt(sum(x*x for x in t['velocity_neu']))*min(.1,max(0,(now-self.last_tick)/1000))
        self.last_tick=now
        if self.mission.status=='active' and not self.segment and not self.motion_future and frames and healthy(t,self.cfg) and stable(t):
            if self.stable_since is None:self.stable_since=now
            if now-self.stable_since>=500:
                if self.visual_calls>=self.cfg.max_visual_calls:
                    self.clear_motion();self.mission.pause();self.publish(t);return
                self.visual_calls+=1
                task={'targets':self.mission.targets[:],'target':self.mission.target,'completed':self.mission.completed[:]}
                self.integrated_hover=0.;self.query_yaw=t['yaw_deg']
                allowed_telemetry={k:t[k] for k in ('altitude_m','velocity_neu','yaw_deg','yaw_rate_deg_s')}
                future=self.pool.submit(self.client.motion,task,frames,list(self.green_history),allowed_telemetry,list(self.recent_actions),self.cfg.camera_note)
                self.motion_future={'future':future,'revision':self.mission.revision,'frames':frames};self.stable_since=None
        else:
            if not self.motion_future:self.stable_since=None
        self.publish(t)

    def run(self):
        if not self.cfg.key or self.cfg.key=='Your API Key':raise ValueError('请先在 .env 填写 OPENAI_API_KEY')
        if self.cfg.flight_enabled:
            import sys
            if sys.platform!='linux':raise RuntimeError('real flight requires Linux onboard computer')
        self.camera.start()
        try:
            while not self.stop.is_set():
                start=time.monotonic();self.tick();self.stop.wait(max(0,.05-(time.monotonic()-start)))
        finally:
            self.clear_motion();self.mission.pause();self.publish({});self.camera.close();self.pool.shutdown(wait=False,cancel_futures=True)
