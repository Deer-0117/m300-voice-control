"""Shadow-only delayed bearing KF, NOT the paper's 18-state inertial EKF.

Inputs must be mapped measurement times in one boot's monotonic clock.
No aircraft commands, no assumed depth, no inference that host arrival is exposure.
"""
from dataclasses import dataclass
import argparse
import bisect
import json
import math
from pathlib import Path


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class Alignment:
    clock_id: str
    calibration_id: str
    verified: bool
    uncertainty_ms: float
    max_uncertainty_ms: float = 25.0

    def check(self):
        if (not self.clock_id.startswith('mono-v1:') or not self.calibration_id or
                not self.verified or not math.isfinite(self.uncertainty_ms) or
                not 0 <= self.uncertainty_ms <= self.max_uncertainty_ms):
            raise ValueError('measurement-time alignment unverified or too uncertain')


@dataclass(frozen=True)
class Pose:
    timestamp_ms: int
    received_ms: int
    yaw: float
    pitch: float
    clock_id: str


@dataclass(frozen=True)
class ClockMapping:
    """An externally calibrated affine mapping; never fitted from arrival alone.

    source_ms must already have documented units. Do not guess the relationship
    between the two raw SDK timestamp fields collected by the recorder.
    """
    alignment: Alignment
    source_clock: str
    scale: float
    offset_ms: float
    source_min_ms: float
    source_max_ms: float

    def map(self, source_ms, source_clock, received_ms):
        self.alignment.check()
        if (source_clock != self.source_clock or
                not all(math.isfinite(v) for v in (source_ms,self.scale,self.offset_ms,received_ms)) or
                self.scale <= 0 or not self.source_min_ms <= source_ms <= self.source_max_ms):
            raise ValueError('source clock or calibration validity mismatch')
        mapped=self.scale*source_ms+self.offset_ms
        if not 0 <= mapped <= received_ms:
            raise ValueError('mapped measurement is in the future or before boot')
        return round(mapped)


def pose_at(poses, timestamp, now, alignment):
    alignment.check()
    valid = sorted((p for p in poses if p.clock_id == alignment.clock_id and
                    0 <= p.timestamp_ms <= p.received_ms <= now and
                    math.isfinite(p.yaw) and math.isfinite(p.pitch)),
                   key=lambda p: p.timestamp_ms)
    times = [p.timestamp_ms for p in valid]
    j = bisect.bisect_left(times, timestamp)
    if j < len(valid) and times[j] == timestamp:
        return valid[j].yaw, valid[j].pitch
    if j == 0 or j == len(valid):
        raise ValueError('no bracketing measurement-time camera poses')
    a,b=valid[j-1],valid[j]
    if timestamp-a.timestamp_ms > 100 or b.timestamp_ms-timestamp > 100:
        raise ValueError('camera pose gap exceeds 100ms per side')
    f=(timestamp-a.timestamp_ms)/(b.timestamp_ms-a.timestamp_ms)
    return wrap(a.yaw+f*wrap(b.yaw-a.yaw)), a.pitch+f*(b.pitch-a.pitch)


@dataclass
class Axis:
    angle: float
    rate: float = 0.0
    p00: float = 1.0
    p01: float = 0.0
    p11: float = 100.0

    def propagate(self, dt, q):
        self.angle += self.rate*dt
        self.p00 += 2*dt*self.p01 + dt*dt*self.p11 + q*dt**3/3
        self.p01 += dt*self.p11 + q*dt*dt/2
        self.p11 += q*dt

    def correct(self, residual, variance):
        s=self.p00+variance
        k0,k1=self.p00/s,self.p01/s
        self.angle += k0*residual
        self.rate += k1*residual
        p00,p01,p11=self.p00,self.p01,self.p11
        self.p00=p00*variance/s
        self.p01=p01*variance/s
        self.p11=max(1e-12,p11-p01*p01/s)


class DelayedBearingFilter:
    """Bounded measurement replay: insert at capture time, then refilter forward.

    A 64-observation sliding window is refit from its earliest observation.
    Its covariance is therefore a finite-window estimate, not full-history EKF.
    Calling predict never mutates the historical posterior.
    """
    def __init__(self, alignment, q=4.0, variance=.25, lag_ms=1500, capacity=64):
        if not all(math.isfinite(x) and x>0 for x in (q,variance,lag_ms)) or capacity<4:
            raise ValueError('invalid filter parameters')
        self.alignment=alignment;self.q=q;self.variance=variance
        self.lag_ms=lag_ms;self.capacity=capacity
        self.reset(None)

    def reset(self, target_id):
        self.target_id=target_id;self.measurements=[];self.axes=None
        self.timestamp=None;self.rejected=0;self.closed_before=-1
        self.last_arrival=-1

    def observe(self, capture_ms, received_ms, yaw, pitch, target_id, clock_id):
        self.alignment.check()
        if (clock_id != self.alignment.clock_id or not target_id or
                not all(math.isfinite(v) for v in (capture_ms,received_ms,yaw,pitch)) or
                not 0 <= capture_ms <= received_ms or received_ms-capture_ms>self.lag_ms):
            raise ValueError('invalid observation time, identity, clock or age')
        if received_ms < self.last_arrival:
            raise ValueError('arrival order reversed; reset for a new replay')
        if target_id != self.target_id:
            raise ValueError('target changed; caller must explicitly reset the filter')
        if capture_ms <= self.closed_before:
            raise ValueError('observation predates retained history')
        if any(m[0]==capture_ms for m in self.measurements):
            raise ValueError('duplicate observation timestamp')
        self.last_arrival=received_ms
        bisect.insort(self.measurements,(capture_ms,float(yaw),float(pitch)))
        while (len(self.measurements)>self.capacity or
               self.measurements[-1][0]-self.measurements[0][0]>self.lag_ms):
            self.closed_before=self.measurements.pop(0)[0]
        self._replay()

    def _replay(self):
        first=self.measurements[0]
        a,b=Axis(first[1],p00=self.variance),Axis(first[2],p00=self.variance)
        t=first[0];self.rejected=0;accepted_t=t
        accepted_axes=(Axis(**vars(a)),Axis(**vars(b)))
        for ts,yaw,pitch in self.measurements[1:]:
            dt=(ts-t)/1000.;a.propagate(dt,self.q);b.propagate(dt,self.q);t=ts
            ry,rp=wrap(yaw-a.angle),pitch-b.angle
            nis=ry*ry/(a.p00+self.variance)+rp*rp/(b.p00+self.variance)
            if nis>25.0:
                self.rejected+=1
                continue
            a.correct(ry,self.variance);b.correct(rp,self.variance)
            accepted_t=t;accepted_axes=(Axis(**vars(a)),Axis(**vars(b)))
        # Rejected observations must not refresh the estimate's freshness.
        self.timestamp=accepted_t;self.axes=accepted_axes

    def predict(self, now_ms, lead_ms=150, max_age_ms=500):
        self.alignment.check()
        if self.axes is None or now_ms < self.last_arrival or not 0<=lead_ms<=500:
            raise ValueError('no causal estimate or invalid lead')
        age=now_ms-self.timestamp
        if not 0<=age<=max_age_ms:
            raise ValueError('estimate stale')
        axes=[Axis(**vars(a)) for a in self.axes]
        for a in axes:a.propagate((age+lead_ms)/1000.,self.q)
        return dict(world_yaw_deg=wrap(axes[0].angle),world_pitch_deg=axes[1].angle,
            yaw_rate_degps=axes[0].rate,pitch_rate_degps=axes[1].rate,
            yaw_variance_deg2=axes[0].p00,pitch_variance_deg2=axes[1].p00,
            age_ms=age,lead_ms=lead_ms,total_propagation_ms=age+lead_ms,
            rejected_in_window=self.rejected,calibration_id=self.alignment.calibration_id,
            shadow_only=True,depth_available=False)


def observe_box(filter_, event, poses):
    """Connect mapped frame time -> historical camera pose -> delayed update.

    yaw/pitch bearing approximation only. Full roll/extrinsic/IMU dynamics of
    the paper are intentionally not represented by this small prototype.
    """
    if event.get('timestamp_kind') != 'mapped_measurement':
        raise ValueError('decoder/arrival timestamp is not a mapped measurement')
    for k in ('u','v','cx','cy','fx','fy'):
        if not math.isfinite(event[k]):raise ValueError('nonfinite image geometry')
    if event['fx']<=1 or event['fy']<=1:raise ValueError('invalid image focal length')
    yaw,pitch=pose_at(poses,event['capture_ms'],event['received_ms'],filter_.alignment)
    yaw=wrap(yaw+math.degrees(math.atan((event['u']-event['cx'])/event['fx'])))
    pitch-=math.degrees(math.atan((event['v']-event['cy'])/event['fy']))
    filter_.observe(event['capture_ms'],event['received_ms'],yaw,pitch,
                    event['target_id'],event['clock_id'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('packets',type=Path,help='mapped measurement JSONL; includes alignment/reset/pose/box/tick')
    parser.add_argument('output',type=Path)
    args=parser.parse_args();filter_=None;poses=[]
    with args.output.open('w',encoding='utf-8') as output:
        for line_no,line in enumerate(args.packets.read_text(encoding='utf-8').splitlines(),1):
            try:
                e=json.loads(line);kind=e.pop('kind')
                if kind=='alignment':
                    alignment=Alignment(**e);alignment.check()
                    filter_=DelayedBearingFilter(alignment);poses=[]
                elif filter_ is None:raise ValueError('alignment required first')
                elif kind=='reset':filter_.reset(e['target_id'])
                elif kind=='pose':poses.append(Pose(**e));poses=poses[-256:]
                elif kind=='box':observe_box(filter_,e,poses)
                elif kind=='tick':
                    result=filter_.predict(**e)
                    output.write(json.dumps(dict(line=line_no,**result))+'\n')
                else:raise ValueError('unknown packet kind')
            except (ValueError,KeyError,TypeError) as error:
                output.write(json.dumps(dict(line=line_no,status='rejected',reason=str(error)))+'\n')


if __name__=='__main__':main()
