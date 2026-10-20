from __future__ import annotations
import math
from .io import now_ms
ZERO={'x':0.,'y':0.,'z':0.,'yaw':0.}

def finite(x):return type(x) in (int,float) and math.isfinite(x)

def validate_motion(d, mission, cfg):
    if d.get('plan')!=mission.targets or d.get('target')!=mission.target:raise ValueError('motion task mismatch')
    mode=d.get('mode');v=d.get('velocity');duration=d.get('duration_s')
    visible=d.get('visible')
    if not isinstance(visible,list) or any(c not in ('red','green','purple') for c in visible):raise ValueError('invalid visible colors')
    if mode not in ('approach','search_left','search_right','altitude_search','hold'):raise ValueError('unsupported motion mode')
    if not isinstance(v,dict) or set(v)!=set(ZERO) or not all(finite(x) for x in v.values()):raise ValueError('invalid velocity')
    if not finite(duration) or not .1<=duration<=cfg.max_duration:raise ValueError('invalid duration')
    if math.hypot(v['x'],v['y'])>cfg.max_horizontal or abs(v['z'])>cfg.max_vertical or abs(v['yaw'])>cfg.max_yaw:raise ValueError('velocity limit')
    if mode=='hold' and any(v.values()):raise ValueError('nonzero hold')
    if mode.startswith('search_') and (v['x'] or v['y'] or v['z'] or (mode=='search_left' and v['yaw']>=0) or (mode=='search_right' and v['yaw']<=0)):raise ValueError('invalid search')
    if mode=='altitude_search' and (v['x'] or v['y']):raise ValueError('invalid altitude search')
    if mode=='approach':
        b=d.get('bbox')
        if not isinstance(b,list) or len(b)!=4 or not all(finite(x) and 0<=x<=1 for x in b) or not(b[0]<b[2] and b[1]<b[3]):raise ValueError('invalid model bbox')
        if mission.target not in d.get('visible',[]):raise ValueError('approach without visible target')
        center=(b[0]+b[2])/2
        if center<.4 and (v['y']>0 or v['yaw']>0):raise ValueError('wrong left correction')
        if center>.6 and (v['y']<0 or v['yaw']<0):raise ValueError('wrong right correction')
        if abs((b[1]+b[3])/2-.5)>.2 and v['x']>0:raise ValueError('align target height before forward movement')
    return dict(v),math.floor(duration*20+1e-8)*50

def healthy(t, cfg, now=None):
    now=now_ms() if now is None else now
    try:
        return (0<=now-t['time_ms']<=cfg.telemetry_max_age_ms and t['fresh'] is True and t['in_air'] is True and t['velocity_healthy'] is True and cfg.min_altitude<=t['altitude_m']<=cfg.max_altitude and (not cfg.flight_enabled or t['has_authority'] is True) and t['prepared'] is True)
    except (KeyError,TypeError):return False

def stable(t):
    try:return math.sqrt(sum(x*x for x in t['velocity_neu']))<.06 and abs(t['yaw_rate_deg_s'])<.3
    except (KeyError,TypeError):return False
