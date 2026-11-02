"""Estimate RELATIVE video/pose log lag from a stationary landmark yaw sweep.

Positive lag means pose_log_time = decoder_frame_time - lag. This is not an
exposure timestamp calibration or SDK clock conversion. Never enables control.
"""
import argparse
import csv
import io
import json
import math
from pathlib import Path
import numpy as np


def read_csv(path):
    return list(csv.DictReader(io.StringIO(Path(path).read_text(
        encoding='utf-8-sig').rstrip('\x00'))))


def validate(rows, time_key):
    if len(rows) < 2:
        raise ValueError('insufficient rows')
    clocks = {r['clock_id'] for r in rows}
    if len(clocks) != 1 or not next(iter(clocks)).startswith('mono-v1:'):
        raise ValueError('missing or mixed boot clock')
    times = np.array([float(r[time_key]) for r in rows])
    if not np.all(np.isfinite(times)) or times[0] < 0 or np.any(np.diff(times) <= 0):
        raise ValueError('invalid or nonmonotonic timestamps')
    return times, next(iter(clocks))


def audit_raw_timing(rows, clock_id, time_range):
    """Inspect original SDK fields without assuming their units/epoch relationship."""
    selected = [r for r in rows if r.get('clock_id') == clock_id and
                time_range[0]-1000 <= float(r['host_read_end_ms']) <= time_range[1]+1000]
    if len(selected) < 2: return dict(status='missing', samples=len(selected))
    ends, _ = validate(selected, 'host_read_end_ms')
    begin = np.array([float(r['host_read_begin_ms']) for r in selected])
    if not np.all(np.isfinite(begin)) or np.any(begin < 0) or np.any(begin > ends):
        raise ValueError('invalid SDK read interval')
    result = dict(status='raw_only', samples=len(selected),
                  read_duration_p95_ms=float(np.percentile(ends-begin,95)),
                  host_interval_p95_ms=float(np.percentile(np.diff(ends),95)))
    for field in ('sdk_millisecond_raw','sdk_microsecond_raw'):
        values = np.array([int(r[field]) for r in selected], dtype=np.int64)
        if np.any(values < 0) or np.any(values > 0xffffffff):
            raise ValueError('invalid raw SDK field')
        delta = np.diff(values)
        result[field] = dict(min=int(min(values)), max=int(max(values)),
            repeats=int(sum(delta == 0)), decreases=int(sum(delta < 0)))
    result['clock_mapping_verified'] = False
    return result


def interpolate_supported(query, times, values, max_gap_ms=150):
    """Interpolate only where each query has nearby actual pose support.

    Exact samples need no interpolation. Check both enclosing samples otherwise,
    including samples outside the queried time range; cropping first can hide a
    boundary gap. Unrelated gaps elsewhere in the same boot do not matter.
    """
    right = np.searchsorted(times, query, side='left')
    left = np.searchsorted(times, query, side='right')-1
    if np.any(left < 0) or np.any(right >= len(times)):
        raise ValueError('pose interpolation would extrapolate beyond support')
    if np.any(times[right]-times[left] > max_gap_ms):
        raise ValueError('pose interpolation support gap exceeds 150ms')
    return np.interp(query, times, values)


def estimate(observations, poses, search_ms=1000, step_ms=5):
    """Fit first half; independently fit second half and test first-half lag.

    Inputs: static landmark bearing_deg at decoder frame_ts_ms, camera yaw_deg
    at host ts_ms. Same boot required. All lag candidates use IDENTICAL samples.
    Quality thresholds are conservative engineering gates, not confidence bounds.
    """
    if not 0 < step_ms <= 25 or not 100 <= search_ms <= 3000:
        raise ValueError('invalid search range or resolution')
    t, clock = validate(observations, 'frame_ts_ms')
    p, pc = validate(poses, 'ts_ms')
    if clock != pc:
        raise ValueError('video and pose boot differ')
    raw_yaw = np.array([float(r['yaw_deg']) for r in poses])
    # Legacy bearing/pose CSVs may omit pitch. Validate it when supplied rather
    # than letting impossible finite SDK values pass through yaw-only fitting.
    pitch = np.array([float(r['pitch_deg']) for r in poses if 'pitch_deg' in r])
    b = np.array([float(r['bearing_deg']) for r in observations])
    if not all(np.all(np.isfinite(a)) for a in (raw_yaw, pitch, b)):
        raise ValueError('nonfinite angles')
    if np.any(abs(raw_yaw) > 360) or np.any(abs(pitch) > 180):
        raise ValueError('invalid camera angles: yaw outside +/-360 or pitch outside +/-180')
    # Check SDK ranges before unwrapping; normal +/-180 crossings remain valid.
    y = np.degrees(np.unwrap(np.radians(raw_yaw)))
    # Avoid endpoint extrapolation and support-changing bias across lag candidates.
    keep = (t-search_ms >= p[0]) & (t+search_ms <= p[-1])
    t, b = t[keep], b[keep]
    if len(t) < 80 or t[-1]-t[0] < 8000:
        raise ValueError('need >=80 supported samples spanning >=8 seconds')
    if np.max(np.diff(t)) > 300:
        raise ValueError('landmark tracking gap exceeds 300ms')
    lags = np.arange(-search_ms, search_ms+step_ms/2, step_ms)
    # Every candidate uses the same observations and must have valid support.
    # Do not repair bad data by discarding only the inconvenient lag candidates.
    aligned_yaw = interpolate_supported(t[None, :]-lags[:, None], p, y)
    split = len(t)//2
    def curve(yi, bs):
        world = yi+bs
        # A fixed landmark must have a constant world bearing; unknown bearing
        # is a nuisance offset. Constant-speed motion cannot identify lag.
        residual = world-np.median(world, axis=1)[:, None]
        return np.sqrt(np.mean(residual**2, axis=1))
    train = curve(aligned_yaw[:, :split], b[:split])
    hold = curve(aligned_yaw[:, split:], b[split:])
    i, j = int(np.argmin(train)), int(np.argmin(hold))
    best, hold_best = float(lags[i]), float(lags[j])
    reasons = []
    widths = []
    for name, score, idx, ts, yi in (
            ('fit', train, i, t[:split], aligned_yaw[i, :split]),
            ('holdout', hold, j, t[split:], aligned_yaw[j, split:])):
        near = lags[score <= score[idx]+0.05]
        width = float(near[-1]-near[0]); widths.append(width)
        if idx in (0, len(lags)-1): reasons.append(name+': optimum at search boundary')
        if width > 50: reasons.append(name+': lag ambiguous (>50ms profile width)')
        rates = np.diff(yi)/np.diff(ts)*1000
        if np.ptp(yi) < 5 or not (np.any(rates > 3) and np.any(rates < -3)):
            reasons.append(name+': need >5deg sweep and both rotation directions')
    if abs(best-hold_best) > 25: reasons.append('fit/holdout lag disagreement >25ms')
    if float(hold[i]) > 0.5: reasons.append('holdout bearing RMS >0.5deg')
    zero = int(np.argmin(abs(lags)))
    return dict(schema=1, clock_id=clock, status='candidate' if not reasons else 'rejected',
        reasons=reasons, relative_lag_ms=best, holdout_lag_ms=hold_best,
        profile_width_ms=widths, holdout_rms_deg=float(hold[i]),
        unaligned_holdout_rms_deg=float(hold[zero]), fit_rms_deg=float(train[i]),
        samples=len(t), valid_frame_range_ms=[float(t[0]), float(t[-1])],
        timestamp_relation='pose_log_time = decoder_frame_time - relative_lag_ms',
        verified=False, exposure_calibrated=False, shadow_only=True,
        warning='Relative lag only; profile width is NOT a statistical confidence interval. '
                'Requires fixed aircraft, static landmark, fixed zoom and horizontal gimbal sweep.',
        profile=[dict(lag_ms=float(l), fit_rms_deg=float(a), holdout_rms_deg=float(b))
                 for l,a,b in zip(lags,train,hold)])


def track(session, roi, fx, start=0, end=None):
    """Sequential decode; track original fixed features, never reseed on a new object."""
    import cv2
    session = Path(session)
    rows = read_csv(session/'frames.csv')
    validate(rows, 'frame_ts_ms')
    if any(int(r['video_frame']) != i for i,r in enumerate(rows)):
        raise ValueError('noncontiguous frame index')
    if not math.isfinite(fx) or fx <= 1 or start < 0 or start >= len(rows):
        raise ValueError('invalid focal length/start frame')
    video = session/'liveview.mkv'
    if not video.exists(): video = session/'liveview.avi'
    cap = cv2.VideoCapture(str(video)); result = []
    old = pts = base = None
    try:
        for i,r in enumerate(rows):
            if end is not None and i >= end: break
            ok, frame = cap.read()
            if not ok: raise ValueError('video ended before indexed frame '+str(i))
            if i < start: continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if old is None:
                x,yy,w,h = roi
                if min(x,yy) < 0 or min(w,h) < 5 or x+w > gray.shape[1] or yy+h > gray.shape[0]:
                    raise ValueError('ROI outside image or too small')
                mask = np.zeros_like(gray); mask[yy:yy+h,x:x+w] = 255
                pts = cv2.goodFeaturesToTrack(gray, 100, .02, 5, mask=mask)
                if pts is None or len(pts) < 12: raise ValueError('need >=12 landmark corners')
                base = np.degrees(np.arctan((pts[:,0,0]-gray.shape[1]/2)/fx))
            else:
                nxt, good, _ = cv2.calcOpticalFlowPyrLK(old, gray, pts, None)
                if nxt is None: raise ValueError('landmark lost')
                back, reverse, _ = cv2.calcOpticalFlowPyrLK(gray, old, nxt, None)
                if back is None: raise ValueError('reverse tracking failed')
                valid = (good.ravel() == 1) & (reverse.ravel() == 1)
                valid &= np.linalg.norm(back-pts, axis=2).ravel() < .75
                valid &= (nxt[:,0,0] >= 0) & (nxt[:,0,0] < gray.shape[1])
                valid &= (nxt[:,0,1] >= 0) & (nxt[:,0,1] < gray.shape[0])
                pts, base = nxt[valid], base[valid]
                if len(pts) < 12: raise ValueError('landmark lost at frame '+str(i))
            bearing = np.degrees(np.arctan((pts[:,0,0]-gray.shape[1]/2)/fx))-base
            center = float(np.median(bearing))
            if np.percentile(abs(bearing-center),90) > .3:
                raise ValueError('inconsistent landmark motion at frame '+str(i))
            result.append(dict(frame_ts_ms=r['frame_ts_ms'], clock_id=r['clock_id'],
                               bearing_deg=center, video_frame=i, tracked_points=len(pts)))
            old = gray
    finally: cap.release()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--observations', type=Path, help='static landmark bearing CSV')
    source.add_argument('--session', type=Path, help='recording directory')
    parser.add_argument('--poses', required=True, type=Path, help='v2 pose CSV from same boot')
    parser.add_argument('--raw-timing', type=Path, help='optional v3 raw timing CSV for diagnostics')
    parser.add_argument('--roi', nargs=4, type=int, help='fixed landmark x y width height at start frame')
    parser.add_argument('--fx', type=float, help='focal length in recorded image pixels, fixed zoom')
    parser.add_argument('--start-frame', type=int, default=0)
    parser.add_argument('--end-frame', type=int)
    parser.add_argument('--output', type=Path, required=True, help='new report directory')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        if args.session:
            if args.roi is None or args.fx is None: raise ValueError('--session needs --roi and --fx')
            obs = track(args.session,args.roi,args.fx,args.start_frame,args.end_frame)
        else: obs = read_csv(args.observations)
        with (args.output/'landmark.csv').open('w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=list(obs[0]));writer.writeheader();writer.writerows(obs)
        _, selected_clock = validate(obs,'frame_ts_ms')
        pose_rows = read_csv(args.poses)
        selected = [r for r in pose_rows if r.get('clock_id') == selected_clock]
        report = estimate(obs, selected)
        report['excluded_other_boot_pose_rows'] = len(pose_rows)-len(selected)
        report['raw_timing'] = audit_raw_timing(read_csv(args.raw_timing),report['clock_id'],
            report['valid_frame_range_ms']) if args.raw_timing else dict(status='not_provided')
        report['sources'] = dict(poses=str(args.poses.resolve()),
            observations=str((args.observations or args.session).resolve()),
            roi=args.roi,fx=args.fx,start_frame=args.start_frame,end_frame=args.end_frame)
    except (ValueError,KeyError,IndexError) as error:
        report = dict(status='rejected',reason=str(error),verified=False,shadow_only=True)
    (args.output/'alignment_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k != 'profile'},indent=2))


if __name__ == '__main__': main()
