"""Shadow-only relative video/pose phase diagnostic without a calibrated focal length.

Model: x_px = intercept_px + scale_px_per_logged_yaw_deg * yaw(frame_ts_ms-lag_ms).
Positive lag compares an image with an earlier pose log time. This small-angle,
near-horizontal model estimates neither focal length nor true exposure time.
"""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path

import numpy as np


FLAGS = dict(shadow_only=True, verified=False, exposure_calibrated=False)
MAX_PITCH_SPAN_DEG = 2.0
LIMITS = [
    'Relative phase of two logs only, not camera exposure calibration or SDK clock conversion.',
    'Fixed aircraft, static tracked landmark, fixed zoom and near-horizontal small-angle sweep required.',
    'The fitted pixel/yaw scale is a nuisance coefficient, NOT calibrated focal length.',
    'At 10Hz pose sampling, a 5ms or 10ms scan step does not imply 5ms or 10ms accuracy.',
    'Profile width is an engineering identifiability check, NOT a confidence interval.',
    'Pitch span <=2deg and recorded landmark y motion are required engineering checks, not a calibrated attitude tolerance.',
    'Motion checks use the complete pose support envelope for ALL scanned lags, including both interpolation boundary samples.',
    'No runtime setting is written; a candidate remains shadow-only and unverified.',
]


def read_csv(path):
    raw = Path(path).read_bytes()
    if b'\x00' in raw:
        raise ValueError('CSV contains zero bytes; use a separately audited clean subset')
    rows = list(csv.DictReader(io.StringIO(raw.decode('utf-8-sig'))))
    if any(None in r or any(v is None for v in r.values()) for r in rows):
        raise ValueError('malformed CSV row')
    return rows


def numeric(rows, key):
    try:
        if any(isinstance(r[key], bool) for r in rows):
            raise ValueError('boolean is not numeric measurement')
        values = np.array([float(r[key]) for r in rows], dtype=float)
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('missing or invalid '+key) from exc
    if not np.all(np.isfinite(values)):
        raise ValueError('nonfinite '+key)
    return values


def validate(rows, key):
    if len(rows) < 2:
        raise ValueError('insufficient '+key+' rows')
    clocks = {r.get('clock_id') for r in rows if isinstance(r.get('clock_id'), str)}
    if len(clocks) != 1 or any(not isinstance(r.get('clock_id'), str) or r.get('clock_id') not in clocks for r in rows):
        raise ValueError('missing or mixed boot clock')
    clock = next(iter(clocks))
    if not clock.startswith('mono-v1:') or len(clock) <= 8:
        raise ValueError('unidentified boot clock')
    t = numeric(rows, key)
    if t[0] < 0 or np.any(np.diff(t) <= 0):
        raise ValueError('invalid or nonmonotonic '+key)
    return t, clock


def pose_token(row):
    values = [row[k] for k in ('session_token', 'token') if row.get(k) not in (None, '')]
    if any(not isinstance(v, str) for v in values) or len(set(values)) > 1:
        raise ValueError('invalid or conflicting pose token fields')
    return values[0] if values else None


def fit(yaw, x):
    centered = yaw-yaw.mean()
    variance = float(centered @ centered)
    scale = float(centered @ (x-x.mean())/variance) if variance > 1e-12 else 0.0
    intercept = float(x.mean()-scale*yaw.mean())
    rms = float(np.sqrt(np.mean((x-intercept-scale*yaw)**2)))
    return intercept, scale, rms


def estimate(observations, poses, search_ms=1000, step_ms=10):
    """Fit chronological first half, score fixed coefficients on held-out half.

    The held-out half also gets an independent lag fit, used only as a stability
    check. All lag candidates use identical samples; interpolation never spans
    a pose gap >150ms. Thresholds below are conservative engineering gates.
    """
    if (isinstance(search_ms, bool) or isinstance(step_ms, bool) or
            not np.isfinite(search_ms) or not np.isfinite(step_ms) or
            not 100 <= search_ms <= 3000 or not 1 <= step_ms <= 25 or
            abs(search_ms/step_ms-round(search_ms/step_ms)) > 1e-8):
        raise ValueError('search must be 100..3000ms and divisible by step (1..25ms)')
    t, clock = validate(observations, 'frame_ts_ms')
    p, pose_clock = validate(poses, 'ts_ms')
    if clock != pose_clock:
        raise ValueError('video and pose boot differ')
    selected_tokens = [pose_token(r) for r in poses]
    tokens = {token for token in selected_tokens if token}
    if len(tokens) > 1 or tokens and any(token is None for token in selected_tokens):
        raise ValueError('pose rows must be filtered to one capture token')
    x = numeric(observations, 'x_px')
    raw_yaw = numeric(poses, 'yaw_deg')
    if np.any(abs(raw_yaw) > 360):
        raise ValueError('yaw outside physical input gate [-360,360]deg')
    pitch = None
    if any('pitch_deg' in row for row in poses):
        pitch = numeric(poses, 'pitch_deg')
        if np.any(abs(pitch) > 180):
            raise ValueError('pitch outside physical input gate [-180,180]deg')
    supported = (t-search_ms >= p[0]) & (t+search_ms <= p[-1])
    total_frames = len(t)
    t, x = t[supported], x[supported]
    if len(t) < 80 or t[-1]-t[0] < 8000:
        raise ValueError('need >=80 jointly supported frames spanning >=8 seconds')
    if np.max(np.diff(t)) > 300:
        raise ValueError('landmark gap >300ms inside supported range')
    lags = np.arange(-search_ms, search_ms+step_ms/2, step_ms, dtype=float)
    queries = t[None, :]-lags[:, None]
    right = np.searchsorted(p, queries, side='left').clip(0, len(p)-1)
    left = np.maximum(right-1, 0)
    left = np.where(p[right] == queries, right, left)
    gaps = p[right]-p[left]
    if np.any(gaps > 150):
        raise ValueError('pose support gap >150ms for at least one lag candidate')
    # Determine support before checking motion. An initialization jump outside
    # every candidate query must not veto a later, separately chosen interval.
    # Keep BOTH bracketing samples, even if outside the query time range, and
    # every intervening row: this cannot discard an inconvenient boundary jump
    # or choose a different set of poses for the winning lag.
    support_begin, support_end = int(left.min()), int(right.max())+1
    support_p = p[support_begin:support_end]
    yaw = np.degrees(np.unwrap(np.radians(raw_yaw[support_begin:support_end])))
    if np.any(abs(np.diff(yaw)) > 30):
        raise ValueError('pose jumps >30deg between adjacent samples in query support')
    samples = np.array([np.interp(q, support_p, yaw) for q in queries])
    split = len(t)//2
    train = np.array([fit(row[:split], x[:split]) for row in samples])
    hold = np.array([fit(row[split:], x[split:]) for row in samples])
    i, j = int(np.argmin(train[:, 2])), int(np.argmin(hold[:, 2]))
    fixed_hold = np.sqrt(np.mean((x[None, split:]-train[:, 0, None]-
                                 train[:, 1, None]*samples[:, split:])**2, axis=1))
    reasons, halves = [], []
    pitch_motion = dict(status='not_provided; near-horizontal sweep not verified',
                        maximum_span_deg=MAX_PITCH_SPAN_DEG)
    if pitch is None:
        reasons.append('pitch_deg missing; near-horizontal sweep not verified')
    else:
        support_pitch = pitch[support_begin:support_end]
        pitch_span = float(np.ptp(support_pitch))
        pitch_motion = dict(status='within_gate' if pitch_span <= MAX_PITCH_SPAN_DEG else 'exceeds_gate',
                            min_deg=float(support_pitch.min()), max_deg=float(support_pitch.max()),
                            span_deg=pitch_span, maximum_span_deg=MAX_PITCH_SPAN_DEG)
        if pitch_span > MAX_PITCH_SPAN_DEG:
            reasons.append('pitch span >2deg in query support; horizontal model questionable')
    for name, rows, idx, ts, xs in (('train', train, i, t[:split], x[:split]),
                                   ('holdout', hold, j, t[split:], x[split:])):
        ys = samples[idx, :split] if name == 'train' else samples[idx, split:]
        rates = np.diff(ys)/np.diff(ts)*1000
        if np.ptp(ys) < 3 or np.ptp(xs) < 8:
            reasons.append(name+': insufficient yaw/pixel motion')
        if np.ptp(ys) > 40:
            reasons.append(name+': yaw sweep >40deg exceeds small-angle diagnostic gate')
        if sum(rates > 1) < 3 or sum(rates < -1) < 3:
            reasons.append(name+': both rotation directions required; one-way/constant-rate motion cannot identify phase')
        if ts[-1]-ts[0] < 3000:
            reasons.append(name+': sample span <3s')
        tolerance = max(.25, .02*float(np.std(xs)))
        near = lags[rows[:, 2] <= rows[idx, 2]+tolerance]
        width = float(near[-1]-near[0])
        if width > 100:
            reasons.append(name+': nonunique phase (>100ms near-optimum profile span)')
        if idx in (0, len(lags)-1):
            reasons.append(name+': optimum at search boundary')
        rms_limit = max(1.5, .08*float(np.ptp(xs)))
        if rows[idx, 2] > rms_limit:
            reasons.append(name+': pixel residual exceeds model-quality gate')
        halves.append(dict(name=name, samples=len(xs), best_lag_ms=float(lags[idx]),
            independent_rms_px=float(rows[idx, 2]), near_optimum_span_ms=width,
            near_optimum_tolerance_px=tolerance, rms_limit_px=rms_limit,
            intercept_px=float(rows[idx, 0]), scale_px_per_logged_yaw_deg=float(rows[idx, 1])))
    if abs(lags[i]-lags[j]) > 50:
        reasons.append('train/holdout lag disagreement >50ms')
    scale, hold_scale = float(train[i, 1]), float(hold[j, 1])
    if abs(scale) < 1e-8 or scale*hold_scale <= 0 or not .8 <= hold_scale/scale <= 1.25:
        reasons.append('train/holdout pixel-yaw sign or scale unstable')
    quarter_scales = []
    for indices in np.array_split(np.arange(len(t)), 4):
        ys = samples[i, indices]
        if np.ptp(ys) < 1:
            quarter_scales.append(None)
            continue
        part_scale = fit(ys, x[indices])[1]
        quarter_scales.append(part_scale)
        if abs(scale) < 1e-8 or part_scale*scale <= 0 or not .75 <= part_scale/scale <= 1.3334:
            reasons.append('quarter-window pixel-yaw sign or scale unstable')
    if fixed_hold[i] > halves[1]['rms_limit_px']:
        reasons.append('held-out residual with fixed training coefficients exceeds gate')
    vertical = dict(status='not_provided; near-horizontal sweep not verified')
    if any('y_px' in r for r in observations):
        yy = numeric(observations, 'y_px')[supported]
        y_span, x_span = float(np.ptp(yy)), float(np.ptp(x))
        vertical = dict(status='within_gate' if y_span <= max(3, .25*x_span) else 'exceeds_gate',
                        span_px=y_span, horizontal_span_px=x_span,
                        warning='vertical image motion can reflect pitch, roll, parallax or tracking error; not a pitch calibration')
        if y_span > max(3, .25*x_span):
            reasons.append('substantial vertical image motion; horizontal model questionable')
    else:
        reasons.append('y_px missing; near-horizontal sweep not verified')
    reasons = list(dict.fromkeys(reasons))
    return dict(schema_version=1, status='candidate' if not reasons else 'rejected', **FLAGS,
        clock_id=clock, capture_token=next(iter(tokens)) if tokens else None, reasons=reasons,
        relative_lag_ms=float(lags[i]), holdout_lag_ms=float(lags[j]),
        fixed_coefficients_holdout_rms_px=float(fixed_hold[i]), halves=halves,
        quarter_scales_px_per_logged_yaw_deg=quarter_scales, vertical_motion=vertical,
        pitch_motion=pitch_motion,
        samples=len(t), excluded_edge_frames=total_frames-len(t),
        valid_frame_range_ms=[float(t[0]), float(t[-1])], split_frame_ts_ms=float(t[split]),
        scan_step_ms=float(step_ms), maximum_query_support_gap_ms=float(gaps.max()),
        pose_interval_median_ms=float(np.median(np.diff(support_p))),
        pose_support_range_ms=[float(support_p[0]), float(support_p[-1])],
        pose_support_rows=len(support_p), excluded_outside_support_pose_rows=len(p)-len(support_p),
        pose_validation_scope='all candidate lag query support envelope, including interpolation boundaries',
        timestamp_relation='pose_log_time = decoder_frame_time - relative_lag_ms',
        limits=LIMITS, profile=[dict(lag_ms=float(l), train_rms_px=float(a[2]),
            holdout_independent_rms_px=float(b[2]), holdout_fixed_train_coefficients_rms_px=float(c))
            for l, a, b, c in zip(lags, train, hold, fixed_hold)])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observations', type=Path, required=True, help='frame_ts_ms,x_px,y_px,clock_id; missing y_px cannot yield a candidate')
    parser.add_argument('--poses', type=Path, required=True, help='ts_ms,yaw_deg,pitch_deg,clock_id; prefiltered capture token; pitch required for a candidate')
    parser.add_argument('--token', help='explicitly select this session_token (or token) before fitting')
    parser.add_argument('--search-ms', type=float, default=1000)
    parser.add_argument('--step-ms', type=float, default=10)
    parser.add_argument('--output', type=Path, required=True, help='new JSON report path')
    args = parser.parse_args(argv)
    if args.output.exists() or args.output.resolve() in (args.poses.resolve(), args.observations.resolve()):
        parser.error('output must be a new path distinct from source files')
    sources = {}
    try:
        for name, path in (('observations', args.observations), ('poses', args.poses)):
            raw = path.read_bytes()
            sources[name] = dict(path=str(path.resolve()), sha256=hashlib.sha256(raw).hexdigest())
        obs, poses = read_csv(args.observations), read_csv(args.poses)
        before = len(poses)
        if args.token:
            poses = [r for r in poses if pose_token(r) == args.token]
        report = estimate(obs, poses, args.search_ms, args.step_ms)
        report['excluded_pose_token_rows'] = before-len(poses)
    except (ValueError, KeyError, TypeError, OSError, UnicodeError, csv.Error) as exc:
        report = dict(schema_version=1, status='rejected', reasons=[str(exc)], **FLAGS, limits=LIMITS)
    report['sources'] = sources
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'profile'}, ensure_ascii=False, indent=2))
    return 0 if report['status'] == 'candidate' else 2


if __name__ == '__main__':
    raise SystemExit(main())
