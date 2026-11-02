"""Conditional, shadow-only local 2D video/pose phase diagnostic.

[x_px,y_px] = intercept + J * [unwrapped yaw,pitch](frame_ts_ms-lag_ms).
J is an empirical local pixel/degree Jacobian, NOT calibrated camera intrinsics
or a 3D projection. No exposure calibration or control setting is produced.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

try:
    from .calibrate_camera_phase import read_csv, numeric, validate, pose_token
except ImportError:
    from calibrate_camera_phase import read_csv, numeric, validate, pose_token


FLAGS = dict(shadow_only=True, verified=False, exposure_calibrated=False)
# Fixed before reading any hardware recording. Engineering gates, not confidence
# bounds; these are deliberately not CLI tuning knobs for passing a recording.
GATES = dict(min_frames=80, min_span_ms=8000, min_half_span_ms=3000,
    max_landmark_gap_ms=300, max_pose_support_gap_ms=150,
    max_adjacent_pose_jump_deg=30, max_support_yaw_span_deg=40,
    max_support_pitch_span_deg=30, min_axis_span_deg=3, min_axis_std_deg=.5,
    min_positive_and_negative_rate_samples=3, min_axis_rate_deg_s=1,
    max_centered_pose_condition=10, max_abs_axis_correlation=.95,
    min_image_span_px=8, min_jacobian_singular_value_px_per_deg=.25,
    max_jacobian_condition=20, max_profile_span_ms=100,
    profile_tolerance_floor_px=.35, profile_tolerance_std_fraction=.02,
    max_train_holdout_lag_difference_ms=50,
    max_relative_jacobian_change=.25, min_jacobian_column_cosine=.906307787,
    min_jacobian_column_scale_ratio=.75, max_jacobian_column_scale_ratio=1.333333334,
    residual_floor_px=2, residual_span_fraction=.08, max_observed_roll_span_deg=2)
LIMITS = [
    'Local 2D image/pose phase only; J is empirical pixel/degree coupling, not focal length, full 3D attitude compensation or depth.',
    'Fixed aircraft, static landmark, fixed zoom and a locally valid projection are required; image tracking is an independent prerequisite.',
    'Yaw and pitch must be separately excited. Correlated axes, constant-rate motion and phase aliases are rejected.',
    'Missing roll leaves an unobserved confound; a candidate is conditional and cannot establish that roll was constant.',
    'All lag candidates use the same frames and complete pose support envelope, including both interpolation boundary rows.',
    'At 10Hz pose sampling, a 5ms/10ms scan step does not establish 5ms/10ms accuracy. Profile width is not a confidence interval.',
    'No runtime setting is written. Exposure calibration and SDK clock mapping remain unverified.',
]


def fit(angles, pixels):
    centered = angles-angles.mean(axis=0)
    coefficients = np.linalg.lstsq(centered, pixels-pixels.mean(axis=0), rcond=None)[0]
    jacobian = coefficients.T
    intercept = pixels.mean(axis=0)-jacobian@angles.mean(axis=0)
    residual = pixels-intercept-angles@jacobian.T
    rms = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    return intercept, jacobian, rms


def condition(matrix):
    singular = np.linalg.svd(matrix, compute_uv=False)
    if len(singular) < 2 or singular[-1] <= 1e-12:
        return None, singular
    return float(singular[0]/singular[-1]), singular


def excitation(angles, times):
    centered = angles-angles.mean(axis=0)
    cond, singular = condition(centered)
    std = np.std(angles, axis=0)
    spans = np.ptp(angles, axis=0)
    correlation = float(np.corrcoef(centered.T)[0, 1]) if np.all(std > 1e-12) else None
    rates = np.diff(angles, axis=0)/np.diff(times)[:, None]*1000
    positive = np.sum(rates > GATES['min_axis_rate_deg_s'], axis=0)
    negative = np.sum(rates < -GATES['min_axis_rate_deg_s'], axis=0)
    reasons = []
    for axis, index in (('yaw', 0), ('pitch', 1)):
        if spans[index] < GATES['min_axis_span_deg'] or std[index] < GATES['min_axis_std_deg']:
            reasons.append(axis+' insufficient excitation')
        if min(positive[index], negative[index]) < GATES['min_positive_and_negative_rate_samples']:
            reasons.append(axis+' needs both rotation directions; one-way/constant-rate motion rejected')
    if cond is None or cond > GATES['max_centered_pose_condition']:
        reasons.append('centered pose design ill-conditioned or rank deficient')
    if correlation is None or abs(correlation) > GATES['max_abs_axis_correlation']:
        reasons.append('yaw/pitch axes too correlated to identify a local 2D Jacobian')
    return dict(centered_pose_condition=cond, centered_pose_singular_values=singular.tolist(),
                axis_span_deg=spans.tolist(), axis_std_deg=std.tolist(),
                axis_correlation=correlation, positive_rate_samples=positive.tolist(),
                negative_rate_samples=negative.tolist()), reasons


def estimate(observations, poses, search_ms=1000, step_ms=10):
    if (isinstance(search_ms, bool) or isinstance(step_ms, bool) or
            not np.isfinite(search_ms) or not np.isfinite(step_ms) or
            not 100 <= search_ms <= 3000 or not 1 <= step_ms <= 25 or
            abs(search_ms/step_ms-round(search_ms/step_ms)) > 1e-8):
        raise ValueError('search must be 100..3000ms and divisible by step (1..25ms)')
    t, clock = validate(observations, 'frame_ts_ms')
    p, pose_clock = validate(poses, 'ts_ms')
    if clock != pose_clock:
        raise ValueError('video and pose boot differ')
    tokens = [pose_token(row) for row in poses]
    if any(not token for token in tokens) or len(set(tokens)) != 1:
        raise ValueError('exactly one nonempty capture token required for all pose rows')
    for row in observations:
        token = pose_token(row)
        if token is not None and token != tokens[0]:
            raise ValueError('image and pose capture tokens differ')
    pixels = np.column_stack((numeric(observations, 'x_px'), numeric(observations, 'y_px')))
    yaw_raw, pitch_raw = numeric(poses, 'yaw_deg'), numeric(poses, 'pitch_deg')
    if np.any(abs(yaw_raw) > 360) or np.any(abs(pitch_raw) > 180):
        raise ValueError('pose exceeds physical input gate yaw +/-360deg, pitch +/-180deg')
    roll = numeric(poses, 'roll_deg') if any('roll_deg' in row for row in poses) else None
    if roll is not None and np.any(abs(roll) > 180):
        raise ValueError('roll exceeds physical input gate +/-180deg')
    supported = (t-search_ms >= p[0]) & (t+search_ms <= p[-1])
    original_frame_count = len(t)
    t, pixels = t[supported], pixels[supported]
    if len(t) < GATES['min_frames'] or t[-1]-t[0] < GATES['min_span_ms']:
        raise ValueError('need >=80 jointly supported frames spanning >=8 seconds')
    if np.max(np.diff(t)) > GATES['max_landmark_gap_ms']:
        raise ValueError('landmark gap exceeds 300ms')
    lags = np.arange(-search_ms, search_ms+step_ms/2, step_ms, dtype=float)
    queries = t[None, :]-lags[:, None]
    right = np.searchsorted(p, queries, side='left').clip(0, len(p)-1)
    left = np.maximum(right-1, 0)
    left = np.where(p[right] == queries, right, left)
    gaps = p[right]-p[left]
    if np.any(gaps > GATES['max_pose_support_gap_ms']):
        raise ValueError('pose support gap exceeds 150ms for at least one lag candidate')
    begin, end = int(left.min()), int(right.max())+1
    sp = p[begin:end]
    sy = np.degrees(np.unwrap(np.radians(yaw_raw[begin:end])))
    support_angles = np.column_stack((sy, pitch_raw[begin:end]))
    # Validate EVERY intervening support row, not just values sampled by a
    # favorable candidate. Initialization jumps outside this envelope do not veto.
    if np.any(abs(np.diff(support_angles, axis=0)) > GATES['max_adjacent_pose_jump_deg']):
        raise ValueError('adjacent yaw/pitch jump >30deg in complete query support')
    support_span = np.ptp(support_angles, axis=0)
    reasons = []
    if support_span[0] > GATES['max_support_yaw_span_deg']:
        reasons.append('support yaw span >40deg exceeds local-model gate')
    if support_span[1] > GATES['max_support_pitch_span_deg']:
        reasons.append('support pitch span >30deg exceeds local-model gate')
    roll_info = dict(observed=roll is not None, modeled=False, span_deg=None,
                     interpretation='unobserved roll cannot be ruled out' if roll is None else 'roll is checked but not modeled')
    if roll is not None:
        roll_info['span_deg'] = float(np.ptp(np.degrees(np.unwrap(np.radians(roll[begin:end])))))
        if roll_info['span_deg'] > GATES['max_observed_roll_span_deg']:
            reasons.append('observed roll span >2deg; 2D yaw/pitch model has an unmodeled axis')
    samples = np.array([np.column_stack((np.interp(q, sp, support_angles[:, 0]),
                                         np.interp(q, sp, support_angles[:, 1]))) for q in queries])
    split = len(t)//2
    train = [fit(row[:split], pixels[:split]) for row in samples]
    hold = [fit(row[split:], pixels[split:]) for row in samples]
    train_score = np.array([row[2] for row in train])
    hold_score = np.array([row[2] for row in hold])
    fixed_score = np.array([np.sqrt(np.mean(np.sum((pixels[split:]-a-sample[split:]@j.T)**2, axis=1)))
                            for (a, j, _), sample in zip(train, samples)])
    i, h = int(np.argmin(train_score)), int(np.argmin(hold_score))
    halves = []
    for name, fits, scores, index, section in (('train', train, train_score, i, slice(None, split)),
                                              ('holdout', hold, hold_score, h, slice(split, None))):
        ts, xy, angles = t[section], pixels[section], samples[index, section]
        a, j, rms = fits[index]
        metrics, issues = excitation(angles, ts)
        reasons.extend(name+': '+reason for reason in issues)
        if ts[-1]-ts[0] < GATES['min_half_span_ms']:
            reasons.append(name+': span <3 seconds')
        image_span = float(np.linalg.norm(np.ptp(xy, axis=0)))
        if image_span < GATES['min_image_span_px']:
            reasons.append(name+': insufficient image motion')
        tolerance = max(GATES['profile_tolerance_floor_px'],
                        GATES['profile_tolerance_std_fraction']*float(np.linalg.norm(np.std(xy, axis=0))))
        near = lags[scores <= scores[index]+tolerance]
        width = float(near[-1]-near[0])
        if width > GATES['max_profile_span_ms']:
            reasons.append(name+': phase nonunique (>100ms near-optimum span)')
        if index in (0, len(lags)-1):
            reasons.append(name+': optimum at search boundary')
        rms_limit = max(GATES['residual_floor_px'], GATES['residual_span_fraction']*image_span)
        if rms > rms_limit:
            reasons.append(name+': 2D pixel residual exceeds fixed model-quality gate')
        jcond, jsv = condition(j)
        if jcond is None or jcond > GATES['max_jacobian_condition'] or jsv[-1] < GATES['min_jacobian_singular_value_px_per_deg']:
            reasons.append(name+': image Jacobian degenerate or too weak')
        halves.append(dict(name=name, samples=len(ts), best_lag_ms=float(lags[index]),
            intercept_px=a.tolist(), jacobian_px_per_deg=j.tolist(), independent_rms_px=rms,
            rms_limit_px=rms_limit, profile_tolerance_px=tolerance, near_optimum_span_ms=width,
            excitation=metrics, jacobian_condition=jcond, jacobian_singular_values=jsv.tolist()))
    if abs(lags[i]-lags[h]) > GATES['max_train_holdout_lag_difference_ms']:
        reasons.append('train/holdout lag disagreement >50ms')
    a0, j0, _ = train[i]
    ah, jh, _ = hold[h]
    relative = float(np.linalg.norm(jh-j0)/max(float(np.linalg.norm(j0)), 1e-12))
    if relative > GATES['max_relative_jacobian_change']:
        reasons.append('train/holdout Jacobian relative change >25%')
    column_checks = []
    for axis in range(2):
        length0, lengthh = float(np.linalg.norm(j0[:, axis])), float(np.linalg.norm(jh[:, axis]))
        ratio = lengthh/length0 if length0 > 1e-12 else None
        cosine = float(j0[:, axis]@jh[:, axis]/(length0*lengthh)) if min(length0, lengthh) > 1e-12 else None
        column_checks.append(dict(axis=('yaw', 'pitch')[axis], norm_ratio=ratio, cosine=cosine))
        if (ratio is None or cosine is None or not GATES['min_jacobian_column_scale_ratio'] <= ratio <= GATES['max_jacobian_column_scale_ratio']
                or cosine < GATES['min_jacobian_column_cosine']):
            reasons.append('train/holdout Jacobian column direction or scale unstable')
    if fixed_score[i] > halves[1]['rms_limit_px']:
        reasons.append('held-out residual with FIXED training intercept/J exceeds gate')
    fixed_lag_metrics, fixed_lag_issues = excitation(samples[i, split:], t[split:])
    reasons.extend('holdout at fixed training lag: '+reason for reason in fixed_lag_issues)
    return dict(schema_version=1, status='candidate' if not reasons else 'rejected', **FLAGS,
        model='local pixel intercept + 2x2 empirical Jacobian * [unwrapped yaw,pitch](frame_ts-lag)',
        clock_id=clock, capture_token=tokens[0], reasons=list(dict.fromkeys(reasons)),
        relative_lag_ms=float(lags[i]), holdout_lag_ms=float(lags[h]),
        fixed_training_model_holdout_rms_px=float(fixed_score[i]), halves=halves,
        relative_jacobian_change=relative, jacobian_column_checks=column_checks,
        fixed_training_lag_holdout_excitation=fixed_lag_metrics, roll=roll_info,
        samples=len(t), excluded_edge_frames=original_frame_count-len(t),
        valid_frame_range_ms=[float(t[0]), float(t[-1])], split_frame_ts_ms=float(t[split]),
        pose_support_range_ms=[float(sp[0]), float(sp[-1])], pose_support_rows=len(sp),
        excluded_outside_support_pose_rows=len(p)-len(sp), pose_support_span_deg=support_span.tolist(),
        maximum_query_support_gap_ms=float(gaps.max()), pose_interval_median_ms=float(np.median(np.diff(sp))),
        scan_step_ms=float(step_ms), timestamp_relation='pose_log_time = decoder_frame_time - relative_lag_ms',
        thresholds=GATES, limits=LIMITS,
        profile=[dict(lag_ms=float(lag), train_rms_px=float(a), independent_holdout_rms_px=float(b),
                      fixed_training_model_holdout_rms_px=float(c)) for lag, a, b, c in zip(lags, train_score, hold_score, fixed_score)])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observations', type=Path, required=True, help='frame_ts_ms,x_px,y_px,clock_id')
    parser.add_argument('--poses', type=Path, required=True, help='ts_ms,yaw_deg,pitch_deg,clock_id,session_token; optional roll_deg')
    parser.add_argument('--token', required=True, help='exact pose session_token (or token)')
    parser.add_argument('--search-ms', type=float, default=1000)
    parser.add_argument('--step-ms', type=float, default=10)
    parser.add_argument('--output', type=Path, required=True, help='new diagnostic JSON, never a control configuration')
    args = parser.parse_args(argv)
    if args.output.exists() or args.output.resolve() in (args.poses.resolve(), args.observations.resolve()):
        parser.error('output must be a new path distinct from input files')
    sources = {}
    try:
        for name, path in (('observations', args.observations), ('poses', args.poses)):
            sources[name] = dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        obs, poses = read_csv(args.observations), read_csv(args.poses)
        selected = [row for row in poses if pose_token(row) == args.token]
        report = estimate(obs, selected, args.search_ms, args.step_ms)
        report['excluded_pose_token_rows'] = len(poses)-len(selected)
    except (ValueError, TypeError, KeyError, OSError, UnicodeError, csv.Error, np.linalg.LinAlgError) as exc:
        report = dict(schema_version=1, status='rejected', reasons=[str(exc)], **FLAGS,
                      thresholds=GATES, limits=LIMITS)
    report['sources'] = sources
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'profile'}, ensure_ascii=False, indent=2))
    return 0 if report['status'] == 'candidate' else 2


if __name__ == '__main__':
    raise SystemExit(main())
