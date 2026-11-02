"""Extract a fixed-background pixel trajectory; no focal length or pose is used.

Track original ROI corners using sequential LK forward/backward checks, never
reseed. Only features surviving the ENTIRE requested interval contribute to
every output row. A fixed initial anchor plus median displacement avoids moving
the reported landmark merely because some points disappear. Optional planar
tracking instead projects that same fixed anchor through a homography fitted
from the original common feature coordinates to each frame.
"""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path

import cv2
import numpy as np


FIELDS = ('frame_ts_ms', 'x_px', 'y_px', 'clock_id', 'video_frame', 'tracked_points')
PLANAR_GATES = dict(ransac_reprojection_px=1.5, min_inlier_fraction=.85,
    min_inlier_points=12, inlier_residual_p95_px=1.25,
    min_spread_eigenvalue_ratio=.04, min_axis_span_px=10.0,
    min_hull_bbox_area_fraction=.2, min_anchor_hull_margin_px=2.0,
    max_normalized_h_condition=1000.0, min_denominator_ratio=.2,
    min_anchor_jacobian_scale=.25, max_anchor_jacobian_scale=4.0,
    min_spatial_subset_points=6, max_spatial_anchor_disagreement_px=.75)
LIMITS = [
    'Pixel trajectory only; no focal length, angular conversion, pose or lag selection is used.',
    'Operator must choose a static textured background ROI and start/end before looking at lag fits.',
    'Same original feature IDs contribute to every output row; lost features are never replaced.',
    'Forward/backward agreement does not prove correct identity in repetitive texture or occlusion.',
    'Large deformation, parallax, blur or occlusion causes rejection; do not automatically search another ROI until lag looks good.',
    'Planar mode requires one static textured plane; it is not a general 3D or moving-object motion model.',
    'Planar mode uses fixed engineering RANSAC, reprojection, spatial-spread and projective-geometry gates; it does not relax LK checks.',
    'Planar P95 residual <=1.25px is supported by synthetic JPEG/noise/blur tests; the RANSAC and maximum residual limits remain 1.5px.',
    'Spatial half-plane anchor disagreement <=0.75px is a model-stability diagnostic, NOT an absolute-error confidence interval.',
    'A coherent wrong correspondence across the entire ROI can pass geometry checks; landmark identity remains unverified.',
    'Timestamps are copied from frames.csv, not recovered exposure timestamps; this does not calibrate camera timing.',
]


class TrackingRejected(ValueError):
    def __init__(self, reason, report):
        super().__init__(reason)
        self.report = dict(report, status='rejected', reasons=[reason])


def read_frames(path):
    raw = Path(path).read_bytes()
    if b'\x00' in raw:
        raise ValueError('frames.csv contains zero bytes; use a separately audited clean subset')
    rows = list(csv.DictReader(io.StringIO(raw.decode('utf-8-sig'))))
    if not rows:
        raise ValueError('empty frames.csv')
    clocks = set()
    last = -1
    for index, row in enumerate(rows):
        if None in row or any(value is None for value in row.values()):
            raise ValueError('malformed frame CSV row')
        for key in ('video_frame', 'frame_ts_ms'):
            value = row.get(key, '')
            if not value.isascii() or not value.isdigit():
                raise ValueError('invalid frame '+key)
            row[key] = int(value)
        clock = row.get('clock_id', '')
        if not clock.startswith('mono-v1:') or len(clock) <= 8:
            raise ValueError('missing identified frame clock')
        clocks.add(clock)
        if row['video_frame'] != index:
            raise ValueError('video_frame must be contiguous and start at zero')
        if row['frame_ts_ms'] <= last or row['frame_ts_ms'] > 2**53:
            raise ValueError('invalid or nonmonotonic frame timestamp')
        last = row['frame_ts_ms']
    if len(clocks) != 1:
        raise ValueError('mixed frame boot clocks')
    return rows


def fingerprint(path):
    path = Path(path).resolve(strict=True)
    before = path.stat()
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError('source changed while hashing: '+str(path))
    return dict(path=str(path), bytes=after.st_size, mtime_ns=after.st_mtime_ns,
                sha256=digest.hexdigest())


def fixed_survivor_trace(history, original, survivor_ids):
    """All frames use one final ID set, not a frame-varying survivor centroid."""
    positions = np.asarray(history, dtype=float)[:, survivor_ids, :]
    base = np.asarray(original, dtype=float)[survivor_ids]
    if not np.all(np.isfinite(positions)) or len(base) == 0:
        raise ValueError('invalid common feature history')
    anchor = np.median(base, axis=0)
    displacement = positions-base[None, :, :]
    median = np.median(displacement, axis=1)
    residual = np.linalg.norm(displacement-median[:, None, :], axis=2)
    steps = np.diff(positions, axis=0)
    step_median = np.median(steps, axis=1)
    step_residual = np.linalg.norm(steps-step_median[:, None, :], axis=2)
    return anchor+median, dict(anchor_px=anchor.tolist(),
        displacement_residual_p90_max_px=float(np.percentile(residual, 90, axis=1).max()),
        step_residual_p90_max_px=float(np.percentile(step_residual, 90, axis=1).max()) if len(steps) else 0.0)


def _planar_spread(points):
    """Reject line-like or tiny feature support before an H can look precise."""
    points = np.asarray(points, dtype=float)
    centered = points-points.mean(axis=0)
    eigenvalues = np.linalg.eigvalsh(centered.T@centered/len(points))
    ratio = float(eigenvalues[0]/eigenvalues[-1]) if eigenvalues[-1] > 1e-12 else 0.0
    spans = np.ptp(points, axis=0)
    hull = cv2.convexHull(points.astype(np.float32))
    area = float(cv2.contourArea(hull))
    bbox_area = float(np.prod(spans))
    fraction = area/bbox_area if bbox_area > 0 else 0.0
    quality = dict(eigenvalue_ratio=ratio, axis_spans_px=spans.tolist(),
                   hull_bbox_area_fraction=fraction)
    if (ratio < PLANAR_GATES['min_spread_eigenvalue_ratio'] or
            spans.min() < PLANAR_GATES['min_axis_span_px'] or
            fraction < PLANAR_GATES['min_hull_bbox_area_fraction']):
        raise ValueError('degenerate two-dimensional feature distribution')
    return quality, hull


def _project_homography(h, points):
    coordinates = np.column_stack((points, np.ones(len(points))))@h.T
    if not np.isfinite(coordinates).all() or np.any(abs(coordinates[:, 2]) < 1e-12):
        raise ValueError('nonfinite or singular projective coordinates')
    return coordinates[:, :2]/coordinates[:, 2, None], coordinates[:, 2]


def _spatial_anchor_quality(base, current, anchor, full_anchor):
    """Check four spatial refits at the SAME anchor, without selecting one.

    A small whole-ROI residual can hide two parts with incompatible motion.
    Left/right and top/bottom refits must predict mutually consistent anchor
    positions. This does not detect a coherent wrong match of the entire ROI.
    """
    anchors = [full_anchor]
    subsets = []
    for axis, axis_name in enumerate(('x', 'y')):
        for positive in (False, True):
            take = base[:, axis] >= anchor[axis] if positive else base[:, axis] < anchor[axis]
            count = int(take.sum())
            if count < PLANAR_GATES['min_spatial_subset_points']:
                raise ValueError('insufficient spatial subset support for anchor stability')
            _planar_spread(base[take])
            _planar_spread(current[take])
            # Use all already accepted inliers in each spatial subset; do not
            # cherry-pick a new RANSAC membership or use these refits as output.
            h, _ = cv2.findHomography(base[take], current[take], 0)
            if h is None or not np.isfinite(h).all():
                raise ValueError('spatial subset homography unavailable')
            projected, denominators = _project_homography(h, np.vstack((base[take], anchor)))
            if (not (np.all(denominators > 0) or np.all(denominators < 0)) or
                    abs(denominators).min()/abs(denominators).max() < PLANAR_GATES['min_denominator_ratio']):
                raise ValueError('spatial subset anchor approaches a projective pole')
            estimate = projected[-1]
            anchors.append(estimate)
            subsets.append(dict(axis=axis_name, side='positive' if positive else 'negative',
                points=count, anchor_xy_px=estimate.tolist()))
    anchors = np.asarray(anchors)
    disagreement = float(np.linalg.norm(anchors[:, None, :]-anchors[None, :, :], axis=2).max())
    return dict(spatial_anchor_disagreement_px=disagreement, spatial_anchor_subsets=subsets)


def planar_survivor_trace(history, original, survivor_ids):
    """Project a fixed anchor; input feature IDs never change between frames.

    RANSAC may reject an inconsistent correspondence but cannot introduce or
    reseed a feature. Per-frame inlier IDs and quality remain in the report.
    Thresholds are fixed geometric diagnostics, not confidence probabilities.
    """
    positions = np.asarray(history, dtype=float)[:, survivor_ids, :]
    base = np.asarray(original, dtype=float)[survivor_ids]
    if (len(base) < PLANAR_GATES['min_inlier_points'] or
            not np.isfinite(base).all() or not np.isfinite(positions).all()):
        raise ValueError('invalid or insufficient common planar feature history')
    anchor = np.median(base, axis=0)
    quality = dict(anchor_px=anchor.tolist(), planar_frame_quality=[])
    trace = []
    for offset, current in enumerate(positions):
        frame_quality = dict(frame_offset=offset, motion_model='planar',
                             input_points=len(base))
        quality['planar_frame_quality'].append(frame_quality)
        try:
            _planar_spread(base)
            _planar_spread(current)
            h, mask = cv2.findHomography(base, current, cv2.RANSAC,
                PLANAR_GATES['ransac_reprojection_px'], maxIters=5000, confidence=.999)
            if h is None or mask is None or h.shape != (3, 3) or not np.isfinite(h).all():
                raise ValueError('homography fit unavailable or nonfinite')
            inliers = mask.ravel().astype(bool)
            count = int(inliers.sum())
            frame_quality.update(inlier_points=count, inlier_fraction=count/len(base),
                inlier_original_feature_ids=np.asarray(survivor_ids)[inliers].tolist())
            if count < PLANAR_GATES['min_inlier_points'] or count/len(base) < PLANAR_GATES['min_inlier_fraction']:
                raise ValueError('insufficient planar RANSAC inliers')
            source_spread, source_hull = _planar_spread(base[inliers])
            target_spread, target_hull = _planar_spread(current[inliers])
            frame_quality.update(source_inlier_spread=source_spread,
                                 target_inlier_spread=target_spread)
            anchor_margin = cv2.pointPolygonTest(source_hull, tuple(map(float, anchor)), True)
            if anchor_margin < PLANAR_GATES['min_anchor_hull_margin_px']:
                raise ValueError('anchor lacks two-dimensional inlier support')
            # Coordinate normalization makes the condition test independent of
            # image origin and avoids penalizing ordinary pixel translations.
            transforms = []
            for points in (base[inliers], current[inliers]):
                center = points.mean(axis=0)
                scale = float(np.sqrt(np.mean(np.sum((points-center)**2, axis=1))))
                transforms.append(np.array([[1/scale, 0, -center[0]/scale],
                    [0, 1/scale, -center[1]/scale], [0, 0, 1]], dtype=float))
            normalized_h = transforms[1]@h@np.linalg.inv(transforms[0])
            condition = float(np.linalg.cond(normalized_h))
            if not np.isfinite(condition) or condition > PLANAR_GATES['max_normalized_h_condition']:
                raise ValueError('ill-conditioned homography')
            projected, denominators = _project_homography(h, np.vstack((base[inliers], anchor)))
            if (not (np.all(denominators > 0) or np.all(denominators < 0)) or
                    abs(denominators).min()/abs(denominators).max() < PLANAR_GATES['min_denominator_ratio']):
                raise ValueError('homography crosses or approaches a projective pole')
            target_anchor = projected[-1]
            if cv2.pointPolygonTest(target_hull, tuple(map(float, target_anchor)), True) < 0:
                raise ValueError('projected anchor outside target inlier support')
            residual = np.linalg.norm(projected[:-1]-current[inliers], axis=1)
            residual_p95 = float(np.percentile(residual, 95))
            frame_quality.update(inlier_reprojection_p95_px=residual_p95,
                inlier_reprojection_max_px=float(residual.max()), normalized_h_condition=condition)
            if (residual_p95 > PLANAR_GATES['inlier_residual_p95_px'] or
                    residual.max() > PLANAR_GATES['ransac_reprojection_px']):
                raise ValueError('planar reprojection residual exceeds gate')
            denominator = denominators[-1]
            jacobian = (h[:2, :2]-np.outer(target_anchor, h[2, :2]))/denominator
            singular = np.linalg.svd(jacobian, compute_uv=False)
            if (np.linalg.det(jacobian) <= 0 or
                    singular.min() < PLANAR_GATES['min_anchor_jacobian_scale'] or
                    singular.max() > PLANAR_GATES['max_anchor_jacobian_scale']):
                raise ValueError('homography anchor mapping is reflected or excessively distorted')
            stability = _spatial_anchor_quality(base[inliers], current[inliers], anchor, target_anchor)
            frame_quality.update(stability)
            if stability['spatial_anchor_disagreement_px'] > PLANAR_GATES['max_spatial_anchor_disagreement_px']:
                raise ValueError('spatial anchor disagreement exceeds gate')
            frame_quality.update(anchor_xy_px=target_anchor.tolist(),
                anchor_jacobian_scales=singular.tolist(), homography=h.tolist(), status='accepted')
            trace.append(target_anchor)
        except (ValueError, np.linalg.LinAlgError, cv2.error) as exc:
            frame_quality.update(status='rejected', reason=str(exc))
            raise TrackingRejected('invalid planar geometry at frame offset '+str(offset)+': '+str(exc), quality) from exc
    quality['planar_inlier_reprojection_p95_max_px'] = max(
        row['inlier_reprojection_p95_px'] for row in quality['planar_frame_quality'])
    quality['planar_spatial_anchor_disagreement_max_px'] = max(
        row['spatial_anchor_disagreement_px'] for row in quality['planar_frame_quality'])
    return np.asarray(trace), quality


def track(session, roi, start=0, end=None, *, video=None, min_points=12,
          max_points=100, fb_threshold_px=.75, min_survival_fraction=.5,
          max_step_residual_px=2.5, max_displacement_residual_px=6.0,
          motion_model='translation'):
    """Return (CSV rows, quality report) or TrackingRejected without partial CSV.

    start is inclusive, end exclusive, both refer to original video_frame IDs.
    The video is always read sequentially from frame zero. Thresholds are fixed
    engineering quality gates, not a tracking confidence probability.
    """
    session = Path(session).resolve()
    if motion_model not in ('translation', 'planar'):
        raise ValueError('motion_model must be translation or planar')
    frame_source = fingerprint(session/'frames.csv')
    rows = read_frames(session/'frames.csv')
    end = len(rows) if end is None else end
    if any(type(v) is not int for v in (start, end, min_points, max_points)) or not 0 <= start < end <= len(rows):
        raise ValueError('invalid inclusive start/exclusive end')
    if end-start < 2 or min_points < 12 or not min_points <= max_points <= 500:
        raise ValueError('need >=2 frames, >=12 minimum features and <=500 initial features')
    if len(roi) != 4 or any(type(v) is not int for v in roi) or min(roi[:2]) < 0 or min(roi[2:]) < 8:
        raise ValueError('ROI must be integer x,y,width,height with positive size')
    if (not all(np.isfinite(v) and v > 0 for v in (fb_threshold_px, max_step_residual_px, max_displacement_residual_px))
            or not 0 < min_survival_fraction <= 1):
        raise ValueError('invalid tracking quality thresholds')
    if video is None:
        candidates = [session/name for name in ('liveview.mkv', 'liveview.avi') if (session/name).exists()]
        if len(candidates) != 1:
            raise ValueError('need one liveview.mkv/avi or explicit --video')
        video = candidates[0]
    sources = dict(frames=frame_source, video=fingerprint(video))
    report = dict(schema_version=1, status='tracking', shadow_only=True, verified=False,
        exposure_calibrated=False, sources=sources, clock_id=rows[0]['clock_id'],
        roi=list(roi), start_frame=start, end_frame_exclusive=end, indexed_frames=len(rows),
        thresholds=dict(min_points=min_points, max_points=max_points, fb_threshold_px=fb_threshold_px,
            min_survival_fraction=min_survival_fraction, max_step_residual_px=max_step_residual_px,
            max_displacement_residual_px=max_displacement_residual_px, max_lk_patch_error=30),
        limits=LIMITS, cv2_version=cv2.__version__, frame_quality=[], motion_model=motion_model)
    if motion_model == 'planar':
        report['thresholds']['planar'] = dict(PLANAR_GATES)
    report['translation_residual_gates_applied'] = motion_model == 'translation'
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        cap.release()
        raise TrackingRejected('video could not be opened', report)
    old = points = original = ids = shape = None
    history = []
    lk_options = dict(winSize=(21, 21), maxLevel=3,
                      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
    try:
        for index in range(end):
            ok, frame = cap.read()
            if not ok:
                raise TrackingRejected('video ended before indexed frame '+str(index), report)
            if index < start:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if old is None:
                shape = gray.shape
                x, y, width, height = roi
                if x+width > shape[1] or y+height > shape[0]:
                    raise TrackingRejected('ROI outside initial image', report)
                mask = np.zeros_like(gray)
                mask[y:y+height, x:x+width] = 255
                points = cv2.goodFeaturesToTrack(gray, max_points, .02, 5, mask=mask)
                if points is None or len(points) < min_points:
                    raise TrackingRejected('insufficient initial textured background features', report)
                original = points[:, 0, :].copy()
                ids = np.arange(len(points))
                report['initial_points'] = len(ids)
                report['image_size_px'] = [shape[1], shape[0]]
                quality = dict(video_frame=index, active_points=len(ids))
            else:
                if gray.shape != shape:
                    raise TrackingRejected('video resolution changed', report)
                nxt, good, error = cv2.calcOpticalFlowPyrLK(old, gray, points, None, **lk_options)
                if nxt is None or good is None or error is None:
                    raise TrackingRejected('forward optical flow unavailable at frame '+str(index), report)
                back, reverse, _ = cv2.calcOpticalFlowPyrLK(gray, old, nxt, None, **lk_options)
                if back is None or reverse is None:
                    raise TrackingRejected('backward optical flow unavailable at frame '+str(index), report)
                fb = np.linalg.norm(back-points, axis=2).ravel()
                valid = (good.ravel() == 1) & (reverse.ravel() == 1)
                valid &= np.isfinite(nxt).all(axis=(1, 2)) & np.isfinite(back).all(axis=(1, 2))
                valid &= np.isfinite(error.ravel()) & (error.ravel() <= 30) & (fb <= fb_threshold_px)
                valid &= (nxt[:, 0, 0] >= 0) & (nxt[:, 0, 0] < shape[1])
                valid &= (nxt[:, 0, 1] >= 0) & (nxt[:, 0, 1] < shape[0])
                quality = dict(video_frame=index, attempted_points=len(ids), retained_points=int(valid.sum()),
                    retained_fb_p95_px=float(np.percentile(fb[valid], 95)) if valid.any() else None)
                report['frame_quality'].append(quality)
                ids, points = ids[valid], nxt[valid]
                if len(ids) < min_points or len(ids)/len(original) < min_survival_fraction:
                    raise TrackingRejected('too few original features survive at frame '+str(index), report)
            if old is None:
                report['frame_quality'].append(quality)
            state = np.full_like(original, np.nan)
            state[ids] = points[:, 0, :]
            history.append(state)
            old = gray
    except cv2.error as exc:
        raise TrackingRejected('OpenCV tracking/decoding error: '+str(exc), report) from exc
    finally:
        cap.release()
    try:
        trace, quality = (planar_survivor_trace(history, original, ids) if motion_model == 'planar'
                          else fixed_survivor_trace(history, original, ids))
    except TrackingRejected as exc:
        report.update(exc.report)
        for item in report.get('planar_frame_quality', []):
            item['video_frame'] = start+item['frame_offset']
        raise TrackingRejected(str(exc), report) from exc
    for item in quality.get('planar_frame_quality', []):
        item['video_frame'] = start+item['frame_offset']
    report.update(quality, common_points=len(ids), survivor_fraction=len(ids)/len(original),
                  common_original_feature_ids=ids.tolist(), initial_feature_xy_px=original.tolist(),
                  output_frames=len(trace), sequentially_decoded_frames=end)
    if motion_model == 'translation' and quality['step_residual_p90_max_px'] > max_step_residual_px:
        raise TrackingRejected('inconsistent feature motion between frames', report)
    if motion_model == 'translation' and quality['displacement_residual_p90_max_px'] > max_displacement_residual_px:
        raise TrackingRejected('accumulated feature motion is not coherent within the ROI', report)
    for source in sources.values():
        stat = Path(source['path']).stat()
        if (stat.st_size, stat.st_mtime_ns) != (source['bytes'], source['mtime_ns']):
            raise TrackingRejected('source changed during tracking', report)
    output = [dict(frame_ts_ms=row['frame_ts_ms'], x_px=float(x), y_px=float(y),
                   clock_id=row['clock_id'], video_frame=row['video_frame'], tracked_points=len(ids))
              for row, (x, y) in zip(rows[start:end], trace)]
    report.update(status='trajectory_extracted', reasons=[],
                  frame_range_ms=[output[0]['frame_ts_ms'], output[-1]['frame_ts_ms']],
                  coordinate_definition=('fixed median original common-feature anchor projected by per-frame original-to-current planar homography; same input IDs for every frame'
                    if motion_model == 'planar' else
                    'median original common-feature anchor + median per-original-feature displacement; same IDs for every frame'))
    return output, report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--video', type=Path, help='explicit matching video if session has ambiguous filenames')
    parser.add_argument('--roi', nargs=4, type=int, required=True, metavar=('X', 'Y', 'W', 'H'))
    parser.add_argument('--start-frame', type=int, default=0)
    parser.add_argument('--end-frame', type=int, help='exclusive original video_frame index')
    parser.add_argument('--motion-model', choices=('translation', 'planar'), default='translation',
                        help='translation by default; planar requires one static textured plane')
    parser.add_argument('--output', type=Path, required=True, help='new pixel trajectory CSV')
    parser.add_argument('--report', type=Path, help='new quality JSON; default <output>.quality.json')
    args = parser.parse_args(argv)
    report_path = args.report or args.output.with_suffix('.quality.json')
    if args.output.resolve() == report_path.resolve() or args.output.exists() or report_path.exists():
        parser.error('CSV and report must be distinct new paths; existing files are never overwritten')
    output = None
    try:
        output, report = track(args.session, args.roi, args.start_frame, args.end_frame,
                               video=args.video, motion_model=args.motion_model)
    except TrackingRejected as exc:
        report = exc.report
    except (ValueError, OSError, UnicodeError, csv.Error) as exc:
        report = dict(schema_version=1, status='rejected', reasons=[str(exc)], shadow_only=True,
                      verified=False, exposure_calibrated=False, limits=LIMITS,
                      requested_session=str(args.session.resolve()), roi=args.roi,
                      start_frame=args.start_frame, end_frame_exclusive=args.end_frame,
                      motion_model=args.motion_model)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    with report_path.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    if output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            writer.writerows(output)
    print(json.dumps({k: v for k, v in report.items() if k not in ('frame_quality', 'planar_frame_quality', 'initial_feature_xy_px')},
                     ensure_ascii=False, indent=2))
    return 0 if report['status'] == 'trajectory_extracted' else 2


if __name__ == '__main__':
    raise SystemExit(main())
