"""Create a separate real-time MKV replay from AVI/MKV and frames.csv.

Never infer timestamps for missing index rows. Extra video tail is counted and
excluded from timed output. Reject mixed clocks, missing frames and time reversal.
"""
import argparse
import csv
import io
import json
from pathlib import Path
import cv2
from timestamp_video import TimestampVideoWriter


def replay(session, output=None):
    session = Path(session)
    output = Path(output) if output else session / 'replay_realtime.mkv'
    source = session / 'liveview.mkv'
    if not source.exists():
        source = session / 'liveview.avi'
    if output.resolve() == source.resolve() or output.exists():
        raise ValueError('refusing to overwrite input or existing replay')
    raw = (session/'frames.csv').read_text(encoding='utf-8-sig')
    text = raw.rstrip('\x00')
    zero_tail_bytes = len(raw)-len(text)
    rows = list(csv.DictReader(io.StringIO(text)))
    incomplete_tail_rows = 0
    if rows and any(rows[-1].get(key) is None for key in ('video_frame', 'frame_ts_ms', 'clock_id')):
        rows.pop()
        incomplete_tail_rows = 1
    if not rows:
        raise ValueError('empty frame index')
    stamps = [int(row['frame_ts_ms']) for row in rows]
    clocks = {row.get('clock_id', '') for row in rows}
    if len(clocks) != 1 or '' in clocks:
        raise ValueError('missing or mixed boot clock identities')
    if any(int(row['video_frame']) != i for i, row in enumerate(rows)):
        raise ValueError('noncontiguous video frame index')
    if stamps[0] < 0 or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError('nonmonotonic frame index')
    output.parent.mkdir(parents=True, exist_ok=True)
    partial = output.with_suffix('.partial.mkv')
    capture = cv2.VideoCapture(str(source))
    writer = None
    try:
        for i, stamp in enumerate(stamps):
            ok, frame = capture.read()
            if not ok:
                raise ValueError(f'video ended before indexed frame {i}')
            if writer is None:
                writer = TimestampVideoWriter(partial, frame.shape[1], frame.shape[0])
            cv2.putText(frame, f'T+{(stamp-stamps[0])/1000:.3f}s  frame={i}', (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 2)
            writer.write(frame, stamp)
        extra = 0
        while capture.read()[0]:
            extra += 1
        writer.release()
        partial.replace(output)
    except Exception:
        if writer:
            writer.release()
        partial.unlink(missing_ok=True)
        raise
    finally:
        capture.release()
    report = dict(source=str(source), output=str(output), clock_id=next(iter(clocks)),
                  indexed_frames=len(rows), excluded_unindexed_tail_frames=extra,
                  excluded_csv_zero_tail_bytes=zero_tail_bytes,
                  excluded_incomplete_csv_tail_rows=incomplete_tail_rows,
                  first_frame_ts_ms=stamps[0], last_frame_ts_ms=stamps[-1],
                  observed_span_ms=stamps[-1]-stamps[0], final_frame_duration_ms=1,
                  timestamp_meaning='decoder frame timestamp, not sensor exposure time')
    output.with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    print(json.dumps(replay(args.session, args.output), ensure_ascii=False))
