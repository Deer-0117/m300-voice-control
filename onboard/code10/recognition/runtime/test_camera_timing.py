import unittest
import math
from calibrate_camera_timing import estimate, validate, audit_raw_timing, track


def data(lag=180, motion=None):
    motion = motion or (lambda t: 12*math.sin(t*1.7)+4*math.sin(t*3.1))
    poses = [dict(ts_ms=t, yaw_deg=motion(t/1000), clock_id='mono-v1:test')
             for t in range(0, 24001, 20)]
    obs = [dict(frame_ts_ms=t, bearing_deg=-motion((t-lag)/1000),
                clock_id='mono-v1:test') for t in range(1500, 22000, 50)]
    return obs, poses


class TimingTests(unittest.TestCase):
    def test_known_delay_and_heldout(self):
        report = estimate(*data())
        self.assertEqual(report['status'], 'candidate')
        self.assertAlmostEqual(report['relative_lag_ms'], 180, delta=5)
        self.assertLess(report['holdout_rms_deg'], .01)
        self.assertFalse(report['verified'])
        self.assertFalse(report['exposure_calibrated'])

    def test_negative_delay(self):
        self.assertEqual(estimate(*data(-120))['relative_lag_ms'], -120)

    def test_constant_speed_is_unidentifiable(self):
        report = estimate(*data(motion=lambda t: 5*t))
        self.assertEqual(report['status'], 'rejected')
        self.assertTrue(any('ambiguous' in r for r in report['reasons']))

    def test_stationary_camera_rejected(self):
        self.assertEqual(estimate(*data(motion=lambda t: 4))['status'], 'rejected')

    def test_variable_latency_rejected(self):
        obs, poses = data()
        other, _ = data(330)
        obs[len(obs)//2:] = other[len(obs)//2:]
        self.assertEqual(estimate(obs,poses)['status'], 'rejected')

    def test_cross_boot_nonfinite_and_backwards(self):
        obs, poses = data()
        obs[0]['clock_id'] = 'mono-v1:other'
        with self.assertRaises(ValueError): estimate(obs, poses)
        obs, poses = data(); obs[0]['bearing_deg'] = float('nan')
        with self.assertRaises(ValueError): estimate(obs, poses)
        with self.assertRaises(ValueError): validate(poses[::-1], 'ts_ms')

    def test_pose_gap_rejected(self):
        obs, poses = data()
        poses = [r for r in poses if not 5000 < r['ts_ms'] < 5500]
        with self.assertRaises(ValueError): estimate(obs, poses)

    def test_yaw_wrap(self):
        obs, poses = data()
        for p in poses: p['yaw_deg'] = (p['yaw_deg']+179+180)%360-180
        self.assertEqual(estimate(obs,poses)['relative_lag_ms'], 180)

    def test_boundary_rejected(self):
        self.assertEqual(estimate(*data(1100))['status'], 'rejected')

    def test_raw_fields_not_interpreted_as_clock(self):
        rows = [dict(host_read_begin_ms=t,host_read_end_ms=t+2,
                     sdk_millisecond_raw=10,sdk_microsecond_raw=999-t,clock_id='mono-v1:test')
                for t in range(100,900,100)]
        report = audit_raw_timing(rows,'mono-v1:test',[100,900])
        self.assertFalse(report['clock_mapping_verified'])
        self.assertEqual(report['sdk_millisecond_raw']['repeats'],7)
        self.assertEqual(report['sdk_microsecond_raw']['decreases'],7)

    def test_sequential_video_tracking(self):
        import tempfile
        import csv
        from pathlib import Path
        import numpy as np
        import cv2
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as directory:
            root=Path(directory)
            rng=np.random.default_rng(4)
            image=rng.integers(0,256,(120,200,3),dtype=np.uint8)
            video=cv2.VideoWriter(str(root/'liveview.avi'),cv2.VideoWriter_fourcc(*'MJPG'),20,(200,120))
            self.assertTrue(video.isOpened())
            with (root/'frames.csv').open('w',newline='') as f:
                writer=csv.writer(f);writer.writerow(['video_frame','frame_ts_ms','clock_id'])
                for i in range(25):
                    frame=cv2.warpAffine(image,np.float32([[1,0,i*.3],[0,1,0]]),(200,120))
                    video.write(frame);writer.writerow([i,1000+i*50,'mono-v1:test'])
            video.release()
            rows=track(root,[60,30,60,60],200)
            self.assertEqual(len(rows),25)
            self.assertGreater(rows[-1]['bearing_deg'],1.5)
            self.assertLess(rows[-1]['bearing_deg'],2.5)


class CaptureTests(unittest.TestCase):
    def test_only_own_request_removed(self):
        import tempfile
        from pathlib import Path
        from capture_camera_timing import remove_owned
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as d:
            p=Path(d)/'request';p.write_text('someone-else')
            remove_owned(p,'mine');self.assertTrue(p.exists())
            remove_owned(p,'someone-else');self.assertFalse(p.exists())

    def test_record_and_pose_ready_then_flushed(self):
        output=self.run_capture()
        self.assertIn('VIDEO_AND_POSE_READY',output)
        self.assertIn('CAPTURE_SAVED',output)

    def test_impossible_camera_angle_never_confirms_capture(self):
        output=self.run_capture(mode='invalid_pose_angle',error=ValueError,reason='invalid camera angles')
        self.assertNotIn('VIDEO_AND_POSE_READY',output)

    def test_stale_sdk_buffer_cannot_be_refreshed_by_new_host_timestamp(self):
        output=self.run_capture(mode='frozen_sdk_stamp',error=ValueError,reason='SDK sample timestamp not advancing')
        self.assertNotIn('VIDEO_AND_POSE_READY',output)

    def test_immediate_stop_receipt_is_accepted(self):
        output=self.run_capture(mode='immediate_stop')
        self.assertIn('CAPTURE_SAVED',output)

    def test_no_pose_times_out_and_cleans_requests(self):
        output=self.run_capture(produce_pose=False,error=ValueError,reason='pose=pose log unavailable')
        self.assertIn('video=ready (frames advancing)',output)
        self.assertNotIn('VIDEO_AND_POSE_READY',output)

    def test_wrong_boot_ack_does_not_confirm_video(self):
        self.run_capture(mode='wrong_boot',error=ValueError,reason='video=recorder boot mismatch')

    def test_future_ack_does_not_confirm_video(self):
        self.run_capture(mode='future_stamp',error=ValueError,reason='video=invalid recorder timestamp')

    def test_frozen_frame_counter_does_not_confirm_video(self):
        output=self.run_capture(mode='frozen_frames',error=ValueError,reason='frame-count progress')
        self.assertNotIn('VIDEO_AND_POSE_READY',output)

    def test_video_stall_after_ready_cleans_requests(self):
        output=self.run_capture(mode='stall',error=RuntimeError,reason='video=recorder receipt stale')
        self.assertIn('VIDEO_AND_POSE_READY',output)
        self.assertNotIn('CAPTURE_SAVED',output)

    def test_foreign_boot_stop_receipt_not_saved(self):
        output=self.run_capture(mode='wrong_stop_boot',error=ValueError,reason='finalization not confirmed')
        self.assertNotIn('CAPTURE_SAVED',output)

    def test_existing_commit_blocks_without_creating_requests(self):
        self.run_capture(mode='committed',error=ValueError,reason='mission is committed')

    def test_failed_recorder_reports_error_and_cleans_requests(self):
        self.run_capture(mode='recorder_failed',error=RuntimeError,reason='disk full')

    def run_capture(self, produce_pose=True, mode='normal', error=None, reason=None):
        import tempfile
        import json
        import io
        from pathlib import Path
        from unittest.mock import patch
        from contextlib import ExitStack, redirect_stdout
        import capture_camera_timing as c
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as d, ExitStack() as stack:
            root=Path(d); now=[1000]; last_token=['']
            for name in ('REQUEST','RECORD','ACK','POSE','COMMIT'):
                stack.enter_context(patch.object(c,name,root/name))
            if mode=='committed': c.COMMIT.write_text('existing-mission')
            if mode=='immediate_stop':
                remove=c.remove_owned
                def immediate_remove(path,token):
                    remove(path,token)
                    if path == c.RECORD:
                        c.ACK.write_text(json.dumps(dict(token=token,status='stopped',
                            directory='test-run',ts_ms=now[0],clock_id='mono-v1:test')))
                        now[0]+=100  # Cleanup returns after the writer's receipt.
                stack.enter_context(patch.object(c,'remove_owned',immediate_remove))
            def sleep(_):
                now[0]+=500
                if c.RECORD.exists():
                    token=c.RECORD.read_text().strip();last_token[0]=token
                    # Match the recorder's 500ms heartbeat rather than faking a
                    # brand-new receipt at every caller poll.
                    if not c.ACK.exists() or now[0]%500 == 0:
                        ack=dict(token=token,status='recording',directory='test-run',
                                 clock_id='mono-v1:test',ts_ms=now[0],frames=(now[0]-1000)//100)
                        if mode=='wrong_boot': ack['clock_id']='mono-v1:other'
                        if mode=='future_stamp': ack['ts_ms']+=1000
                        if mode=='frozen_frames': ack['frames']=1
                        if mode=='recorder_failed': ack.update(status='failed',error='disk full')
                        if mode!='stall' or now[0]<2500: c.ACK.write_text(json.dumps(ack))
                    if produce_pose:
                        pitch='1.047651819262683e34' if mode=='invalid_pose_angle' else '0'
                        sdk=430208 if mode=='frozen_sdk_stamp' else now[0]
                        with c.POSE.open('a') as pose:
                            pose.write(f'{now[0]},0,{pitch},mono-v1:test,{now[0]},{now[0]},{sdk},{sdk*1000},{token}\n')
                else:
                    c.ACK.write_text(json.dumps(dict(token=last_token[0],status='stopped',
                        directory='test-run',ts_ms=now[0],
                        clock_id='mono-v1:other' if mode=='wrong_stop_boot' else 'mono-v1:test')))
            stack.enter_context(patch.object(c,'now_ms',lambda:now[0]))
            stack.enter_context(patch.object(c,'clock_id',lambda:'mono-v1:test'))
            stack.enter_context(patch.object(c.time,'sleep',sleep))
            output=io.StringIO()
            with redirect_stdout(output):
                if error:
                    with self.assertRaisesRegex(error,reason): c.capture(15)
                else: c.capture(15)
            self.assertFalse(c.REQUEST.exists());self.assertFalse(c.RECORD.exists())
            return output.getvalue()


if __name__ == '__main__': unittest.main()
