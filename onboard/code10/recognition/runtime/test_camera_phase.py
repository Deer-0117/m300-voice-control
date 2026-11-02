import contextlib
import csv
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

try:
    from .calibrate_camera_phase import estimate, main
except ImportError:
    from calibrate_camera_phase import estimate, main


def data(lag=180, motion=None):
    motion = motion or (lambda t: 8*math.sin(t*1.7)+3*math.sin(t*3.1))
    poses = [dict(ts_ms=t, yaw_deg=motion(t/1000), pitch_deg=0,
                  clock_id='mono-v1:test', token='one')
             for t in range(0, 30001, 100)]
    obs = [dict(frame_ts_ms=t, x_px=640-9*motion((t-lag)/1000), y_px=360,
                clock_id='mono-v1:test') for t in range(1500, 28500, 50)]
    return obs, poses


class PhaseTests(unittest.TestCase):
    def test_known_positive_negative_lag_and_flags(self):
        for lag in (180, -120):
            r = estimate(*data(lag))
            self.assertEqual(r['status'], 'candidate', r['reasons'])
            self.assertAlmostEqual(r['relative_lag_ms'], lag, delta=10)
            self.assertAlmostEqual(r['holdout_lag_ms'], lag, delta=10)
            self.assertTrue(r['shadow_only'])
            self.assertFalse(r['verified'])
            self.assertFalse(r['exposure_calibrated'])
            self.assertEqual(r['maximum_query_support_gap_ms'], 100)
            self.assertNotIn('fx', r)

    def test_static_and_constant_rate_rejected(self):
        for motion in (lambda t: 4, lambda t: 1.1*t):
            r = estimate(*data(motion=motion))
            self.assertEqual(r['status'], 'rejected')
            self.assertTrue(any('nonunique phase' in x for x in r['reasons']))

    def test_accelerating_but_one_way_rejected(self):
        r = estimate(*data(motion=lambda t: .02*t*t))
        self.assertEqual(r['status'], 'rejected')
        self.assertTrue(any('both rotation directions' in x for x in r['reasons']))

    def test_pose_gap_and_cross_boot_rejected(self):
        obs, poses = data()
        with self.assertRaisesRegex(ValueError, 'support gap'):
            estimate(obs, [r for r in poses if not 5000 < r['ts_ms'] < 5500])
        for row in poses:
            row['clock_id'] = 'mono-v1:other'
        with self.assertRaisesRegex(ValueError, 'boot differ'):
            estimate(obs, poses)

    def test_holdout_different_delay_rejected(self):
        obs, poses = data()
        changed, _ = data(400)
        obs[len(obs)//2:] = changed[len(obs)//2:]
        r = estimate(obs, poses)
        self.assertEqual(r['status'], 'rejected')
        self.assertTrue(any('lag disagreement' in x for x in r['reasons']))

    def test_sign_scale_and_vertical_instability_rejected(self):
        for scale in (-1, 1.5):
            obs, poses = data()
            for r in obs[len(obs)//2:]:
                r['x_px'] = 640+scale*(r['x_px']-640)
            result = estimate(obs, poses)
            self.assertEqual(result['status'], 'rejected')
            self.assertTrue(any('sign or scale' in x for x in result['reasons']))
        obs, poses = data()
        for r in obs:
            r['y_px'] = r['x_px']
        self.assertTrue(any('vertical' in x for x in estimate(obs, poses)['reasons']))

    def test_search_boundary_and_periodic_alias_rejected(self):
        r = estimate(*data(1200))
        self.assertEqual(r['status'], 'rejected')
        self.assertTrue(any('boundary' in x for x in r['reasons']))
        r = estimate(*data(motion=lambda t: 3*math.sin(2*math.pi*t)))
        self.assertEqual(r['status'], 'rejected')
        self.assertTrue(any('nonunique' in x for x in r['reasons']))

    def test_invalid_rows_tokens_and_yaw_wrap(self):
        obs, poses = data()
        for r in poses:
            r['yaw_deg'] = (r['yaw_deg']+179+180)%360-180
        self.assertEqual(estimate(obs, poses)['status'], 'candidate')
        poses[0]['token'] = 'other'
        with self.assertRaisesRegex(ValueError, 'token'):
            estimate(obs, poses)
        obs, poses = data()
        obs[0]['x_px'] = float('nan')
        with self.assertRaises(ValueError):
            estimate(obs, poses)
        with self.assertRaises(ValueError):
            estimate(*data(), step_ms=0)

    def test_finite_but_physically_invalid_pose_rejected(self):
        obs, poses = data()
        poses[0]['yaw_deg'] = 1e34
        with self.assertRaisesRegex(ValueError, 'physical input gate'):
            estimate(obs, poses)
        obs, poses = data()
        poses[40]['pitch_deg'] = 1e34
        with self.assertRaisesRegex(ValueError, 'physical input gate'):
            estimate(obs, poses)

    def test_initialization_jump_outside_query_support_does_not_veto(self):
        obs, poses = data()
        baseline = estimate(obs, poses)
        # Queries begin at 500ms and finish at 29450ms. These startup/cleanup
        # samples are not interpolation support for any scanned lag.
        poses[0]['yaw_deg'] = 150
        poses[-1]['yaw_deg'] = -150
        poses[0]['pitch_deg'] = 45
        poses[-1]['pitch_deg'] = -45
        result = estimate(obs, poses)
        self.assertEqual(result['status'], 'candidate', result['reasons'])
        self.assertEqual(result['relative_lag_ms'], baseline['relative_lag_ms'])
        self.assertEqual(result['pose_support_range_ms'], [500, 29500])
        self.assertEqual(result['pitch_motion']['span_deg'], 0)
        self.assertGreater(result['excluded_outside_support_pose_rows'], 0)

    def test_query_boundary_bracket_jumps_are_not_discarded(self):
        for boundary in ('left', 'right'):
            obs, poses = data()
            # Earliest query becomes 525ms, so 500ms must still be checked;
            # latest query becomes 29475ms, so 29500ms is also required.
            for row in obs:
                row['frame_ts_ms'] += 25
            index = 5 if boundary == 'left' else 295
            poses[index]['yaw_deg'] = 150
            with self.assertRaisesRegex(ValueError, 'pose jumps.*query support'):
                estimate(obs, poses)

    def test_pitch_gate_checks_all_lags_and_their_boundary_support(self):
        for index in (5, 295):
            obs, poses = data()
            for row in obs:
                row['frame_ts_ms'] += 25
            poses[index]['pitch_deg'] = 2.01
            result = estimate(obs, poses)
            self.assertEqual(result['status'], 'rejected')
            self.assertTrue(any('pitch span >2deg' in reason for reason in result['reasons']))
        obs, poses = data()
        for row in poses:
            row['pitch_deg'] = 45*math.sin(row['ts_ms']/1000)
        result = estimate(obs, poses)
        self.assertEqual(result['status'], 'rejected')
        self.assertEqual(result['pitch_motion']['status'], 'exceeds_gate')
        obs, poses = data()
        poses[5]['pitch_deg'] = 2.0
        result = estimate(obs, poses)
        self.assertEqual(result['status'], 'candidate', result['reasons'])
        self.assertEqual(result['pitch_motion']['span_deg'], 2.0)

    def test_missing_pitch_or_vertical_pixels_cannot_be_candidate(self):
        for missing in ('pitch_deg', 'y_px'):
            obs, poses = data()
            for row in poses if missing == 'pitch_deg' else obs:
                row.pop(missing)
            result = estimate(obs, poses)
            self.assertEqual(result['status'], 'rejected')
            self.assertTrue(any(missing+' missing; near-horizontal sweep not verified' == reason
                                for reason in result['reasons']))
            self.assertFalse(result['verified'])
            self.assertFalse(result['exposure_calibrated'])
    def test_cli_provenance_rejection_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name, rows in zip(('obs.csv', 'pose.csv'), data()):
                if name == 'pose.csv':
                    for row in rows:
                        row['session_token'] = row.pop('token')
                with (root/name).open('w', newline='', encoding='utf8') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
            args = ['--observations', str(root/'obs.csv'), '--poses', str(root/'pose.csv'),
                    '--token', 'one', '--output', str(root/'result.json')]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(args), 0)
            report = json.loads((root/'result.json').read_text())
            self.assertEqual(len(report['sources']['poses']['sha256']), 64)
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(args)
            args[-1] = str(root/'rejected.json')
            args[5] = 'absent'
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(args), 2)
            report = json.loads((root/'rejected.json').read_text())
            self.assertFalse(report['exposure_calibrated'])


if __name__ == '__main__':
    unittest.main()
