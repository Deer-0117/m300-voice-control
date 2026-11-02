import contextlib
import csv
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

try:
    from .calibrate_camera_phase_2d import estimate, main, GATES
except ImportError:
    from calibrate_camera_phase_2d import estimate, main, GATES


def motion(t):
    return np.array([7*math.sin(t*1.3)+2*math.sin(t*2.9),
                     4*math.sin(t*.87)+1.8*math.sin(t*2.1)])


def data(lag=180, function=motion, jacobian=None):
    jacobian = np.array([[-8, 1], [.8, 7]]) if jacobian is None else np.asarray(jacobian)
    poses, obs = [], []
    for t in range(0, 34001, 100):
        yaw, pitch = function(t/1000)
        poses.append(dict(ts_ms=t, yaw_deg=float(yaw), pitch_deg=float(pitch),
                          clock_id='mono-v1:synthetic-2d', session_token='synthetic-only'))
    for t in range(1500, 32500, 50):
        x, y = [640, 360]+jacobian@function((t-lag)/1000)
        obs.append(dict(frame_ts_ms=t, x_px=float(x), y_px=float(y), clock_id='mono-v1:synthetic-2d'))
    return obs, poses


class Phase2DTests(unittest.TestCase):
    def test_positive_negative_lag_and_shadow_flags(self):
        for lag in (180, -120):
            r = estimate(*data(lag))
            self.assertEqual(r['status'], 'candidate', r['reasons'])
            self.assertAlmostEqual(r['relative_lag_ms'], lag, delta=10)
            self.assertAlmostEqual(r['holdout_lag_ms'], lag, delta=10)
            self.assertTrue(r['shadow_only'])
            self.assertFalse(r['verified'])
            self.assertFalse(r['exposure_calibrated'])
            self.assertFalse(r['roll']['observed'])
            self.assertEqual(r['maximum_query_support_gap_ms'], 100)
            np.testing.assert_allclose(r['halves'][0]['jacobian_px_per_deg'], [[-8, 1], [.8, 7]], atol=.1)
            self.assertEqual(r['thresholds'], GATES)

    def test_correlated_axes_and_constant_rates_rejected(self):
        functions = [lambda t: np.array([5*math.sin(t), 2*math.sin(t)]),
                     lambda t: np.array([.5*t, .2*t]), lambda t: np.array([1, 2])]
        for function in functions:
            r = estimate(*data(function=function))
            self.assertEqual(r['status'], 'rejected')
            self.assertTrue(any('correlated' in x or 'rank deficient' in x for x in r['reasons']))

    def test_two_axes_quadrature_still_phase_unidentifiable(self):
        r = estimate(*data(function=lambda t: np.array([5*math.sin(t*2), 5*math.cos(t*2)])))
        self.assertEqual(r['status'], 'rejected')
        self.assertTrue(any('phase nonunique' in x for x in r['reasons']))

    def test_heldout_delay_or_jacobian_changes_rejected(self):
        obs, poses = data()
        other, _ = data(400)
        obs[len(obs)//2:] = other[len(obs)//2:]
        r = estimate(obs, poses)
        self.assertEqual(r['status'], 'rejected')
        self.assertTrue(any('lag disagreement' in x for x in r['reasons']))
        obs, poses = data()
        other, _ = data(jacobian=[[-4, 4], [.8, -7]])
        obs[len(obs)//2:] = other[len(obs)//2:]
        r = estimate(obs, poses)
        self.assertEqual(r['status'], 'rejected')
        self.assertTrue(any('Jacobian' in x for x in r['reasons']))

    def test_fixed_holdout_intercept_really_is_fixed(self):
        obs, poses = data()
        for row in obs[len(obs)//2:]:
            row['x_px'] += 60
        r = estimate(obs, poses)
        self.assertEqual(r['status'], 'rejected')
        self.assertTrue(any('FIXED training' in x for x in r['reasons']))

    def test_external_initialization_jump_allowed_but_boundary_jump_rejected(self):
        obs, poses = data()
        poses[0]['yaw_deg'] = 150
        poses[0]['pitch_deg'] = -90
        r = estimate(obs, poses)
        self.assertEqual(r['status'], 'candidate', r['reasons'])
        # Queries begin at500ms: this exact boundary is required by some lags.
        poses[5]['yaw_deg'] = 150
        with self.assertRaisesRegex(ValueError, 'complete query support'):
            estimate(obs, poses)

    def test_between_queries_boundary_bracket_cannot_hide_bad_pose(self):
        obs, poses = data()
        # Earliest image1510ms -> earliest lag query510ms, bracket500ms is needed.
        for row in obs:
            row['frame_ts_ms'] += 10
        poses[5]['pitch_deg'] = 80
        with self.assertRaisesRegex(ValueError, 'query support'):
            estimate(obs, poses)

    def test_unknown_boot_token_bad_angles_and_missing_fields(self):
        for key, value in (('clock_id', 'unknown'), ('session_token', ''), ('pitch_deg', 1e34)):
            obs, poses = data()
            for row in poses:
                row[key] = value
            with self.assertRaises(ValueError):
                estimate(obs, poses)
        obs, poses = data()
        del poses[10]['pitch_deg']
        with self.assertRaises(ValueError):
            estimate(obs, poses)
        obs, poses = data()
        for row in poses:
            row['clock_id'] = 'mono-v1:another-boot'
        with self.assertRaisesRegex(ValueError, 'boot differ'):
            estimate(obs, poses)
        obs, poses = data()
        poses[-1]['session_token'] = 'different-capture'
        with self.assertRaisesRegex(ValueError, 'capture token'):
            estimate(obs, poses)
        obs, poses = data()
        del obs[10]['y_px']
        with self.assertRaises(ValueError):
            estimate(obs, poses)

    def test_gaps_nonfinite_short_data_and_search_boundary(self):
        obs, poses = data()
        with self.assertRaisesRegex(ValueError, 'support gap'):
            estimate(obs, [r for r in poses if not 5000 < r['ts_ms'] < 5500])
        with self.assertRaises(ValueError):
            estimate(obs[:20], poses)
        obs[2]['x_px'] = float('nan')
        with self.assertRaises(ValueError):
            estimate(obs, poses)
        r = estimate(*data(1200))
        self.assertEqual(r['status'], 'rejected')
        self.assertTrue(any('boundary' in x for x in r['reasons']))

    def test_roll_and_support_spans_are_not_ignored(self):
        obs, poses = data()
        for row in poses:
            row['roll_deg'] = 5*math.sin(row['ts_ms']/1000)
        r = estimate(obs, poses)
        self.assertEqual(r['status'], 'rejected')
        self.assertTrue(any('roll span' in x for x in r['reasons']))
        r = estimate(*data(function=lambda t: motion(t)*3))
        self.assertEqual(r['status'], 'rejected')
        self.assertTrue(any('local-model gate' in x for x in r['reasons']))

    def test_cli_exact_token_provenance_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name, rows in zip(('obs.csv', 'pose.csv'), data()):
                with (root/name).open('w', newline='', encoding='utf8') as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                    writer.writeheader(); writer.writerows(rows)
            args = ['--observations', str(root/'obs.csv'), '--poses', str(root/'pose.csv'),
                    '--token', 'synthetic-only', '--output', str(root/'report.json')]
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(args), 0)
            r = json.loads((root/'report.json').read_text())
            self.assertEqual(len(r['sources']['poses']['sha256']), 64)
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                main(args)
            args[5], args[-1] = 'absent', str(root/'rejected.json')
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(args), 2)
            self.assertFalse(json.loads((root/'rejected.json').read_text())['verified'])


if __name__ == '__main__':
    unittest.main()
