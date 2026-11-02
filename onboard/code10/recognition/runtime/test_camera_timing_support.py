"""Regression checks for real interpolation support and SDK angle validity."""
import math
import unittest
from unittest.mock import patch

import numpy as np

from calibrate_camera_timing import estimate, interpolate_supported


def sweep():
    def motion(t):
        return 12*math.sin(t*1.7)+4*math.sin(t*3.1)
    poses = [dict(ts_ms=t, yaw_deg=motion(t/1000), pitch_deg=0,
                  clock_id='mono-v1:support') for t in range(0, 24001, 20)]
    observations = [dict(frame_ts_ms=t, bearing_deg=-motion((t-180)/1000),
                         clock_id='mono-v1:support')
                    for t in range(1500, 22000, 50)]
    return observations, poses


class TimingSupportTests(unittest.TestCase):
    def test_boundary_gaps_cannot_hide_outside_cropped_pose_range(self):
        # Full candidate query range is [500, 22950]. At each boundary, one
        # enclosing pose is outside that range and the other is inside it.
        for low, high in ((0, 520), (22940, 24000)):
            with self.subTest(gap=(low, high)):
                obs, poses = sweep()
                poses = [r for r in poses if not low < r['ts_ms'] < high]
                with patch('calibrate_camera_timing.np.interp') as interpolate:
                    with self.assertRaisesRegex(ValueError, 'support gap exceeds 150ms'):
                        estimate(obs, poses)
                    interpolate.assert_not_called()

    def test_gap_at_nonwinning_candidate_still_rejects_entire_fit(self):
        obs, poses = sweep()
        # Best lag 180 ms only queries >=1320 ms, but lag 1000 ms needs
        # interpolation at 500 ms. Candidate-specific skipping is forbidden.
        poses = [r for r in poses if not 0 < r['ts_ms'] < 520]
        with self.assertRaisesRegex(ValueError, 'support gap exceeds 150ms'):
            estimate(obs, poses)

    def test_distant_same_boot_segment_gap_does_not_reject_valid_recording(self):
        obs, poses = sweep()
        expected = estimate(obs, poses)
        poses += [dict(ts_ms=t, yaw_deg=0, pitch_deg=0, clock_id='mono-v1:support')
                  for t in range(1000000, 1001001, 100)]
        report = estimate(obs, poses)
        self.assertEqual(report['status'], 'candidate')
        self.assertEqual(report['profile'], expected['profile'])
        self.assertFalse(report['verified'])
        self.assertFalse(report['exposure_calibrated'])
        self.assertTrue(report['shadow_only'])

    def test_exact_pose_samples_do_not_need_neighbors_across_gap(self):
        actual = interpolate_supported(np.array([0, 1000]),
                                       np.array([0, 1000]), np.array([1, 9]))
        np.testing.assert_array_equal(actual, [1, 9])

    def test_bracket_limit_is_150ms_inclusive(self):
        actual = interpolate_supported(np.array([75]),
                                       np.array([0, 150]), np.array([0, 10]))
        np.testing.assert_allclose(actual, [5])
        for interval in (151, 200):
            with self.subTest(interval=interval):
                with self.assertRaisesRegex(ValueError, 'support gap exceeds 150ms'):
                    interpolate_supported(np.array([interval/2]),
                                          np.array([0, interval]), np.array([0, 10]))

    def test_no_endpoint_extrapolation(self):
        for query in (-1, 101):
            with self.subTest(query=query):
                with self.assertRaisesRegex(ValueError, 'extrapolate'):
                    interpolate_supported(np.array([query]),
                                          np.array([0, 100]), np.array([0, 10]))

    def test_impossible_finite_sdk_angles_are_rejected_before_unwrap(self):
        cases = [('yaw_deg', value) for value in (1.047651819262683e34, 360.001, -360.001)]
        cases += [('pitch_deg', value) for value in (1.047651819262683e34, 180.001, -180.001)]
        for field, value in cases:
            with self.subTest(field=field, value=value):
                obs, poses = sweep()
                poses[len(poses)//2][field] = value
                with patch('calibrate_camera_timing.np.unwrap') as unwrap:
                    with self.assertRaisesRegex(ValueError, 'invalid camera angles'):
                        estimate(obs, poses)
                    unwrap.assert_not_called()

    def test_nonfinite_sdk_angles_are_rejected(self):
        for field in ('yaw_deg', 'pitch_deg'):
            for value in (float('nan'), float('inf'), -float('inf')):
                with self.subTest(field=field, value=value):
                    obs, poses = sweep()
                    poses[len(poses)//2][field] = value
                    with self.assertRaisesRegex(ValueError, 'nonfinite angles'):
                        estimate(obs, poses)

    def test_legal_yaw_wrap_and_pitch_bounds_remain_valid(self):
        obs, poses = sweep()
        for i, row in enumerate(poses):
            row['yaw_deg'] = (row['yaw_deg']+179+180)%360-180
            row['pitch_deg'] = 180 if i % 2 else -180
        report = estimate(obs, poses)
        self.assertEqual(report['status'], 'candidate')
        self.assertEqual(report['relative_lag_ms'], 180)


if __name__ == '__main__':
    unittest.main()
