import contextlib
import csv
import io
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import cv2
import numpy as np

try:
    from . import track_camera_landmark as tracking
except ImportError:
    import track_camera_landmark as tracking


class LandmarkTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def recording(self, count=36, blank=False, vanish=False, extra_index=0):
        rng = np.random.default_rng(4)
        original = rng.integers(0, 256, (180, 260, 3), dtype=np.uint8)
        original = cv2.GaussianBlur(original, (3, 3), .6)
        video = cv2.VideoWriter(str(self.root/'liveview.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 20, (260, 180))
        self.assertTrue(video.isOpened(), 'MJPG writer unavailable')
        try:
            for i in range(count):
                frame = cv2.warpAffine(original, np.float32([[1, 0, i*.35], [0, 1, i*.08]]), (260, 180))
                if blank or vanish and i >= count//2:
                    frame[:] = 0
                video.write(frame)
        finally:
            video.release()
        with (self.root/'frames.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=('video_frame', 'frame_ts_ms', 'clock_id'))
            writer.writeheader()
            writer.writerows(dict(video_frame=i, frame_ts_ms=1000+50*i, clock_id='mono-v1:synthetic')
                             for i in range(count+extra_index))

    def test_sequential_tracking_recovers_pixel_translation(self):
        self.recording()
        rows, report = tracking.track(self.root, [60, 45, 110, 85])
        self.assertEqual(len(rows), 36)
        self.assertAlmostEqual(rows[-1]['x_px']-rows[0]['x_px'], 35*.35, delta=.35)
        self.assertAlmostEqual(rows[-1]['y_px']-rows[0]['y_px'], 35*.08, delta=.35)
        self.assertGreaterEqual(report['common_points'], 12)
        self.assertEqual({r['tracked_points'] for r in rows}, {report['common_points']})
        self.assertEqual(rows[0]['frame_ts_ms'], 1000)
        self.assertEqual(report['sequentially_decoded_frames'], 36)
        self.assertEqual(len(report['sources']['video']['sha256']), 64)
        self.assertFalse(report['exposure_calibrated'])
        self.assertNotIn('fx', report)

    def test_start_end_use_original_indices_and_decode_prefix(self):
        self.recording()
        rows, report = tracking.track(self.root, [60, 45, 110, 85], start=10, end=25)
        self.assertEqual([rows[0]['video_frame'], rows[-1]['video_frame']], [10, 24])
        self.assertEqual(len(rows), 15)
        self.assertEqual(rows[0]['frame_ts_ms'], 1500)
        self.assertEqual(report['sequentially_decoded_frames'], 25)
        self.assertAlmostEqual(rows[-1]['x_px']-rows[0]['x_px'], 14*.35, delta=.35)

    def test_same_final_feature_set_removes_survivor_centroid_drift(self):
        original = np.array([[10, 10], [20, 10], [30, 10], [200, 10]], dtype=float)
        history = [original.copy(), original+[2, 0], original+[4, 0]]
        history[1][3] = [210, 10]
        history[2][3] = [float('nan'), float('nan')]
        trace, report = tracking.fixed_survivor_trace(history, original, np.array([0, 1, 2]))
        np.testing.assert_allclose(trace, [[20, 10], [22, 10], [24, 10]])
        self.assertEqual(report['displacement_residual_p90_max_px'], 0)
        self.assertNotEqual(float(np.median(history[1][:, 0])), trace[1, 0])

    def test_no_features_or_lost_original_features_reject(self):
        self.recording(blank=True)
        with self.assertRaisesRegex(tracking.TrackingRejected, 'insufficient initial'):
            tracking.track(self.root, [60, 45, 110, 85])
        self.recording(vanish=True)
        with self.assertRaises(tracking.TrackingRejected):
            tracking.track(self.root, [60, 45, 110, 85])

    def test_early_video_end_and_invalid_roi_reject(self):
        self.recording(extra_index=4)
        with self.assertRaisesRegex(tracking.TrackingRejected, 'ended before indexed'):
            tracking.track(self.root, [60, 45, 110, 85])
        with self.assertRaisesRegex(tracking.TrackingRejected, 'ROI outside'):
            tracking.track(self.root, [240, 150, 80, 80])

    def test_strict_frame_index_clock_and_timestamp(self):
        self.recording(count=5)
        original = (self.root/'frames.csv').read_text()
        for replacement in (original.replace('2,1100', '3,1100'),
                            original.replace('2,1100', '2,1050'),
                            original.replace('2,1100,mono-v1:synthetic', '2,1100,mono-v1:other'),
                            original+'\x00'):
            (self.root/'frames.csv').write_text(replacement)
            with self.assertRaises(ValueError):
                tracking.read_frames(self.root/'frames.csv')

    def test_quality_limits_cannot_go_below_twelve_features(self):
        self.recording()
        with self.assertRaisesRegex(ValueError, '>=12'):
            tracking.track(self.root, [60, 45, 110, 85], min_points=3)
        with self.assertRaisesRegex(tracking.TrackingRejected, 'inconsistent feature motion'):
            tracking.track(self.root, [60, 45, 110, 85], max_step_residual_px=.000001)

    def test_cli_no_overwrite_and_failure_has_report_but_no_csv(self):
        self.recording()
        args = ['--session', str(self.root), '--roi', '60', '45', '110', '85',
                '--output', str(self.root/'out.csv')]
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(tracking.main(args), 0)
        with (self.root/'out.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(list(rows[0]), list(tracking.FIELDS))
        quality = json.loads((self.root/'out.quality.json').read_text())
        self.assertEqual(quality['status'], 'trajectory_extracted')
        before = (self.root/'out.csv').read_bytes()
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            tracking.main(args)
        self.assertEqual((self.root/'out.csv').read_bytes(), before)
        args[-1] = str(self.root/'bad.csv')
        args[3] = '250'
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(tracking.main(args), 2)
        self.assertFalse((self.root/'bad.csv').exists())
        self.assertEqual(json.loads((self.root/'bad.quality.json').read_text())['status'], 'rejected')

    @staticmethod
    def planar_h(fraction):
        angle = fraction*.28
        cosine, sine = np.cos(angle), np.sin(angle)
        center = np.array([[1, 0, -130], [0, 1, -90], [0, 0, 1]], dtype=float)
        rotate = np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])
        perspective = np.array([[1, .04*fraction, 0], [0, 1, 0],
                                [.0007*fraction, -.0004*fraction, 1]])
        restore = np.array([[1, 0, 130+8*fraction], [0, 1, 90-4*fraction], [0, 0, 1]])
        return restore@perspective@rotate@center

    def planar_recording(self, count=40):
        rng = np.random.default_rng(31)
        original = rng.integers(0, 256, (180, 260, 3), dtype=np.uint8)
        original = cv2.GaussianBlur(original, (3, 3), .6)
        transforms = [self.planar_h(i/(count-1)) for i in range(count)]
        video = cv2.VideoWriter(str(self.root/'liveview.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 20, (260, 180))
        self.assertTrue(video.isOpened(), 'MJPG writer unavailable')
        try:
            for transform in transforms:
                video.write(cv2.warpPerspective(original, transform, (260, 180)))
        finally:
            video.release()
        with (self.root/'frames.csv').open('w', newline='', encoding='utf8') as stream:
            writer = csv.DictWriter(stream, fieldnames=('video_frame', 'frame_ts_ms', 'clock_id'))
            writer.writeheader()
            writer.writerows(dict(video_frame=i, frame_ts_ms=1000+50*i, clock_id='mono-v1:synthetic')
                             for i in range(count))
        return transforms

    def noisy_jpeg_planar_sequence(self, seed, *, jpeg=90, contrast=.7,
                                  noise=2., gain=.25, defocus=1., exposure=.8, count=100):
        """Known static plane, independently noisy JPEG frames, no pose data.

        Coordinates are rendered directly from ground-truth H. Five shutter
        samples simulate motion blur; noise is before JPEG compression. An
        in-memory capture avoids adding uncontrolled AVI recompression.
        """
        rng = np.random.default_rng(seed)
        source = cv2.GaussianBlur(rng.integers(0, 256, (360, 520), dtype=np.uint8), (3, 3), .6)
        source = np.clip(32+contrast*(source.astype(float)-32), 0, 255).astype(np.uint8)

        def transform(index):
            value = .8*math.sin(2*math.pi*index/(count-1))+.2*math.sin(4*math.pi*index/(count-1))
            cosine, sine = math.cos(.28*value), math.sin(.28*value)
            return (np.array([[1, 0, 260+18*value], [0, 1, 180-7*value], [0, 0, 1.]])@
                np.array([[1, .04*value, 0], [0, 1, 0], [.0006*value, -.00035*value, 1]])@
                np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])@
                np.array([[1, 0, -260], [0, 1, -180], [0, 0, 1.]]))

        frames, matrices = [], []
        for index in range(count):
            matrices.append(transform(index))
            offsets = np.linspace(-exposure/2, exposure/2, 5) if exposure else [0]
            image = np.mean([cv2.warpPerspective(source, transform(index+dt), (520, 360)).astype(float)
                             for dt in offsets], axis=0)
            if defocus:
                image = cv2.GaussianBlur(image, (0, 0), defocus)
            image = np.clip(image*(1+gain*math.sin(2*math.pi*index/37))+
                            rng.normal(0, noise, image.shape), 0, 255).astype(np.uint8)
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, jpeg])
            self.assertTrue(ok)
            image = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
            frames.append(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
        (self.root/'liveview.avi').write_bytes(b'synthetic JPEG frames supplied by in-memory capture')
        with (self.root/'frames.csv').open('w', newline='', encoding='utf8') as stream:
            writer = csv.DictWriter(stream, fieldnames=('video_frame', 'frame_ts_ms', 'clock_id'))
            writer.writeheader()
            writer.writerows(dict(video_frame=i, frame_ts_ms=1000+100*i, clock_id='mono-v1:synthetic')
                             for i in range(count))

        class MemoryCapture:
            def __init__(self):
                self.index = 0

            def isOpened(self):
                return True

            def release(self):
                pass

            def read(self):
                if self.index == len(frames):
                    return False, None
                frame = frames[self.index]
                self.index += 1
                return True, frame

        return MemoryCapture(), matrices

    def test_planar_fixed_anchor_recovers_rotation_perspective_translation(self):
        xx, yy = np.meshgrid(np.linspace(60, 190, 8), np.linspace(40, 140, 7))
        original = np.column_stack((xx.ravel(), yy.ravel()))
        transforms = [self.planar_h(i/19) for i in range(20)]
        history = [tracking._project_homography(h, original)[0] for h in transforms]
        # Two features lost elsewhere never contribute at any frame. The
        # anchor is fixed from the same FINAL original ID set throughout.
        ids = np.arange(2, len(original))
        history[3][0] = [1000, 1000]
        history[-1][0:2] = np.nan
        trace, report = tracking.planar_survivor_trace(history, original, ids)
        anchor = np.median(original[ids], axis=0)
        expected = np.array([tracking._project_homography(h, anchor[None, :])[0][0] for h in transforms])
        np.testing.assert_allclose(trace, expected, atol=1e-5)
        self.assertEqual(len(report['planar_frame_quality']), len(transforms))
        for quality in report['planar_frame_quality']:
            self.assertEqual(quality['inlier_original_feature_ids'], ids.tolist())
            self.assertEqual(quality['motion_model'], 'planar')
            self.assertEqual(quality['status'], 'accepted')

    def test_planar_rejects_outliers_degenerate_geometry_and_reflection(self):
        xx, yy = np.meshgrid(np.linspace(50, 200, 8), np.linspace(40, 140, 7))
        original = np.column_stack((xx.ravel(), yy.ravel()))
        ids = np.arange(len(original))
        moved = tracking._project_homography(self.planar_h(1), original)[0]
        rng = np.random.default_rng(9)
        moved[:20] += rng.uniform(-80, 80, (20, 2))
        with self.assertRaisesRegex(tracking.TrackingRejected, 'RANSAC inliers'):
            tracking.planar_survivor_trace([original, moved], original, ids)
        line = np.column_stack((np.linspace(20, 200, 40), np.linspace(30, 30.01, 40)))
        with self.assertRaisesRegex(tracking.TrackingRejected, 'degenerate two-dimensional'):
            tracking.planar_survivor_trace([line, line+[2, 1]], line, np.arange(len(line)))
        mirror = original.copy()
        mirror[:, 0] = 260-mirror[:, 0]
        with self.assertRaisesRegex(tracking.TrackingRejected, 'reflected'):
            tracking.planar_survivor_trace([original, mirror], original, ids)
        near_pole_h = np.array([[1, 0, -125], [0, 1, -90], [.006, 0, -.2]])
        near_pole = tracking._project_homography(near_pole_h, original)[0]
        with self.assertRaisesRegex(tracking.TrackingRejected, 'projective pole'):
            tracking.planar_survivor_trace([original, near_pole], original, ids)

    def test_planar_video_extracts_anchor_without_translation_gate_relaxation(self):
        transforms = self.planar_recording()
        rows, report = tracking.track(self.root, [50, 35, 150, 110], motion_model='planar')
        anchor = np.asarray(report['anchor_px'])
        expected = np.array([tracking._project_homography(h, anchor[None, :])[0][0] for h in transforms])
        observed = np.array([[row['x_px'], row['y_px']] for row in rows])
        self.assertLess(float(np.max(np.linalg.norm(observed-expected, axis=1))), .8)
        self.assertEqual(report['motion_model'], 'planar')
        self.assertEqual(report['thresholds']['fb_threshold_px'], .75)
        self.assertEqual(report['thresholds']['max_lk_patch_error'], 30)
        self.assertFalse(report['translation_residual_gates_applied'])
        self.assertEqual([q['video_frame'] for q in report['planar_frame_quality']], list(range(40)))
        self.assertNotIn('lag_ms', report)
        self.assertNotIn('yaw_deg', report)
        with self.assertRaisesRegex(tracking.TrackingRejected, 'accumulated feature motion'):
            tracking.track(self.root, [50, 35, 150, 110])

    def test_planar_cli_model_and_lost_features_keep_existing_lk_checks(self):
        self.recording()
        args = ['--session', str(self.root), '--roi', '60', '45', '110', '85',
                '--motion-model', 'planar', '--output', str(self.root/'planar.csv')]
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(tracking.main(args), 0)
        report = json.loads((self.root/'planar.quality.json').read_text())
        self.assertEqual(report['motion_model'], 'planar')
        self.assertEqual(len(report['planar_frame_quality']), 36)
        self.recording(vanish=True)
        with self.assertRaisesRegex(tracking.TrackingRejected, 'too few original features'):
            tracking.track(self.root, [60, 45, 110, 85], motion_model='planar')

    def test_noisy_jpeg_held_out_anchor_truth_and_spatial_quality(self):
        # Seeds 41/73 were held out from the initial 7/19 threshold study.
        # Independently rendered sensor noise, JPEG90, brightness variation,
        # defocus and shutter blur are all present. No real video is used.
        for seed in (41, 73):
            with self.subTest(seed=seed):
                capture, transforms = self.noisy_jpeg_planar_sequence(seed)
                with mock.patch.object(tracking.cv2, 'VideoCapture', return_value=capture):
                    rows, report = tracking.track(self.root, [190, 110, 140, 130], motion_model='planar')
                anchor = np.asarray(report['anchor_px'])
                truth = np.array([tracking._project_homography(h, anchor[None, :])[0][0] for h in transforms])
                observed = np.array([[row['x_px'], row['y_px']] for row in rows])
                self.assertLess(float(np.linalg.norm(observed-truth, axis=1).max()), .5)
                # JPEG/LK/RANSAC output depends on the OpenCV build. A smaller
                # residual is valid, not a failed fixture: image accuracy and
                # all production upper gates still have to pass. Test the
                # 1..1.25px acceptance interval independently below.
                self.assertTrue(math.isfinite(report['planar_inlier_reprojection_p95_max_px']))
                self.assertGreaterEqual(report['planar_inlier_reprojection_p95_max_px'], 0.)
                self.assertLessEqual(report['planar_inlier_reprojection_p95_max_px'], 1.25)
                self.assertLess(report['planar_spatial_anchor_disagreement_max_px'], .75)
                self.assertEqual(report['thresholds']['fb_threshold_px'], .75)
                self.assertEqual(report['thresholds']['planar']['ransac_reprojection_px'], 1.5)
                self.assertFalse(report['verified'])

    def test_mixed_motion_cannot_hide_behind_low_whole_roi_residual(self):
        xx, yy = np.meshgrid(np.linspace(50, 200, 8), np.linspace(40, 140, 7))
        base = np.column_stack((xx.ravel(), yy.ravel()))
        moved = base.copy()
        moved[moved[:, 0] < 125, 0] += 3
        # A codec/platform build may reject this incompatible motion at the
        # RANSAC gate before the spatial gate. Neither may yield a trajectory.
        # The spatial gate is isolated with a controlled fit in the next test.
        with self.assertRaises(tracking.TrackingRejected) as raised:
            tracking.planar_survivor_trace([base, moved], base, np.arange(len(base)))
        quality = raised.exception.report['planar_frame_quality'][-1]
        self.assertEqual(quality['status'], 'rejected')
        self.assertTrue(any(reason in quality['reason'] for reason in
                            ('RANSAC inliers', 'reprojection residual', 'spatial anchor disagreement')))

    def test_spatial_gate_rejects_mixed_motion_after_valid_controlled_fit(self):
        # Isolate spatial validation from version-specific RANSAC membership.
        # The injected WHOLE fit has known 0.7px residuals and all inliers;
        # the production left/right/top/bottom least-squares refits are real.
        xx, yy = np.meshgrid(np.arange(50, 191, 20), np.arange(40, 141, 20))
        base = np.column_stack((xx.ravel(), yy.ravel())).astype(float)
        moved = base.copy()
        moved[moved[:, 0] < 120, 0] += 1.4
        real_fit = tracking.cv2.findHomography

        def known_whole_fit(source, target, method=0, *args, **kwargs):
            if method == tracking.cv2.RANSAC:
                fit = np.eye(3)
                fit[:2, 2] = np.mean(target-source, axis=0)
                return fit, np.ones((len(source), 1), dtype=np.uint8)
            return real_fit(source, target, method, *args, **kwargs)

        with mock.patch.object(tracking.cv2, 'findHomography', side_effect=known_whole_fit):
            with self.assertRaisesRegex(tracking.TrackingRejected, 'spatial anchor disagreement') as raised:
                tracking.planar_survivor_trace([base, moved], base, np.arange(len(base)))
        quality = raised.exception.report['planar_frame_quality'][-1]
        self.assertEqual(quality['inlier_points'], len(base))
        self.assertLess(quality['inlier_reprojection_p95_px'], 1.)
        self.assertGreater(quality['spatial_anchor_disagreement_px'], .75)

    def test_planar_p95_boundary_uses_fixed_gate_without_codec_dependency(self):
        # Inject an identity fit so the actual production P95 gate sees exact
        # independently specified pixel errors. JPEG noise is exercised above;
        # it must not be tuned until it happens to cross a residual lower bound.
        xx, yy = np.meshgrid(np.arange(50, 191, 20), np.arange(40, 141, 20))
        base = np.column_stack((xx.ravel(), yy.ravel())).astype(float)
        signs = np.where(np.indices(xx.shape).sum(axis=0).ravel() % 2, 1., -1.)

        def identity_fit(source, _target, *_args, **_kwargs):
            return np.eye(3), np.ones((len(source), 1), dtype=np.uint8)

        for error in (1.1, 1.25):
            with self.subTest(accepted_error=error):
                moved = base.copy()
                moved[:, 0] += signs*error
                with mock.patch.object(tracking.cv2, 'findHomography', side_effect=identity_fit):
                    _, report = tracking.planar_survivor_trace([base, moved], base, np.arange(len(base)))
                self.assertAlmostEqual(report['planar_inlier_reprojection_p95_max_px'], error)
                self.assertEqual(report['planar_frame_quality'][-1]['status'], 'accepted')
        moved = base.copy()
        moved[:, 0] += signs*1.26
        with mock.patch.object(tracking.cv2, 'findHomography', side_effect=identity_fit):
            with self.assertRaisesRegex(tracking.TrackingRejected, 'reprojection residual') as raised:
                tracking.planar_survivor_trace([base, moved], base, np.arange(len(base)))
        quality = raised.exception.report['planar_frame_quality'][-1]
        self.assertGreater(quality['inlier_reprojection_p95_px'], 1.25)
        self.assertLess(quality['inlier_reprojection_max_px'], 1.5)
        self.assertEqual(tracking.PLANAR_GATES['inlier_residual_p95_px'], 1.25)

    def test_coherent_identity_error_is_explicitly_not_verified(self):
        xx, yy = np.meshgrid(np.linspace(50, 200, 8), np.linspace(40, 140, 7))
        base = np.column_stack((xx.ravel(), yy.ravel()))
        trace, quality = tracking.planar_survivor_trace([base, base+[3, 0]], base, np.arange(len(base)))
        # Geometry cannot distinguish a real 3px translation from a coherent
        # whole-ROI 3px mismatch. Preserve this limitation, never claim identity.
        np.testing.assert_allclose(trace[-1]-trace[0], [3, 0], atol=1e-5)
        self.assertLess(quality['planar_spatial_anchor_disagreement_max_px'], 1e-5)
        self.assertTrue(any('coherent wrong correspondence' in text for text in tracking.LIMITS))


if __name__ == '__main__':
    unittest.main()
