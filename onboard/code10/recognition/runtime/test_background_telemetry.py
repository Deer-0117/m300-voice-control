"""Flight telemetry statistics: provenance, control ownership and measured peaks."""
import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
import unittest

import analyze_background_telemetry as telemetry


def sample(seq=1, ts=1000, *, yaw=0, pitch=0, velocity=(3, 4, 2),
           boot='mono-v1:boot-a', session='session-a', device=0, mode=6, status=2):
    """SDK-shaped records; SDK and host clocks intentionally have different epochs."""
    def topic(**values):
        return dict(code=0, valid=True, fresh=True, new_sample=True,
                    host_read_begin_ms=ts - 1, host_read_end_ms=ts,
                    last_progress_ms=ts, sample_age_ms=0,
                    sdk_millisecond_raw=ts + 100_000,
                    sdk_microsecond_raw=(ts + 100_000) * 1000, **values)
    cy, sy = math.cos(math.radians(yaw / 2)), math.sin(math.radians(yaw / 2))
    cp, sp = math.cos(math.radians(pitch / 2)), math.sin(math.radians(pitch / 2))
    return dict(schema=telemetry.SCHEMA, event='telemetry_sample', clock_id=boot,
                session_id=session, seq=seq, ts_ms=ts, sample_begin_ms=ts - 2,
                sample_end_ms=ts, queue_dropped_total=0, write_errors_total=0,
                velocity=topic(n_mps=velocity[0], e_mps=velocity[1], u_mps=velocity[2], health=1),
                quaternion=topic(w=cy * cp, x=-sy * sp, y=cy * sp, z=sy * cp,
                                 yaw_deg=99999, pitch_deg=99999),
                angular_rate=topic(frame='NED', x_radps=0, y_radps=0, z_radps=0.5),
                flight_status=topic(value=status), display_mode=topic(value=mode),
                control_device=topic(device_status=device, control_mode=0,
                                     flight_status=0, vrc_status=0))


class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def write(self, rows, name='background_telemetry.jsonl'):
        path = self.root / name
        path.write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')
        return path

    def report(self, rows):
        return telemetry.analyze([self.write(rows)])

    def bucket(self, report, group='rc_manual_airborne', index=0):
        return report['sessions'][index]['groups'][group]

    def maximum(self, report, metric, group='rc_manual_airborne', index=0):
        return self.bucket(report, group, index)['metrics'][metric]['maximum']

    def test_actual_velocity_components_and_peak_evidence(self):
        rows = [sample(), sample(2, 1100, velocity=(6, 8, -3))]
        rows[1]['yaw_rate_degps'] = 99999  # A command must never replace telemetry.
        report = self.report(rows)
        self.assertEqual(self.maximum(report, 'horizontal_speed_mps')['value'], 10)
        self.assertAlmostEqual(self.maximum(report, 'speed_3d_mps')['value'], math.sqrt(109))
        self.assertEqual(self.maximum(report, 'ascent_speed_mps')['value'], 2)
        self.assertEqual(self.maximum(report, 'descent_speed_mps')['value'], 3)
        peak = self.maximum(report, 'horizontal_speed_mps')
        self.assertEqual((peak['clock_id'], peak['session_id'], peak['seq'], peak['host_sample_ts_ms']),
                         ('mono-v1:boot-a', 'session-a', 2, 1100))
        self.assertEqual(self.bucket(report)['metrics']['horizontal_speed_mps']['count'], 2)

    def test_manual_auto_unknown_are_not_combined(self):
        rows = [sample(), sample(2, 1100, device=4, velocity=(100, 0, 0)),
                sample(3, 1200, velocity=(200, 0, 0)), sample(4, 1300, status=0)]
        rows[2]['control_device']['fresh'] = False
        report = self.report(rows)
        self.assertEqual(self.maximum(report, 'horizontal_speed_mps')['value'], 5)
        self.assertEqual(self.maximum(report, 'horizontal_speed_mps', 'automatic_airborne')['value'], 100)
        self.assertEqual(self.maximum(report, 'horizontal_speed_mps', 'unknown_airborne')['value'], 200)
        self.assertEqual(self.bucket(report, 'ground')['rows'], 1)

    def test_rc_owner_with_return_home_mode_is_automatic(self):
        report = self.report([sample(mode=15)])
        self.assertEqual(self.bucket(report)['rows'], 0)
        self.assertEqual(self.bucket(report, 'automatic_airborne')['rows'], 1)

    def test_granted_sdk_authority_does_not_change_actual_rc_source(self):
        row = sample()
        row['control_device']['flight_status'] = 1
        self.assertEqual(self.bucket(self.report([row]))['rows'], 1)

    def test_virtual_rc_and_assisted_takeoff_are_not_manual(self):
        row = sample()
        row['control_device']['vrc_status'] = 1
        report = self.report([row, sample(2, 1100, mode=10)])
        self.assertEqual(self.bucket(report)['rows'], 0)
        self.assertEqual(self.bucket(report, 'automatic_airborne')['rows'], 1)
        self.assertEqual(self.bucket(report, 'unknown_airborne')['rows'], 1)

    def test_missing_stale_or_failed_classification_topics_fail_closed(self):
        for name in ('flight_status', 'display_mode', 'control_device'):
            for defect in ('missing', 'age', 'progress_age', 'sdk_error'):
                with self.subTest(name=name, defect=defect):
                    row = sample()
                    if defect == 'missing':
                        del row[name]
                    elif defect == 'age':
                        row[name]['sample_age_ms'] = 501
                    elif defect == 'progress_age':
                        row[name]['last_progress_ms'] = 499
                    else:
                        row[name]['code'] = 1
                    report = self.report([row])
                    self.assertEqual(self.bucket(report)['rows'], 0)
                    group = 'unknown_flight_state' if name == 'flight_status' else 'unknown_airborne'
                    self.assertEqual(self.bucket(report, group)['rows'], 1)

    def test_invalid_or_repeated_velocity_has_no_peak_not_zero(self):
        for key, value in [('health', 0), ('health', True), ('fresh', False),
                           ('new_sample', False), ('sample_age_ms', 201),
                           ('host_read_end_ms', 1001), ('code', False), ('n_mps', None)]:
            with self.subTest(key=key):
                row = sample()
                row['velocity'][key] = value
                report = self.report([row])
                metric = self.bucket(report)['metrics']['horizontal_speed_mps']
                self.assertIsNone(metric['maximum'])
                self.assertEqual(metric['count'], 0)

    def test_heading_wrap_uses_quaternion_and_host_time(self):
        report = self.report([sample(yaw=179), sample(2, 1100, yaw=-179)])
        peak = self.maximum(report, 'heading_rate_abs_degps')
        self.assertAlmostEqual(peak['value'], 20)
        self.assertAlmostEqual(peak['signed_value'], 20)
        self.assertEqual(peak['previous_seq'], 1)
        self.assertAlmostEqual(self.maximum(report, 'ned_vertical_angular_rate_abs_degps')['value'],
                               math.degrees(.5))

    def test_gyro_ned_is_distinct_and_other_frames_are_rejected(self):
        rows = [sample(), sample(2, 1100, yaw=0)]
        rows[1]['angular_rate']['z_radps'] = -1
        report = self.report(rows)
        self.assertEqual(self.maximum(report, 'heading_rate_abs_degps')['value'], 0)
        self.assertAlmostEqual(self.maximum(report, 'ned_vertical_angular_rate_abs_degps')['signed_value'],
                               -180 / math.pi)
        rows[0]['angular_rate']['frame'] = 'BODY'
        self.assertIsNone(self.maximum(self.report(rows[:1]), 'ned_vertical_angular_rate_abs_degps'))

    def test_heading_never_crosses_boot_session_or_control_group(self):
        for second in (sample(2, 1100, yaw=90, boot='mono-v1:boot-b'),
                       sample(2, 1100, yaw=90, session='session-b'),
                       sample(2, 1100, yaw=90, device=4)):
            with self.subTest(second=second['session_id'], device=second['control_device']['device_status']):
                report = self.report([sample(), second])
                for session in report['sessions']:
                    for bucket in session['groups'].values():
                        self.assertIsNone(bucket['metrics']['heading_rate_abs_degps']['maximum'])

    def test_heading_rejects_long_short_gap_and_near_vertical_pitch(self):
        for second in (sample(2, 1005, yaw=90), sample(2, 1201, yaw=90),
                       sample(2, 1100, yaw=90, pitch=85)):
            with self.subTest(ts=second['ts_ms'], quaternion=second['quaternion']):
                self.assertIsNone(self.maximum(self.report([sample(), second]), 'heading_rate_abs_degps'))

    def test_invalid_quaternion_breaks_differentiation(self):
        rows = [sample(), sample(2, 1050, yaw=10), sample(3, 1100, yaw=20)]
        rows[1]['quaternion']['w'] = None
        self.assertIsNone(self.maximum(self.report(rows), 'heading_rate_abs_degps'))

    def test_duplicate_sdk_sample_does_not_inflate_count_or_peak(self):
        rows = [sample(), sample(2, 1100, velocity=(999, 0, 0), yaw=90)]
        for topic in ('velocity', 'quaternion', 'angular_rate'):
            for key in ('sdk_millisecond_raw', 'sdk_microsecond_raw'):
                rows[1][topic][key] = rows[0][topic][key]
        report = self.report(rows)
        self.assertEqual(self.maximum(report, 'horizontal_speed_mps')['value'], 5)
        self.assertEqual(self.bucket(report)['metrics']['horizontal_speed_mps']['count'], 1)
        self.assertIsNone(self.maximum(report, 'heading_rate_abs_degps'))

    def test_repeated_fresh_reads_preserve_last_new_quaternion_for_difference(self):
        rows = [sample(yaw=0), sample(2, 1020, yaw=0), sample(3, 1040, yaw=4),
                sample(4, 1060, yaw=4), sample(5, 1080, yaw=8)]
        for index in (1, 3):
            current, previous = rows[index]['quaternion'], rows[index - 1]['quaternion']
            current['new_sample'] = False
            current['last_progress_ms'] = previous['last_progress_ms']
            current['sample_age_ms'] = 20
            for key in ('sdk_millisecond_raw', 'sdk_microsecond_raw'):
                current[key] = previous[key]
        report = self.report(rows)
        metric = self.bucket(report)['metrics']['heading_rate_abs_degps']
        self.assertEqual(metric['count'], 2)
        self.assertAlmostEqual(metric['maximum']['value'], 100)

    def test_reversed_sdk_stamps_and_host_times_are_rejected(self):
        rows = [sample(), sample(2, 1100, velocity=(999, 0, 0), yaw=90)]
        for topic in ('velocity', 'quaternion', 'angular_rate'):
            rows[1][topic]['sdk_millisecond_raw'] = 90_000
        report = self.report(rows)
        self.assertEqual(self.maximum(report, 'horizontal_speed_mps')['value'], 5)
        self.assertEqual(report['sessions'][0]['diagnostics']['velocity_nonmonotonic_topic'], 1)
        report = self.report([sample(), sample(2, 999, velocity=(999, 0, 0))])
        self.assertEqual(self.maximum(report, 'horizontal_speed_mps')['value'], 5)
        self.assertEqual(report['sessions'][0]['diagnostics']['nonmonotonic_samples'], 1)

    def test_sdk_millisecond_wrap_is_accepted(self):
        rows = [sample(yaw=0), sample(2, 1100, yaw=10)]
        rows[0]['quaternion']['sdk_millisecond_raw'] = 2**32 - 50
        rows[1]['quaternion']['sdk_millisecond_raw'] = 50
        self.assertAlmostEqual(self.maximum(self.report(rows), 'heading_rate_abs_degps')['value'], 100)

    def test_rotations_inputs_duplicates_and_conflicts(self):
        a = self.write([sample(), sample(2, 1100)], 'background_telemetry.1.jsonl')
        b = self.write([sample(2, 1100), sample(3, 1200)])
        report = telemetry.analyze([b, a, a])
        self.assertEqual(len(report['sources']), 2)
        self.assertEqual(self.bucket(report)['rows'], 3)
        self.assertEqual(report['input_quality']['duplicate_samples'], 1)
        conflict = sample(2, 1100, velocity=(999, 0, 0))
        self.write([conflict, sample(3, 1200)])
        report = telemetry.analyze([a, b])
        self.assertEqual(self.bucket(report)['rows'], 2)
        self.assertEqual(report['excluded_conflicting_identities'], 1)
        self.assertEqual(self.maximum(report, 'horizontal_speed_mps')['value'], 5)

    def test_retained_window_and_observed_cumulative_counters(self):
        rows = [sample(100), sample(102, 1500)]
        for row in rows:
            row['queue_dropped_total'] = 7
            row['write_errors_total'] = 2
        report = self.report(rows)
        session = report['sessions'][0]
        self.assertEqual((session['first_seq'], session['last_seq']), (100, 102))
        self.assertTrue(session['prefix_missing_possible'])
        self.assertFalse(session['complete_flight_verified'])
        self.assertEqual(session['queue_dropped_total_observed_max'], 7)
        self.assertEqual(session['write_errors_total_observed_max'], 2)
        self.assertEqual(session['diagnostics']['missing_sequence_samples'], 1)
        self.assertEqual(session['diagnostics']['gaps_over_200ms'], 1)
        self.assertIn('会话开头不在输入中', telemetry.format_text(report))

    def test_bad_json_identity_clock_filter_and_duplicate_keys(self):
        path = self.write([sample(), sample(2, 1100, boot='mono-v1:boot-b')])
        with path.open('a', encoding='utf-8') as stream:
            stream.write('broken\n{"schema":1,"schema":2}\n[]\n')
            broken = sample(3, 1200)
            broken['sample_begin_ms'] = 1300
            stream.write(json.dumps(broken) + '\n')
        report = telemetry.analyze([path], clock_id='mono-v1:boot-a')
        self.assertEqual(len(report['sessions']), 1)
        self.assertEqual(report['input_quality']['invalid_json'], 3)
        self.assertEqual(report['input_quality']['invalid_samples'], 1)
        self.assertEqual(report['input_quality']['other_clocks'], 1)
        empty = telemetry.analyze([path], clock_id='mono-v1:no-such-boot')
        self.assertEqual(empty['sessions'], [])
        self.assertIn('不能将最大值填写为0', telemetry.format_text(empty))

    def test_nonfinite_input_and_derived_overflow_remain_json_safe(self):
        for value in (float('nan'), float('inf'), 10**500, 1.7e308):
            with self.subTest(value=repr(value)[:20]):
                row = sample(velocity=(value, value, value))
                row['angular_rate']['z_radps'] = value
                report = self.report([row])
                json.dumps(report, allow_nan=False)

    def test_quoted_glob_and_directory_and_missing_input(self):
        self.write([sample()], 'background_telemetry.1.jsonl')
        self.write([sample(2, 1100)])
        glob_report = telemetry.analyze(str(self.root / 'background_telemetry*.jsonl'))
        directory_report = telemetry.analyze(self.root)
        self.assertEqual(self.bucket(glob_report)['rows'], 2)
        self.assertEqual(self.bucket(directory_report)['rows'], 2)
        with self.assertRaises(FileNotFoundError):
            telemetry.analyze(self.root / 'missing*.jsonl')

    def test_cli_writes_reports_without_modifying_input(self):
        path = self.write([sample()])
        original = path.read_bytes()
        out_json, out_text = self.root / 'report.json', self.root / 'report.txt'
        with contextlib.redirect_stdout(io.StringIO()):
            result = telemetry.main([str(path), '--json-out', str(out_json), '--text-out', str(out_text)])
        self.assertEqual(result, 0)
        self.assertEqual(path.read_bytes(), original)
        self.assertEqual(json.loads(out_json.read_text(encoding='utf-8'))['schema'], 'background_telemetry_report_v1')
        self.assertIn('不依赖摄像头', out_text.read_text(encoding='utf-8'))

    def test_cli_rejects_input_output_alias_and_existing_output(self):
        path = self.write([sample()])
        original = path.read_bytes()
        out = self.root / 'report.txt'
        out.write_text('retain', encoding='utf-8')
        for args in ([str(path), '--json-out', str(path), '--overwrite'],
                     [str(self.root / '*.jsonl'), '--json-out', str(path), '--overwrite'],
                     [str(path), '--text-out', str(out)]):
            with self.subTest(args=args), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                telemetry.main(args)
        self.assertEqual(path.read_bytes(), original)
        self.assertEqual(out.read_text(encoding='utf-8'), 'retain')


if __name__ == '__main__':
    unittest.main()
