import copy
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location('telemetry_check', Path(__file__).resolve().parents[1]/'scripts/check_background_telemetry.py')
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)
BOOT = '11111111-2222-3333-4444-555555555555'
PID = '1551'


def sample(seq=1, stamp=1000, boot=BOOT, pid=PID):
    clock = 'mono-v1:' + boot
    row = dict(schema='background_telemetry_v1', event='telemetry_sample', clock_id=clock,
               session_id=f'{clock}:{pid}:100', seq=seq, ts_ms=stamp,
               sample_begin_ms=stamp, sample_end_ms=stamp,
               queue_dropped_total=0, write_errors_total=0)
    for topic in checker.TOPICS:
        row[topic] = dict(code=0, valid=True, fresh=True, new_sample=True,
                          host_read_begin_ms=stamp, host_read_end_ms=stamp,
                          last_progress_ms=stamp, sample_age_ms=0,
                          sdk_millisecond_raw=stamp, sdk_microsecond_raw=stamp*1000)
    row['velocity'].update(n_mps=3, e_mps=4, u_mps=0, health=1)
    row['quaternion'].update(w=1, x=0, y=0, z=0)
    row['angular_rate'].update(x_radps=0, y_radps=0, z_radps=.2, frame='NED')
    row['flight_status']['value'] = 0
    row['control_device'].update(device_status=0, vrc_status=0)
    row['display_mode']['value'] = 6
    return row


class TelemetryCheckTests(unittest.TestCase):
    def rows(self):
        return [sample(), sample(2, 1100)]

    def check(self, rows=None, now=1150):
        return checker.inspect(self.rows() if rows is None else rows, BOOT, PID, now)

    def test_current_process_six_topics_ready_without_camera(self):
        report = self.check()
        self.assertEqual(report['status'], 'TELEMETRY_OK')
        self.assertEqual(report['missing'], [])
        self.assertEqual(report['topics']['angular_rate']['frame'], 'NED')
        self.assertEqual(report['topics']['velocity']['health'], 1)

    def test_old_boot_or_other_pid_or_pid_prefix_rejected(self):
        for rows in ([sample(boot='old'), sample(2, 1100, boot='old')],
                     [sample(pid='15510'), sample(2, 1100, pid='15510')]):
            with self.subTest(rows=rows[0]['session_id']):
                self.assertEqual(self.check(rows)['status'], 'TELEMETRY_NOT_READY')

    def test_malformed_session_suffix_rejected(self):
        rows = self.rows()
        for row in rows:
            row['session_id'] += ':old'
        self.assertEqual(self.check(rows)['status'], 'TELEMETRY_NOT_READY')

    def test_old_or_future_rows_cannot_pass(self):
        self.assertEqual(self.check(now=3000)['status'], 'TELEMETRY_NOT_READY')
        self.assertEqual(self.check(now=900)['status'], 'TELEMETRY_NOT_READY')

    def test_one_or_duplicate_sequence_cannot_prove_progress(self):
        report = self.check([sample(), sample()])
        self.assertEqual(report['recent_samples'], 1)
        self.assertFalse(report['sequence_progress'])
        self.assertNotEqual(report['status'], 'TELEMETRY_OK')

    def test_sequence_time_reversal_rejected(self):
        self.assertFalse(self.check([sample(1, 1100), sample(2, 1000)])['sequence_progress'])

    def test_conflicting_duplicate_rejected(self):
        rows = self.rows()
        duplicate = copy.deepcopy(rows[0]); duplicate['velocity']['n_mps'] = 7
        report = self.check(rows + [duplicate])
        self.assertEqual(report['conflicting_samples'], 1)
        self.assertNotEqual(report['status'], 'TELEMETRY_OK')

    def test_sdk_frozen_even_if_seq_advances(self):
        rows = self.rows()
        for topic in checker.TOPICS:
            for field in ('sdk_millisecond_raw', 'sdk_microsecond_raw'):
                rows[1][topic][field] = rows[0][topic][field]
        report = self.check(rows)
        self.assertEqual(set(report['missing']), set(checker.TOPICS))

    def test_raw_sdk_wrap_is_forward(self):
        rows = self.rows()
        for topic in checker.TOPICS:
            rows[0][topic]['sdk_millisecond_raw'] = 2**32-10
            rows[1][topic]['sdk_millisecond_raw'] = 10
        self.assertEqual(self.check(rows)['status'], 'TELEMETRY_OK')

    def test_missing_and_null_topic_report_partial(self):
        for value in (None, {}, {'code': 1234}):
            rows = self.rows(); rows[1]['control_device'] = value
            report = self.check(rows)
            self.assertEqual(report['status'], 'TELEMETRY_PARTIAL')
            self.assertIn('control_device', report['missing'])

    def test_invalid_flags_health_values_and_frame_rejected(self):
        cases = [('velocity', 'fresh', False), ('velocity', 'valid', False),
                 ('velocity', 'code', 4), ('velocity', 'health', 0),
                 ('velocity', 'n_mps', float('nan')), ('quaternion', 'w', 0),
                 ('angular_rate', 'frame', 'BODY'), ('flight_status', 'value', 255),
                 ('control_device', 'device_status', 7)]
        for topic, field, value in cases:
            with self.subTest(topic=topic, field=field):
                rows = self.rows(); rows[1][topic][field] = value
                self.assertIn(topic, self.check(rows)['missing'])

    def test_topic_age_and_read_interval_validated(self):
        rows = self.rows()
        rows[1]['velocity']['sample_age_ms'] = 201
        rows[1]['flight_status']['last_progress_ms'] = 500
        rows[1]['quaternion']['host_read_end_ms'] = 9999
        report = self.check(rows)
        for name in ('velocity', 'flight_status', 'quaternion'):
            self.assertIn(name, report['missing'])

    def test_counters_increment_or_invalid_rejected_but_past_stable_allowed(self):
        rows = self.rows()
        rows[1]['write_errors_total'] = 1
        self.assertFalse(self.check(rows)['counters_stable'])
        rows[0]['write_errors_total'] = 1
        self.assertEqual(self.check(rows)['status'], 'TELEMETRY_OK')
        rows[1]['queue_dropped_total'] = None
        self.assertFalse(self.check(rows)['counters_stable'])

    def test_read_tail_bounded_ignores_partial_line_and_reads_rotation(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            first, second = (json.dumps(r).encode()+b'\n' for r in self.rows())
            (root/'background_telemetry.1.jsonl').write_bytes(first)
            (root/'background_telemetry.jsonl').write_bytes(b'x'*300 + b'\n' + second + b'{"unfinished":')
            with patch.object(checker, 'TAIL_BYTES', len(second)+100):
                rows, errors = checker.read_tail(root)
            self.assertEqual(len(rows), 2)
            self.assertFalse(errors)
            self.assertEqual(self.check(rows)['status'], 'TELEMETRY_OK')

    def test_run_waits_for_second_poll_with_sequence_growth(self):
        rows = self.rows()
        with patch.object(checker.Path, 'read_text', return_value=BOOT), \
             patch.object(checker, 'service_state', return_value=(True, PID)), \
             patch.object(checker, 'read_tail', side_effect=[(rows, []), (rows+[sample(3, 1150)], [])]), \
             patch.object(checker.time, 'monotonic_ns', return_value=1200000000), \
             patch.object(checker.time, 'sleep'):
            report = checker.run(2)
        self.assertEqual(report['status'], 'TELEMETRY_OK')
        self.assertTrue(report['observation_progress'])

    def test_inactive_service_cannot_pass_good_cached_rows(self):
        with patch.object(checker.Path, 'read_text', return_value=BOOT), \
             patch.object(checker, 'service_state', return_value=(False, PID)), \
             patch.object(checker, 'read_tail', return_value=(self.rows(), [])), \
             patch.object(checker.time, 'monotonic_ns', return_value=1200000000):
            report = checker.run(0)
        self.assertEqual(report['status'], 'TELEMETRY_NOT_READY')

    def test_bad_history_is_reported_but_does_not_block_current_process(self):
        rows = self.rows()
        warnings = ['background_telemetry.1.jsonl: invalid JSON line']
        with patch.object(checker.Path, 'read_text', return_value=BOOT), \
             patch.object(checker, 'service_state', return_value=(True, PID)), \
             patch.object(checker, 'read_tail', side_effect=[(rows, warnings), (rows+[sample(3, 1150)], warnings)]), \
             patch.object(checker.time, 'monotonic_ns', return_value=1200000000), \
             patch.object(checker.time, 'sleep'):
            report = checker.run(2)
        self.assertEqual(report['status'], 'TELEMETRY_OK')
        self.assertEqual(report['read_warnings'], warnings)

    def test_frozen_snapshot_never_means_current_recording(self):
        with patch.object(checker.Path, 'read_text', return_value=BOOT), \
             patch.object(checker, 'service_state', return_value=(True, PID)), \
             patch.object(checker, 'read_tail', return_value=(self.rows(), [])), \
             patch.object(checker.time, 'monotonic_ns', return_value=1200000000), \
             patch.object(checker.time, 'monotonic', side_effect=[0, 0, 0, 1]), \
             patch.object(checker.time, 'sleep'):
            report = checker.run(1)
        self.assertFalse(report['observation_progress'])
        self.assertEqual(report['status'], 'TELEMETRY_PARTIAL')

    def test_error_counter_increase_across_polls_cannot_be_forgotten(self):
        rows = self.rows()
        later = [sample(3, 1120), sample(4, 1140)]
        for row in later:
            row['write_errors_total'] = 1
        with patch.object(checker.Path, 'read_text', return_value=BOOT), \
             patch.object(checker, 'service_state', return_value=(True, PID)), \
             patch.object(checker, 'read_tail', side_effect=[(rows, []), (later, [])]), \
             patch.object(checker.time, 'monotonic_ns', return_value=1200000000), \
             patch.object(checker.time, 'monotonic', side_effect=[0, 0, 0, 1]), \
             patch.object(checker.time, 'sleep'):
            report = checker.run(1)
        self.assertEqual(report['status'], 'TELEMETRY_PARTIAL')
        self.assertTrue(report['counter_change_during_check'])

    def test_corrupt_metadata_remains_printable(self):
        rows = self.rows()
        rows[-1]['quaternion']['code'] = float('nan')
        rows[-1]['velocity']['fresh'] = {'unexpected': 1}
        json.dumps(self.check(rows), allow_nan=False)

    def test_timeout_bounds(self):
        for value in (-1, 31, float('nan')):
            with self.assertRaises(ValueError):
                checker.run(value)

    def test_service_override_queries_exact_unit_and_uses_its_pid(self):
        completed = SimpleNamespace(stdout='MainPID=42\nActiveState=active\n')
        with patch.object(checker.subprocess, 'run', return_value=completed) as command:
            self.assertEqual(checker.service_state('balloon-telemetry.service'), (True, '42'))
        self.assertEqual(command.call_args.args[0],
                         ['systemctl', 'show', 'balloon-telemetry.service', '-p', 'ActiveState', '-p', 'MainPID'])

    def test_override_checks_selected_directory_and_current_service_pid(self):
        root = Path('/var/log/balloon-telemetry')
        rows = [sample(pid='42'), sample(2, 1100, pid='42')]
        with patch.object(checker.Path, 'read_text', return_value=BOOT), \
             patch.object(checker, 'service_state', return_value=(True, '42')) as service, \
             patch.object(checker, 'read_tail', side_effect=[(rows, []), (rows+[sample(3, 1150, pid='42')], [])]) as reader, \
             patch.object(checker.time, 'monotonic_ns', return_value=1200000000), \
             patch.object(checker.time, 'sleep'):
            report = checker.run(2, root=str(root), service='balloon-telemetry.service')
        self.assertEqual(report['status'], 'TELEMETRY_OK')
        self.assertEqual(report['pid'], '42')
        self.assertEqual(report['service'], 'balloon-telemetry.service')
        self.assertEqual(report['log_dir'], str(root))
        self.assertTrue(all(call.args == ('balloon-telemetry.service',) for call in service.call_args_list))
        self.assertTrue(all(call.args == (root,) for call in reader.call_args_list))

    def test_override_does_not_accept_original_app_samples(self):
        with patch.object(checker.Path, 'read_text', return_value=BOOT), \
             patch.object(checker, 'service_state', return_value=(True, '42')), \
             patch.object(checker, 'read_tail', return_value=(self.rows(), [])), \
             patch.object(checker.time, 'monotonic_ns', return_value=1200000000):
            report = checker.run(0, service='balloon-telemetry.service')
        self.assertEqual(report['status'], 'TELEMETRY_NOT_READY')
        self.assertEqual(report['recent_samples'], 0)

    def test_cli_defaults_preserve_existing_deployment(self):
        with patch.object(checker, 'run', return_value={'status': 'TELEMETRY_OK'}) as run, \
             patch('sys.stdout', new_callable=io.StringIO) as output:
            self.assertEqual(checker.main([]), 0)
        run.assert_called_once_with(20, root=checker.ROOT, service='psdk-flight-panel-display')
        self.assertEqual(output.getvalue().splitlines()[-1], 'TELEMETRY_OK')

    def test_cli_forwards_overrides_and_reports_failure(self):
        root = '/var/log/balloon-telemetry'
        with patch.object(checker, 'run', return_value={'status': 'TELEMETRY_PARTIAL'}) as run, \
             patch('sys.stdout', new_callable=io.StringIO) as output:
            self.assertEqual(checker.main(['--timeout', '7', '--service', 'balloon-telemetry.service',
                                           '--log-dir', root]), 2)
        run.assert_called_once_with(7, root=Path(root), service='balloon-telemetry.service')
        self.assertEqual(output.getvalue().splitlines()[-1], 'TELEMETRY_PARTIAL')

    def test_service_option_path_and_wildcard_rejected_without_query(self):
        for name in ('', '--all', '*', '/tmp/fake.service', 'psdk-balloon balloon-telemetry', 'bad\nservice'):
            with self.subTest(name=name), patch.object(checker.subprocess, 'run') as command:
                with self.assertRaises(ValueError):
                    checker.run(0, service=name)
                with self.assertRaises(ValueError):
                    checker.service_state(name)
                command.assert_not_called()


if __name__ == '__main__':
    unittest.main()
