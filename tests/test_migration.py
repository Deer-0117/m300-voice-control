import os
from pathlib import Path
import tempfile
import unittest
from concurrent.futures import Future
from unittest.mock import patch
from app.config import Config
from app.runtime import Runtime
from app.clock import clock_id
from app.io import now_ms, atomic_json
from app.asr import simplified_converter


class MigrationRegressionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.cfg=Config({'UAV_STATE_DIR':self.tmp.name,'OPENAI_API_KEY':'Your API Key'})
        self.runtime=Runtime(self.cfg)
        self.runtime.boot='migration-test-bridge'
        self.addCleanup(self.runtime.pool.shutdown,wait=True,cancel_futures=True)

    def request(self, running=False):
        future=Future()
        if running:future.set_running_or_notify_cancel()
        self.runtime.motion_future={'future':future,'revision':1,'frames':[]}
        return future

    def event(self, kind):
        event={'bridge_boot':self.runtime.boot,'clock_id':clock_id(),'time_ms':now_ms(),
               'id':'migration-'+kind,'kind':kind,'audio_id':42}
        atomic_json(self.cfg.state/'ipc/events/0001.json',event)
        self.runtime.events()

    def test_clear_cancels_future_and_detaches_record(self):
        future=self.request()
        self.runtime.clear_motion()
        self.assertTrue(future.cancelled())
        self.assertIsNone(self.runtime.motion_future)

    def test_running_request_detached_without_claiming_http_was_stopped(self):
        future=self.request(running=True)
        self.runtime.clear_motion()
        self.assertFalse(future.cancelled())
        self.assertIsNone(self.runtime.motion_future)
        future.set_result({'answer':{'velocity':{'x':.5}},'call_id':'late'})
        self.runtime.completed_requests({},None)
        self.assertIsNone(self.runtime.segment)

    def test_pause_while_request_pending(self):
        future=self.request()
        self.event('pause')
        self.assertTrue(future.cancelled())
        self.assertEqual(self.runtime.mission.status,'paused')

    def test_new_voice_while_request_pending(self):
        future=self.request()
        self.event('voice_start')
        self.assertTrue(future.cancelled())
        self.assertEqual(self.runtime.mission.status,'analyzing_task')
        self.assertEqual(self.runtime.voice_revision[0],42)

    def test_release_while_request_pending(self):
        future=self.request()
        self.event('release')
        self.assertTrue(future.cancelled())
        self.assertEqual(self.runtime.mission.status,'paused')

    def test_cleanup_during_exit_while_request_pending(self):
        future=self.request()
        self.cfg.key='unit-test-no-api-call'
        with patch.object(self.runtime.camera,'start'), patch.object(self.runtime.camera,'close'), \
                patch.object(self.runtime,'tick',side_effect=RuntimeError('controlled exit')):
            with self.assertRaisesRegex(RuntimeError,'controlled exit'):
                self.runtime.run()
        self.assertTrue(future.cancelled())
        self.assertIsNone(self.runtime.segment)
        self.assertTrue((self.cfg.state/'ipc/command.json').is_file())

    def test_task_worker_keeps_its_own_revision_when_next_voice_arrives(self):
        jobs=[]
        def queued(function):
            jobs.append(function)
            future=Future();future.set_running_or_notify_cancel()
            return future
        for index,kind,audio_id in ((1,'voice_start',42),(2,'voice_finish',42),(3,'voice_start',43)):
            atomic_json(self.cfg.state/f'ipc/events/{index:04d}.json',
                        {'bridge_boot':self.runtime.boot,'clock_id':clock_id(),'time_ms':now_ms(),
                         'id':str(index),'kind':kind,'audio_id':audio_id})
        with patch.object(self.runtime.pool,'submit',side_effect=queued):self.runtime.events()
        self.assertEqual(self.runtime.mission.revision,2)
        with patch.object(self.runtime.asr,'recognize_event',return_value='red'), \
                patch.object(self.runtime,'wait_for_stability',side_effect=RuntimeError('old request')) as wait:
            with self.assertRaisesRegex(RuntimeError,'old request'):jobs[0]()
        wait.assert_called_once_with(1)

    def test_simplified_conversion(self):
        self.assertEqual(simplified_converter().convert('撞擊紅色氣球，天氣不錯'),
                         '撞击红色气球，天气不错')

    def test_external_config_file(self):
        config_file=Path(self.tmp.name)/'external.env'
        config_file.write_text('OPENAI_API_KEY=unit-test-placeholder\nUAV_ENABLE_FLIGHT=0\n',encoding='utf-8')
        with patch.dict(os.environ,{'M300_ENV_FILE':str(config_file)},clear=True):
            config=Config()
        self.assertEqual(config.key,'unit-test-placeholder')
        self.assertFalse(config.flight_enabled)
