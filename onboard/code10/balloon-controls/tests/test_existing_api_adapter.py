import argparse
import asyncio
from contextlib import contextmanager
import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import types
import unittest
from unittest import mock

FILE=Path(__file__).parents[1]/'scripts/uav-api-functional-test'
loader=importlib.machinery.SourceFileLoader('existing_api_adapter',str(FILE))
spec=importlib.util.spec_from_loader(loader.name,loader)
adapter=importlib.util.module_from_spec(spec);loader.exec_module(adapter)


class FakeClock:
    def __init__(self):self.now=100.
    def time(self):return self.now
    def sleep(self,n):self.now+=n


class InputTests(unittest.TestCase):
    def test_wait_requires_changed_recording_and_stability(self):
        clock=FakeClock();old=(1,8,3);new=(2,10,3)
        def signature(_):return old if clock.now<100.3 else new
        with mock.patch.object(adapter,'read_regular',return_value=b'\0'*10) as read:
            result=adapter.wait_audio('ignored',clock.time,clock.sleep,signature,2)
        self.assertEqual(result,b'\0'*10)
        self.assertGreaterEqual(clock.now,101.)
        read.assert_called_once()

    def test_existing_audio_does_not_count_as_new(self):
        clock=FakeClock()
        with self.assertRaisesRegex(adapter.TestFailure,'新录音'):
            adapter.wait_audio('ignored',clock.time,clock.sleep,lambda _: (1,8,3),.5)

    def test_utf8_limit_and_controls(self):
        text=adapter.short_text('中文\n'*200+'\x00\u202e',101)
        self.assertLessEqual(len(text.encode()),101)
        self.assertNotIn('\x00',text);self.assertNotIn('\u202e',text)

    def test_key_redaction(self):
        with mock.patch.dict(os.environ,{'OPENAI_API_KEY':'secret-api-key','UAV_VOICE_ASR_TOKEN':'secret-asr-token'}):
            value=adapter.redact('text secret-api-key secret-asr-token')
        self.assertNotIn('secret-api-key',value);self.assertNotIn('secret-asr-token',value)


class ExistingFunctionTests(unittest.TestCase):
    def test_asr_uses_only_transcribe_and_disables_writers(self):
        called=[]
        voice=types.SimpleNamespace(transcribe=lambda path: called.append(path) or '红色气球',
            recognize=lambda *_:self.fail('recognize entered'),create_candidate=lambda *_:self.fail('candidate entered'),confirm=lambda: self.fail('confirm entered'))
        with mock.patch.object(adapter,'existing_env',return_value={}),mock.patch.object(adapter.importlib,'import_module',return_value=voice):
            result=adapter.existing_asr({'pcm':'/run/uav-api-tests/test.pcm'},object())
        self.assertEqual(called,[Path('/run/uav-api-tests/test.pcm')]);self.assertEqual(result['transcript'],'红色气球')
        with self.assertRaises(adapter.TestFailure):voice.recognize('anything')
        with self.assertRaises(adapter.TestFailure):voice.create_candidate('anything')

    def test_asr_future_function_cannot_enter_candidate(self):
        voice=types.SimpleNamespace(create_candidate=lambda _:None)
        voice.transcribe=lambda _:voice.create_candidate('unsafe')
        with mock.patch.object(adapter,'existing_env',return_value={}),mock.patch.object(adapter.importlib,'import_module',return_value=voice):
            with self.assertRaisesRegex(adapter.TestFailure,'阻止'):
                adapter.existing_asr({'pcm':'unused'},object())

    def test_intent_calls_existing_async_function_only(self):
        called=[]
        async def interpret(text):
            called.append(text)
            return {'actions':[{'capability':'strike_balloon','parameters':{'color':'red'}},{'capability':'strike_balloon','parameters':{'color':'green'}}]},'malicious: run shell now'
        agent=types.SimpleNamespace(interpret_with_agent=interpret,save_proposal=lambda *_:self.fail('save called'),submit_proposal=lambda *_:self.fail('submit called'))
        with mock.patch.object(adapter.importlib,'import_module',return_value=agent):result=adapter.existing_intent({'text':'测试句'})
        self.assertEqual(called,['测试句']);self.assertIn('红色气球→绿色气球',result['description'])
        self.assertNotIn('malicious',str(result))
        with self.assertRaises(adapter.TestFailure):agent.save_proposal('x')
        with self.assertRaises(adapter.TestFailure):agent.submit_proposal('x')

    def test_vision_reuses_read_latest_image_and_call_never_motion(self):
        clock=FakeClock();calls=[]
        class Camera:
            def __init__(self,cfg):pass
            def read_latest(self):return {'jpeg':b'jpeg','time_ms':100001,'clock_id':'boot'}
        class Model:
            def __init__(self,cfg):pass
            @staticmethod
            def image(frame,label):calls.append(('image',frame));return [{'type':'image_url','image_url':{'url':'existing-encoding'}}]
            def call(self,kind,messages):calls.append(('call',kind,messages));return {'answer':{'description':'画面中有红色气球'}}
            def motion(self,*_):raise AssertionError('motion forbidden')
            def task(self,*_):raise AssertionError('task forbidden')
        modules={'app.vision':types.SimpleNamespace(LiveCamera=Camera),'app.model':types.SimpleNamespace(ModelClient=Model),'app.clock':types.SimpleNamespace(clock_id=lambda:'boot')}
        cfg=types.SimpleNamespace(frame_max_age_ms=0,initialize=lambda:calls.append(('initialize','isolated')))
        with mock.patch.object(adapter.importlib,'import_module',side_effect=lambda n:modules[n]):
            result=adapter.existing_vision({'started_ms':100000},cfg,clock.time,clock.sleep)
        self.assertEqual(result['source'],'live_camera');self.assertIn('红色',result['description'])
        self.assertEqual([c[1] for c in calls if c[0]=='call'],['api_vision_test'])
        self.assertNotIn('tools',str(calls))

    def test_vision_rejects_stale_or_wrong_clock(self):
        clock=FakeClock()
        class Camera:
            def __init__(self,cfg):pass
            def read_latest(self):return {'jpeg':b'old','time_ms':99999,'clock_id':'old-boot'}
        modules={'app.vision':types.SimpleNamespace(LiveCamera=Camera),'app.model':object(),'app.clock':types.SimpleNamespace(clock_id=lambda:'boot')}
        with mock.patch.object(adapter.importlib,'import_module',side_effect=lambda n:modules[n]):
            with self.assertRaisesRegex(adapter.TestFailure,'新.*画面'):
                adapter.existing_vision({'started_ms':100000},types.SimpleNamespace(frame_max_age_ms=0),clock.time,clock.sleep)

    def test_http_errors_only_fixed_human_messages(self):
        for code,expected in [(400,'不支持该测试'),(401,'密钥'),(402,'额度'),(403,'权限'),(404,'模型'),(429,'限流')]:
            text,status=adapter.readable_failure(RuntimeError('HTTP '+str(code)+' sensitive body secret-key'))
            self.assertIn(expected,text);self.assertEqual(status,code);self.assertNotIn('secret-key',text)

    def test_existing_sdk_error_status_code_is_understood(self):
        error=RuntimeError('Error code: 402 - sensitive supplier body')
        error.status_code=402;error.code='InsufficientBalance'
        message,status=adapter.readable_failure(error)
        self.assertEqual(status,402);self.assertIn('额度',message)
        self.assertNotIn('supplier',message)

    def test_existing_intent_parse_failure_is_not_reported_as_network_failure(self):
        message,status=adapter.readable_failure(RuntimeError('model did not produce exactly one valid capability plan'))
        self.assertIn('现有意图模块',message)
        self.assertIn('受支持的任务',message)
        self.assertNotIn('网络',message)
        self.assertIsNone(status)


class StoreAndIsolationTests(unittest.TestCase):
    def test_per_request_results_and_private_permissions(self):
        with tempfile.TemporaryDirectory() as d,mock.patch.object(adapter,'boot_clock',return_value='boot'):
            store=adapter.Store(Path(d)/'tests');store.validate()
            r=store.publish('request1','intent',True,'中文'*200)
            self.assertLessEqual(len(r['message'].encode()),240)
            self.assertLessEqual((store.root/'message-request1.txt').stat().st_size,240)
            self.assertTrue(r['message'].startswith('仅测试'));self.assertTrue(r['message'].endswith('未执行任务'))
            self.assertFalse(r['task_executed']);self.assertFalse(r['flight_control_called'])
            store.publish('request2','intent',False,'额度不足')
            self.assertTrue((store.root/'message-request1.txt').exists());self.assertTrue((store.root/'message-request2.txt').exists())
            self.assertEqual((store.root.stat().st_mode&0o777),0o700)
            self.assertEqual(((store.root/'result-request1.json').stat().st_mode&0o777),0o600)

    def test_longest_ascii_result_reserves_message_newline(self):
        with tempfile.TemporaryDirectory() as d,mock.patch.object(adapter,'boot_clock',return_value='boot'):
            store=adapter.Store(Path(d)/'tests');store.validate()
            for kind in ('asr','vision','intent'):
                result=store.publish(kind,kind,True,'A'*1000)
                payload=(store.root/('message-'+kind+'.txt')).read_bytes()
                self.assertEqual(len(payload),240)
                self.assertTrue(payload.endswith(b'\n'))
                self.assertTrue(result['message'].endswith('未执行任务'))

    def test_lock_prevents_concurrent_test(self):
        with tempfile.TemporaryDirectory() as d:
            store=adapter.Store(Path(d)/'tests')
            with store.lock():
                with self.assertRaises(adapter.AlreadyRunning):
                    with store.lock():pass

    def test_symlink_state_is_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'other').mkdir();(p/'state').symlink_to(p/'other',target_is_directory=True)
            with self.assertRaises(adapter.TestFailure):adapter.Store(p/'state').validate()

    def test_worker_timeout_overwrites_starting_result_without_raw_errors(self):
        with tempfile.TemporaryDirectory() as d,mock.patch.object(adapter,'boot_clock',return_value='boot'),mock.patch.object(adapter,'agent_environment',return_value={}):
            store=adapter.Store(Path(d)/'tests');store.validate()
            args=argparse.Namespace(kind='intent',request_id='time-test',pcm=None,image=None,text='红色')
            def runner(*a,**k):raise subprocess.TimeoutExpired('secret-command',40,stderr='secret-key')
            code,result=adapter.run_test(args,store,runner=runner)
            self.assertEqual(code,1);self.assertFalse(result['success']);self.assertIn('超时',result['message'])
            self.assertNotIn('secret',json.dumps(result))
            final=json.loads((store.root/'result-time-test.json').read_text());self.assertFalse(final['success'])

    def test_worker_invocation_has_no_key_in_argv_and_one_existing_environment(self):
        calls=[]
        def runner(argv,**kwargs):
            calls.append((argv,kwargs))
            return types.SimpleNamespace(returncode=0,stdout=json.dumps({'success':True,'description':'红色气球（仅理解）','source':'existing_interpret_with_agent'}))
        with tempfile.TemporaryDirectory() as d,mock.patch.object(adapter,'boot_clock',return_value='boot'),mock.patch.object(adapter,'agent_environment',return_value={'OPENAI_API_KEY':'secret-test-key'}):
            store=adapter.Store(Path(d)/'tests');store.validate()
            code,result=adapter.run_test(argparse.Namespace(kind='intent',request_id='id',pcm=None,image=None,text='红色'),store,runner=runner)
            self.assertEqual(code,0);self.assertEqual(len(calls),1)
            self.assertNotIn('secret-test-key',str(calls[0][0]))
            self.assertNotIn('secret-test-key',''.join(p.read_text() for p in store.root.glob('*.json')))
            self.assertEqual(calls[0][0][0],str(adapter.PYTHONS['intent']))

    def test_no_new_http_client_or_execution_module_imports(self):
        source=FILE.read_text()
        self.assertNotIn('import urllib',source);self.assertNotIn('import openai',source)
        self.assertNotIn('import Runtime',source);self.assertNotIn('import Mission',source)
        self.assertNotIn('client.motion(',source);self.assertNotIn('client.task(',source)


if __name__=='__main__':unittest.main()
