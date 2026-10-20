import copy,io,json,tempfile,time,unittest,sys
from pathlib import Path
from unittest.mock import patch
from PIL import Image
from app.config import Config,path,ROOT
from app.mission import Mission
from app.control import validate_motion,healthy
from app.vision import LiveCamera
from app.clock import clock_id
from app.io import now_ms,atomic_json
from app.model import ModelClient
from app.asr import Recognizer

def cfg(directory):
    c=Config({'UAV_STATE_DIR':str(directory),'OPENAI_API_KEY':'Your API Key'});c.initialize();c.frame_dir=Path(directory)/'frames';c.frame_dir.mkdir();c.metadata=Path(directory)/'latest.tsv';return c

def active():
    m=Mission();r=m.interrupt();m.propose({'intent':'execute','targets':['red','purple']},r);receipt=copy.deepcopy(m.pending);assert m.confirm(receipt);return m

class Missions(unittest.TestCase):
    def test_initial_plan_requires_confirmation(self):
        m=Mission();r=m.interrupt();m.propose({'intent':'execute','targets':['red']},r);self.assertEqual(m.status,'awaiting_confirmation');self.assertFalse(m.approved_nonce)
    def test_confirm_must_match_current_plan(self):
        m=Mission();r=m.interrupt();m.propose({'intent':'execute','targets':['red']},r);bad=copy.deepcopy(m.pending);bad['nonce']='old';self.assertFalse(m.confirm(bad));self.assertTrue(m.confirm(m.pending.copy()))
    def test_ignored_old_api_result(self):
        m=Mission();old=m.interrupt();m.interrupt();self.assertFalse(m.propose({'intent':'execute','targets':['red']},old))
    def test_reject_pauses_and_requires_new_confirmation(self):
        m=active();m.reject();self.assertEqual(m.status,'paused');self.assertEqual(m.targets,['red','purple']);self.assertFalse(m.approved_nonce)
    def test_modify_while_waiting_replaces_confirmation(self):
        m=Mission();r=m.interrupt();m.propose({'intent':'execute','targets':['red']},r);old=m.pending.copy();r=m.interrupt();m.propose({'intent':'revise','targets':['green']},r);self.assertFalse(m.confirm(old));self.assertEqual(m.pending['targets'],['green'])
    def test_completed_prefix_cannot_change(self):
        m=active();m.mark_complete({'revision':m.revision,'target_index':0});r=m.interrupt()
        with self.assertRaises(ValueError):m.propose({'intent':'revise','targets':['green']},r)
        m.propose({'intent':'revise','targets':['red','green']},r);self.assertEqual(m.completed,['red'])
    def test_completion_requires_operator_event(self):
        m=active();self.assertFalse(m.mark_complete({'revision':999}));self.assertEqual(m.completed,[]);self.assertTrue(m.mark_complete({'revision':m.revision,'target_index':0}));self.assertEqual(m.target,'purple')
    def test_pause_preserves_plan(self):
        m=active();r=m.interrupt();m.propose({'intent':'pause','targets':['red','purple']},r);self.assertEqual(m.status,'paused');self.assertEqual(m.targets,['red','purple'])
    def test_unsupported_target_rejected(self):
        m=Mission();r=m.interrupt()
        with self.assertRaises(ValueError):m.propose({'intent':'execute','targets':['car']},r)
    def test_summary_fits_sdk_floating_window(self):
        m=Mission();r=m.interrupt();m.propose({'intent':'execute','targets':['red','green','purple']},r);self.assertLess(len(m.summary().encode()),255)

class Controls(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.c=cfg(self.tmp.name);self.m=active();self.d={'plan':['red','purple'],'target':'red','mode':'approach','visible':['red'],'bbox':[.4,.4,.6,.6],'velocity':{'x':.1,'y':0.,'z':0.,'yaw':0.},'duration_s':.23}
    def test_velocity_not_replaced_and_duration_rounded_down(self):
        v,ms=validate_motion(self.d,self.m,self.c);self.assertEqual(v,self.d['velocity']);self.assertEqual(ms,200)
    def test_target_mismatch_rejected(self):
        self.d['target']='green'
        with self.assertRaises(ValueError):validate_motion(self.d,self.m,self.c)
    def test_nonfinite_rejected(self):
        self.d['velocity']['x']=float('nan')
        with self.assertRaises(ValueError):validate_motion(self.d,self.m,self.c)
    def test_velocity_limit_rejected(self):
        self.d['velocity']['x']=.6
        with self.assertRaises(ValueError):validate_motion(self.d,self.m,self.c)
    def test_nonzero_hold_rejected(self):
        self.d['mode']='hold'
        with self.assertRaises(ValueError):validate_motion(self.d,self.m,self.c)
    def test_wrong_search_sign_rejected(self):
        self.d['mode']='search_left';self.d['velocity']={'x':0.,'y':0.,'z':0.,'yaw':2.}
        with self.assertRaises(ValueError):validate_motion(self.d,self.m,self.c)
    def test_old_telemetry_rejected(self):
        self.assertFalse(healthy({'time_ms':now_ms()-1000},self.c))

class Frames(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.c=cfg(self.tmp.name);self.reader=LiveCamera(self.c)
    def write_frame(self,ident=1,stamp=None,token=None,extension='.jpg'):
        stamp=now_ms() if stamp is None else stamp;token=clock_id() if token is None else token
        f=self.c.frame_dir/f'frame_{ident:012d}{extension}';Image.new('RGB',(64,64),'green').save(f)
        f.with_suffix(extension+'.tsv').write_text(f'{stamp}\t{f.name}\t{token}\n')
        self.c.metadata.write_text(f'{ident}\t{stamp}\t2\t{f}\t{token}\n');return f
    def test_existing_five_field_protocol(self):
        self.write_frame();self.assertIsNotNone(self.reader.read_latest())
    def test_duplicate_not_reused_as_new_frame(self):
        self.write_frame();self.reader.read_latest();self.assertIsNone(self.reader.read_latest())
    def test_frame_clock_from_previous_boot_rejected(self):
        self.write_frame(token='mono-v1:wrong-boot');self.assertIsNone(self.reader.read_latest())
    def test_no_fallback_to_mtime(self):
        f=self.write_frame();f.with_suffix('.jpg.tsv').unlink();self.assertIsNone(self.reader.read_latest())
    def test_stale_frame_rejected(self):
        self.write_frame(stamp=now_ms()-1000);self.assertIsNone(self.reader.read_latest())
    def test_sidecar_and_index_must_match(self):
        f=self.write_frame();f.with_suffix('.jpg.tsv').write_text(f'{now_ms()-50}\t{f.name}\t{clock_id()}\n');self.assertIsNone(self.reader.read_latest())
    def test_bmp_input_is_supported(self):
        self.write_frame(extension='.bmp');frame=self.reader.read_latest();self.assertEqual(frame['jpeg'][:2],b'\xff\xd8')
    def test_external_path_escape_rejected(self):
        outside=Path(self.tmp.name)/'other.jpg';Image.new('RGB',(64,64)).save(outside);self.c.metadata.write_text(f'1\t{now_ms()}\t1\t{outside}\t{clock_id()}\n');self.assertIsNone(self.reader.read_latest())
    def test_legacy_untagged_clock_is_rejected(self):
        self.c.metadata.write_text(f'1\t{now_ms()}\t1\tframe.jpg\n');self.assertIsNone(self.reader.read_latest())

class Configuration(unittest.TestCase):
    def test_placeholder_key_does_not_make_request(self):
        with tempfile.TemporaryDirectory() as tmp:
            c=cfg(tmp)
            with patch('urllib.request.urlopen',side_effect=AssertionError('network forbidden')):
                with self.assertRaises(ValueError):ModelClient(c).call('task',[])
    def test_relative_path_uses_package_location(self):self.assertEqual(path('config/example'),ROOT/'config/example')
    def test_audio_checksum_rejected_before_decoder(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):Recognizer(cfg(tmp)).decode_psdk(b'bad',8000,'wrong')
    def test_motion_request_has_images_but_no_local_boxes(self):
        with tempfile.TemporaryDirectory() as tmp:
            client=ModelClient(cfg(tmp));captured=[];client.call=lambda k,m:captured.append(m)
            frame={'time_ms':now_ms(),'jpeg':b'jpeg'};client.motion({'targets':['red'],'target':'red','completed':[]},[frame,frame,frame],[frame],{'velocity_neu':[0,0,0]},[],'camera')
            body=captured[0][1]['content'];images=[x for x in body if x['type']=='image_url'];self.assertEqual(len(images),4);self.assertIn('HISTORICAL GREEN IMAGE',body[1]['text']);self.assertNotIn('detections',json.dumps(body));self.assertNotIn('target_bearing',json.dumps(body))

class ApiRecords(unittest.TestCase):
    def test_success_saves_reply_and_real_elapsed_time_without_key(self):
        with tempfile.TemporaryDirectory() as tmp:
            c=cfg(tmp);c.key='test-private-key';client=ModelClient(c)
            response={'id':'response-fixture','model':'fixture-model','choices':[{'message':{'content':'{"intent":"execute","targets":["red"]}'}}]}
            with patch('urllib.request.urlopen',return_value=io.BytesIO(json.dumps(response).encode())) as transport:
                result=client.call('task',[{'role':'user','content':'test'}])
            self.assertEqual(result['answer']['targets'],['red']);self.assertEqual(transport.call_count,1)
            directory=c.state/'api-transcript'/result['call_id'];self.assertTrue((directory/'timing.json').is_file())
            for f in directory.glob('*.json'):self.assertNotIn(c.key,f.read_text())
    def test_malformed_reply_is_preserved_and_never_a_control(self):
        with tempfile.TemporaryDirectory() as tmp:
            c=cfg(tmp);c.key='test-private-key';client=ModelClient(c)
            response={'id':'bad','choices':[{'message':{'content':'not JSON'}}]}
            with patch('urllib.request.urlopen',return_value=io.BytesIO(json.dumps(response).encode())):
                with self.assertRaises(RuntimeError):client.call('motion',[])
            self.assertEqual(len(list((c.state/'api-transcript').glob('*/response.json'))),1)

class LateReplies(unittest.TestCase):
    def test_interrupted_motion_reply_cannot_create_segment(self):
        from app.runtime import Runtime
        from concurrent.futures import Future
        with tempfile.TemporaryDirectory() as tmp:
            runtime=Runtime(cfg(tmp));runtime.mission=active();old=runtime.mission.revision
            f=Future();f.set_result({'answer':{},'call_id':'old'})
            runtime.motion_future={'future':f,'revision':old,'frames':[]};runtime.mission.interrupt()
            runtime.completed_requests({},None);self.assertIsNone(runtime.segment)
            runtime.pool.shutdown(wait=False,cancel_futures=True)
    def test_late_task_response_after_pause_is_discarded(self):
        from app.runtime import Runtime
        from concurrent.futures import Future
        with tempfile.TemporaryDirectory() as tmp:
            runtime=Runtime(cfg(tmp));old=runtime.mission.interrupt();f=Future()
            f.set_result(('old command',{'answer':{'intent':'execute','targets':['red']},'call_id':'old'}))
            runtime.task_future=(f,old);runtime.mission.pause();runtime.completed_requests({},None)
            self.assertEqual(runtime.mission.status,'paused');self.assertIsNone(runtime.mission.pending)
            runtime.pool.shutdown(wait=False,cancel_futures=True)

if __name__=='__main__':unittest.main()
