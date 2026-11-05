"""Fail-open producer helpers; no persistence, hashing, or retries per frame.

The enabled process keeps session filenames and pins even when control expires.
Control only gates side-channel messages. Linux tmpfs is required for capture.
"""
from __future__ import annotations

import array
from collections import Counter
import copy
import ctypes
import hashlib
import math
import os
from pathlib import Path
import socket
import stat
import sys
import threading
import time
import uuid

from hardcase import protocol as p
from hardcase.protocol import parse_frame_filename

DEFAULT_IPC_DIR = Path('/dev/shm/balloon-hardcase')
PIN_PREFIX = '.hardcase-pins-'
PIN_LOCK = '.owner.lock'


def is_tmpfs_fd(fd):
    if sys.platform != 'linux': return False
    libc=ctypes.CDLL(None,use_errno=True)
    result=ctypes.create_string_buffer(256)
    if libc.fstatfs(fd,ctypes.byref(result)) != 0: return False
    return ctypes.c_long.from_buffer(result).value == 0x01021994


class ControlReader:
    def __init__(self, ipc_dir, boot, clock=time.monotonic_ns, require_tmpfs=False):
        self.path=Path(ipc_dir)/'control.json'
        self.boot=boot
        self.clock=clock
        self.next_read=0
        self.cached=None
        self.require_tmpfs=require_tmpfs

    def snapshot(self):
        now=self.clock()
        if now >= self.next_read:
            self.next_read=now+250_000_000
            self.cached=None
            fd=None; directory_fd=None
            try:
                flags=os.O_RDONLY | getattr(os,'O_NOFOLLOW',0) | getattr(os,'O_NONBLOCK',0)
                if self.require_tmpfs:
                    directory_fd=os.open(self.path.parent,flags | getattr(os,'O_DIRECTORY',0))
                    if not is_tmpfs_fd(directory_fd): return None
                    fd=os.open(self.path.name,flags,dir_fd=directory_fd)
                else:
                    fd=os.open(self.path,flags)
                info=os.fstat(fd)
                if not stat.S_ISREG(info.st_mode) or not 0 < info.st_size < 4096: return None
                value=p.loads(os.read(fd,4096),4095)
                p._fields(value,'protocol_version run_id boot_id config_sha256 generation enabled published_mono_ns'.split())
                if (type(value['protocol_version']) is not int or value['protocol_version']!=1
                    or value['boot_id']!=self.boot or not isinstance(value['run_id'],str)
                    or not p.HEX_ID.fullmatch(value['run_id']) or not isinstance(value['config_sha256'],str)
                    or not p.HASH.fullmatch(value['config_sha256']) or not p._integer(value['generation'])
                    or type(value['enabled']) is not bool or not p._integer(value['published_mono_ns'])):
                    return None
                self.cached=value
            except (OSError,ValueError,TypeError):
                return None
            finally:
                if fd is not None: os.close(fd)
                if directory_fd is not None: os.close(directory_fd)
        if self.cached and self.cached['enabled'] and 0 <= now-self.cached['published_mono_ns'] <= 1_000_000_000:
            return dict(self.cached)
        return None


def _pin_lock(fd, exclusive=False):
    # Real capture already requires Linux tmpfs. The other-platform branch only
    # permits existing filesystem unit tests that explicitly mock that guard.
    if sys.platform == 'linux':
        import fcntl
        fcntl.flock(fd, (fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH) | fcntl.LOCK_NB)


def _reap_pin_directory(parent_fd, name):
    """Reclaim only recognized private contents while owning the session lease."""
    directory_fd=lock_fd=None
    try:
        directory_fd=os.open(name,os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,dir_fd=parent_fd)
        info=os.fstat(directory_fd)
        if (info.st_uid!=os.getuid() or stat.S_IMODE(info.st_mode)!=0o700
                or info.st_dev!=os.fstat(parent_fd).st_dev): return False
        # Lockless directories may belong to older workers; never guess they died.
        lock_fd=os.open(PIN_LOCK,os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK,dir_fd=directory_fd)
        lock_info=os.fstat(lock_fd)
        if (not stat.S_ISREG(lock_info.st_mode) or lock_info.st_uid!=os.getuid()
                or stat.S_IMODE(lock_info.st_mode)!=0o600 or lock_info.st_nlink!=1): return False
        _pin_lock(lock_fd,exclusive=True)
        current=os.stat(name,dir_fd=parent_fd,follow_symlinks=False)
        current_lock=os.stat(PIN_LOCK,dir_fd=directory_fd,follow_symlinks=False)
        if ((current.st_dev,current.st_ino)!=(info.st_dev,info.st_ino)
                or (current_lock.st_dev,current_lock.st_ino)!=(lock_info.st_dev,lock_info.st_ino)): return False
        files=[]
        with os.scandir(directory_fd) as entries:
            for entry in entries:
                if len(files)>=128: return False
                item=entry.stat(follow_symlinks=False)
                stem,suffix=os.path.splitext(entry.name)
                if (not stat.S_ISREG(item.st_mode) or item.st_uid!=os.getuid()
                        or item.st_dev!=info.st_dev
                        or (entry.name!=PIN_LOCK and
                            (not p.HEX_ID.fullmatch(stem) or suffix not in ('.jpg','.bmp')))): return False
                files.append(entry.name)
        for filename in files:
            if filename!=PIN_LOCK: os.unlink(filename,dir_fd=directory_fd)
        os.unlink(PIN_LOCK,dir_fd=directory_fd)
        os.rmdir(name,dir_fd=parent_fd)
        return True
    except (OSError,ValueError):
        return False
    finally:
        for fd in (lock_fd,directory_fd):
            if fd is not None: os.close(fd)


def cleanup_stale_pins(parent, session, max_entries=256):
    """One bounded startup sweep; live shared leases and unknown files are kept."""
    if sys.platform!='linux': return
    parent_fd=None
    try:
        parent_fd=os.open(parent,os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        if not is_tmpfs_fd(parent_fd): return
        with os.scandir(parent_fd) as entries:
            for index,entry in enumerate(entries):
                if index>=max_entries: break
                name=entry.name
                if (name.startswith(PIN_PREFIX) and p.HEX_ID.fullmatch(name[len(PIN_PREFIX):])
                        and name!=PIN_PREFIX+session):
                    _reap_pin_directory(parent_fd,name)
    except (OSError,ValueError):
        pass
    finally:
        if parent_fd is not None: os.close(parent_fd)


class PinnedInput:
    def __init__(self,source,session):
        self.source=Path(source)
        self.path=self.source
        self.session=session
        self.fd=None
        self.lock_fd=None
        self.directory=None

    def acquire(self):
        source_fd=None; candidate=None
        try:
            if self.source.is_symlink(): return False
            source_fd=os.open(self.source,os.O_RDONLY | getattr(os,'O_NOFOLLOW',0) | getattr(os,'O_NONBLOCK',0))
            info=os.fstat(source_fd)
            if not stat.S_ISREG(info.st_mode) or info.st_size<=0 or not is_tmpfs_fd(source_fd): return False
            self.directory=self.source.parent/(PIN_PREFIX+self.session)
            self.directory.mkdir(mode=0o700,exist_ok=True)
            directory_info=self.directory.lstat()
            if (not stat.S_ISDIR(directory_info.st_mode) or directory_info.st_dev!=info.st_dev
                or (sys.platform=='linux' and (directory_info.st_uid!=os.getuid() or stat.S_IMODE(directory_info.st_mode)!=0o700))):
                return False
            self.lock_fd=os.open(self.directory/PIN_LOCK,os.O_RDWR | os.O_CREAT |
                                 getattr(os,'O_NOFOLLOW',0) | getattr(os,'O_NONBLOCK',0),0o600)
            lock_info=os.fstat(self.lock_fd)
            if (not stat.S_ISREG(lock_info.st_mode) or lock_info.st_nlink!=1
                    or (sys.platform=='linux' and (lock_info.st_uid!=os.getuid()
                        or stat.S_IMODE(lock_info.st_mode)!=0o600))):
                raise ValueError('unsafe pin lease')
            _pin_lock(self.lock_fd)
            current=self.directory.lstat()
            current_lock=(self.directory/PIN_LOCK).lstat()
            if ((current.st_dev,current.st_ino)!=(directory_info.st_dev,directory_info.st_ino)
                    or (current_lock.st_dev,current_lock.st_ino)!=(lock_info.st_dev,lock_info.st_ino)):
                raise ValueError('pin directory replaced while locking')
            candidate=self.directory/(uuid.uuid4().hex+self.source.suffix)
            os.link(self.source,candidate,follow_symlinks=False)
            self.fd=os.open(candidate,os.O_RDONLY | getattr(os,'O_NOFOLLOW',0))
            pinned=os.fstat(self.fd)
            if (pinned.st_dev,pinned.st_ino)!=(info.st_dev,info.st_ino): raise ValueError('source replaced during pin')
            self.path=candidate
            return True
        except (OSError,ValueError):
            if self.fd is not None: os.close(self.fd); self.fd=None
            if self.lock_fd is not None: os.close(self.lock_fd); self.lock_fd=None
            if candidate is not None:
                try: candidate.unlink()
                except OSError: pass
            return False
        finally:
            if source_fd is not None: os.close(source_fd)

    def close(self):
        if self.fd is not None:
            try: os.close(self.fd)
            except OSError: pass
            self.fd=None
        if self.path!=self.source:
            try: self.path.unlink()
            except OSError: pass
        if self.lock_fd is not None:
            try: os.close(self.lock_fd)
            except OSError: pass
            self.lock_fd=None
        if self.directory is not None:
            if sys.platform=='linux':
                parent_fd=None
                try:
                    parent_fd=os.open(self.directory.parent,os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
                    _reap_pin_directory(parent_fd,self.directory.name)
                except OSError: pass
                finally:
                    if parent_fd is not None: os.close(parent_fd)
            else:
                try:
                    (self.directory/PIN_LOCK).unlink(missing_ok=True)
                    self.directory.rmdir()
                except OSError: pass


def _safe_raw(value):
    if isinstance(value,dict): return {str(k):_safe_raw(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [_safe_raw(v) for v in value]
    if isinstance(value,Path): return str(value)
    if isinstance(value,float) and not math.isfinite(value): return {'nonfinite':repr(value)}
    if value is None or type(value) in (str,int,float,bool): return value
    return repr(value)


def normalize_boxes(records,width,height,origin,stage):
    result=[]
    for index,record in enumerate(records):
        box=dict(class_id=record.get('class_id'),class_name=record.get('class_name'),
                 confidence=record.get('confidence'),bbox_xyxy=copy.deepcopy(record.get('bbox_xyxy')),
                 origin='tile' if record.get('detection_pass')=='far_tile' else origin,
                 stage=stage,source_index=index)
        roi=record.get('tile_region_xyxy')
        if roi is not None:
            box['tile_roi']=list(roi); box['tile_to_full']=[1,0,roi[0],0,1,roi[1]]
        coords=box['bbox_xyxy']
        invalid=[]
        for key,check in (('class_id',p._integer),('class_name',lambda v:p._text(v,128)),
                          ('confidence',lambda v:p._number(v,0,1))):
            if not check(box[key]):
                box[key]=None; invalid.append(key)
        # Out-of-frame boxes retain exact coordinates and never become sampling evidence.
        if (isinstance(coords,list) and len(coords)==4 and all(p._number(v,-1e9,1e9) for v in coords)
            and width is not None and height is not None):
            x1,y1,x2,y2=coords
            if not (0<=x1<x2<=width and 0<=y1<y2<=height):
                invalid.append('out_of_bounds_or_nonpositive_area')
        else:
            box['bbox_xyxy']=None; invalid.append('bbox_xyxy')
        if invalid:
            box.update(valid=False,invalid_reason=','.join(invalid))
        result.append(box)
    return result


class Producer:
    def __init__(self,enabled,ipc_dir):
        self.enabled=bool(enabled)
        self.ipc_dir=Path(ipc_dir)
        self.session_id=uuid.uuid4().hex
        self.counters=Counter()
        self.reader=None; self.socket=None; self.boot=None
        if not self.enabled: return
        try:
            self.boot=p.boot_id()
            directory_fd=None
            try:
                directory_fd=os.open(self.ipc_dir,os.O_RDONLY | getattr(os,'O_DIRECTORY',0) | getattr(os,'O_NOFOLLOW',0))
                if not is_tmpfs_fd(directory_fd): raise ValueError('IPC directory is not tmpfs')
            except FileNotFoundError:
                # Allow the collector to start later; each control read rechecks tmpfs.
                pass
            finally:
                if directory_fd is not None: os.close(directory_fd)
            self.reader=ControlReader(self.ipc_dir,self.boot,require_tmpfs=True)
            # Opening an unbound datagram socket does not wait for a collector.
            self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
            self.socket.setblocking(False)
        except Exception:
            self.counters['initialization_unavailable']+=1

    def snapshot(self):
        try: return self.reader.snapshot() if self.reader else None
        except Exception:
            self.counters['control_unavailable']+=1
            return None

    def _header(self,snapshot,kind,decoder,frame):
        return dict(protocol_version=1,kind=kind,run_id=snapshot['run_id'],boot_id=snapshot['boot_id'],
                    generation=snapshot['generation'],decoder_session_id=decoder,frame_id=frame,
                    producer_mono_ns=time.monotonic_ns(),metadata_mode='inline')

    def send(self,channel,header,image_fd):
        metadata_fd=None; writable=None
        try:
            if self.socket is None: return False
            p.validate_header(header); p.validate_payload(header,header['payload'])
            fds=[image_fd]
            try:
                raw=p.dumps(header)
            except ValueError:
                metadata=p.dumps(header['payload'],p.MAX_METADATA_BYTES)
                if header['kind']!='DetectionResult' or not hasattr(os,'memfd_create'): raise
                import fcntl
                writable=os.memfd_create('hardcase-result',os.MFD_CLOEXEC | os.MFD_ALLOW_SEALING)
                offset=0
                while offset<len(metadata):
                    written=os.write(writable,metadata[offset:])
                    if written<=0: raise OSError('short metadata write')
                    offset+=written
                fcntl.fcntl(writable,fcntl.F_ADD_SEALS,fcntl.F_SEAL_WRITE | fcntl.F_SEAL_GROW | fcntl.F_SEAL_SHRINK | fcntl.F_SEAL_SEAL)
                metadata_fd=os.open('/proc/self/fd/'+str(writable),os.O_RDONLY | os.O_CLOEXEC)
                os.close(writable); writable=None
                wire={k:v for k,v in header.items() if k!='payload'}
                wire.update(metadata_mode='fd',metadata_bytes=len(metadata))
                p.validate_header(wire); raw=p.dumps(wire); fds.append(metadata_fd)
            self.socket.sendmsg([raw],[(socket.SOL_SOCKET,socket.SCM_RIGHTS,array.array('i',fds))],
                                getattr(socket,'MSG_DONTWAIT',0),str(self.ipc_dir/channel))
            self.counters['sent']+=1
            return True
        except Exception:
            self.counters['send_dropped']+=1
            return False
        finally:
            for fd in (metadata_fd,writable):
                if fd is not None:
                    try: os.close(fd)
                    except OSError: pass

    def frame_ready(self,path,frame_id,width,height,source_ns,wall_ms,frame_ts_ms,encode_ms,snapshot=None):
        fd=None
        try:
            if not self.enabled or Path(path).suffix!='.jpg': return
            # publish_frame freezes the snapshot before encoding; no run switch here.
            if snapshot is None: return
            fd=os.open(path,os.O_RDONLY | getattr(os,'O_NOFOLLOW',0))
            if not is_tmpfs_fd(fd): raise ValueError('frame not on tmpfs')
            header=self._header(snapshot,'FrameReady',self.session_id,frame_id)
            header['payload']=dict(width=width,height=height,source_mono_ns=source_ns,wall_utc_ms=wall_ms,
                frame_ts_ms=frame_ts_ms,encode_ms=encode_ms,codec='jpeg',original_path=str(path))
            self.send('frames.sock',header,fd)
        except Exception:
            self.counters['frame_unavailable']+=1
        finally:
            if fd is not None: os.close(fd)

    def close(self):
        if self.socket is not None: self.socket.close(); self.socket=None


class WorkerProducer(Producer):
    def __init__(self,enabled,ipc_dir,config,detector):
        super().__init__(enabled,ipc_dir)
        self.sequence=0; self.sequence_lock=threading.Lock()
        self.pin_cleanup_parents=set()
        self.config={}; self.model_hashes={}; self.model_reasons={}; self.class_map={}
        if not enabled: return
        # Hash model files and configuration exactly once, never on a request path.
        self.config=_safe_raw(config)
        self.config_hash=hashlib.sha256(p.dumps(self.config,p.MAX_METADATA_BYTES)).hexdigest()
        paths={'pytorch':config['weights']} if config.get('weights') is not None else {}
        if config.get('active_detector') == 'pixel':
            self.model_reasons['color_threshold']='not applicable: pixel detector has no model weights'
        primary=getattr(detector,'primary',detector)
        if getattr(primary,'engine_path',None): paths['tensorrt']=primary.engine_path
        for backend,path in paths.items():
            try:
                digest=hashlib.sha256()
                with Path(path).open('rb') as stream:
                    for block in iter(lambda:stream.read(1024*1024),b''): digest.update(block)
                self.model_hashes[backend]=digest.hexdigest()
            except (OSError,TypeError): self.model_reasons[backend]='model file unavailable at initialization'
        names=getattr(detector,'class_names',None) or getattr(getattr(detector,'model',None),'names',{}) or {}
        if isinstance(names,list): names=dict(enumerate(names))
        self.class_map={str(k):str(v) for k,v in names.items()}
        if not self.class_map and 'tensorrt' in paths: self.class_map={'0':'balloon'}

    def begin(self,source,request_seq):
        with self.sequence_lock:
            sequence=self.sequence; self.sequence+=1
            parent=Path(source).parent
            sweep=self.enabled and parent not in self.pin_cleanup_parents and len(self.pin_cleanup_parents)<32
            if sweep: self.pin_cleanup_parents.add(parent)
        if sweep: cleanup_stale_pins(parent,self.session_id)
        capture=RequestCapture(self,source,request_seq,sequence,self.snapshot())
        return capture


class RequestCapture:
    def __init__(self,producer,source,request_seq,inference_seq,snapshot):
        self.producer=producer; self.source=Path(source); self.path=self.source
        self.request_seq=request_seq; self.inference_seq=inference_seq
        self.worker_session_id=producer.session_id; self.snapshot=snapshot
        self.request_start_ns=time.monotonic_ns(); self.inference_start_ns=None; self.inference_end_ns=None
        self.pin=None; self.full=[]; self.tiles=[]; self.passes=[]
        self.width=None; self.height=None
        if producer.enabled:
            pin=PinnedInput(source,producer.session_id)
            if pin.acquire(): self.pin=pin; self.path=pin.path
            else: pin.close(); producer.counters['pin_unavailable']+=1

    def observe(self,records,origin,detector,roi=None):
        if not self.producer.enabled or self.snapshot is None or self.pin is None: return
        copied=copy.deepcopy(records)
        if roi is not None:
            for record in copied:
                coords=record.get('bbox_xyxy')
                record['tile_region_xyxy']=list(roi); record['detection_pass']='far_tile'
                if isinstance(coords,list) and len(coords)==4:
                    record['bbox_xyxy']=[coords[0]+roi[0],coords[1]+roi[1],coords[2]+roi[0],coords[3]+roi[1]]
        (self.full if origin=='full' else self.tiles).extend(copied)
        actual=getattr(detector,'fallback',None) or getattr(detector,'primary',detector)
        self.passes.append(dict(origin=origin,tile_roi=list(roi) if roi is not None else None,
            backend=getattr(detector,'backend_name','unknown'),
            timings=copy.deepcopy(getattr(actual,'last_timings_ms',None)),
            software_auto_exposure=copy.deepcopy(getattr(actual,'last_auto_exposure_info',None)),
            tile_encoding={'codec':'jpeg','quality':92} if roi is not None else None,
            input_transform={'max_decode_divisor':getattr(actual,'max_decode_divisor',None),
                             'image_size':getattr(actual,'image_size',None)}))

    def finish(self,business,detector):
        try:
            identity=parse_frame_filename(self.source)
            if self.pin is None or self.snapshot is None or identity is None or identity[0]=='unknown' or self.source.suffix!='.jpg':
                return False
            producer=self.producer
            backend=getattr(detector,'backend_name','unknown')
            model_backend='pytorch' if backend.startswith('pytorch') else backend
            box_stage='post_pixel_filter' if model_backend=='color_threshold' else 'post_model_nms'
            result_stage='post_pixel_filter' if model_backend=='color_threshold' else 'post_tile_merge'
            model_hash=producer.model_hashes.get(model_backend)
            reasons={}
            if model_hash is None: reasons['model_sha256']=producer.model_reasons.get(model_backend,'backend artifact unavailable at initialization')
            effective=getattr(getattr(detector,'fallback',None) or detector,'model',None)
            predictor=getattr(effective,'predictor',None)
            iou=getattr(getattr(predictor,'args',None),'iou',None)
            if model_backend=='tensorrt': iou=.7
            if iou is None: reasons['online_iou']='effective backend IoU unavailable'
            conf=producer.config.get('conf')
            if conf is None: reasons['online_conf']='effective confidence unavailable'
            header=producer._header(self.snapshot,'DetectionResult',*identity)
            header.update(worker_session_id=self.worker_session_id,request_seq=self.request_seq,inference_seq=self.inference_seq)
            raw=copy.deepcopy(business)
            raw.update(full_detections=self.full,tile_detections=self.tiles)
            # Existing business payload can retain ok=True when a later target write fails.
            ok=business.get('ok') is True and not business.get('error')
            header['payload']=dict(ok=ok,error=None if ok else {'type':'DetectionError','message':str(business.get('error','unknown error'))[:4096]},
                image_width=self.width,image_height=self.height,source_path=str(self.source),pin_used=True,
                request_start_ns=self.request_start_ns,inference_start_ns=self.inference_start_ns,
                inference_end_ns=self.inference_end_ns or time.monotonic_ns(),backend=backend,model_sha256=model_hash,
                config_sha256=producer.config_hash,online_conf=conf,online_iou=iou,class_map=producer.class_map,
                detections=normalize_boxes(business.get('detections',[]),self.width,self.height,'full',result_stage),
                full_detections=normalize_boxes(self.full,self.width,self.height,'full',box_stage),
                tile_detections=normalize_boxes(self.tiles,self.width,self.height,'tile',box_stage),
                selected_target=normalize_boxes([business['best_balloon']],self.width,self.height,'full','selection')[0] if business.get('best_balloon') else None,
                detection_stage=result_stage,timings={k:business.get(k) for k in ('latency_ms','far_tile_latency_ms','wrapper_latency_ms')},
                capture_ts_ms=business.get('capture_ts_ms'),request_ts_ms=business.get('request_ts_ms'),
                raw_result=_safe_raw(raw),unavailable_reasons=reasons,passes=_safe_raw(self.passes),
                model_info={'hashes':producer.model_hashes,'configuration':producer.config})
            header['producer_mono_ns']=time.monotonic_ns()
            return producer.send('results.sock',header,self.pin.fd)
        except Exception:
            self.producer.counters['result_unavailable']+=1
            return False

    def close(self):
        if self.pin is not None: self.pin.close()


def restore_source_references(value,pinned,original):
    """Keep existing business metadata naming the caller's original source."""
    if isinstance(value,dict):
        for key,item in value.items(): value[key]=restore_source_references(item,pinned,original)
    elif isinstance(value,list):
        for index,item in enumerate(value): value[index]=restore_source_references(item,pinned,original)
    elif isinstance(value,str) and value==str(pinned): return str(original)
    return value
