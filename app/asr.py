from __future__ import annotations
import ctypes, ctypes.util, hashlib, subprocess, wave, uuid
from pathlib import Path

class Recognizer:
    def __init__(self,cfg):self.cfg=cfg

    def decode_psdk(self, data, bitrate, expected_md5):
        if hashlib.md5(data).hexdigest()!=expected_md5:raise ValueError('audio checksum mismatch')
        if type(bitrate) is not int or bitrate not in range(8000,64001,8000):raise ValueError('unsupported Opus bitrate')
        # PSDK speaker sample: CBR, 40-byte packets at 8kbps, 16kHz mono.
        size=40*(bitrate//8000)
        if not data or len(data)%size or len(data)>2*1024*1024:raise ValueError('invalid PSDK voice payload')
        name=ctypes.util.find_library('opus')
        if not name:raise RuntimeError('libopus is not installed')
        lib=ctypes.CDLL(name);lib.opus_decoder_create.argtypes=[ctypes.c_int,ctypes.c_int,ctypes.POINTER(ctypes.c_int)];lib.opus_decoder_create.restype=ctypes.c_void_p
        lib.opus_decode.argtypes=[ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int,ctypes.POINTER(ctypes.c_short),ctypes.c_int,ctypes.c_int];lib.opus_decode.restype=ctypes.c_int
        lib.opus_decoder_destroy.argtypes=[ctypes.c_void_p];lib.opus_decoder_destroy.restype=None
        error=ctypes.c_int();decoder=lib.opus_decoder_create(16000,1,ctypes.byref(error));pcm=bytearray();buf=(ctypes.c_short*5760)()
        if not decoder or error.value:raise RuntimeError('Opus decoder init failed')
        try:
            for offset in range(0,len(data),size):
                n=lib.opus_decode(decoder,data[offset:offset+size],size,buf,5760,0)
                if n<0:raise ValueError('invalid Opus packet')
                # explicit little-endian PCM regardless of host byte order
                for value in buf[:n]:pcm.extend(int(value).to_bytes(2,'little',signed=True))
                if len(pcm)>16000*2*30:raise ValueError('voice command longer than 30 seconds')
        finally:lib.opus_decoder_destroy(decoder)
        return bytes(pcm)

    def recognize_event(self,event):
        file=(self.cfg.state/'ipc/audio'/event['file']).resolve()
        if file.parent!=(self.cfg.state/'ipc/audio').resolve():raise ValueError('invalid audio path')
        pcm=self.decode_psdk(file.read_bytes(),event['bitrate'],event['md5'])
        return self.recognize_pcm(pcm)

    def recognize_pcm(self,pcm):
        if not self.cfg.whisper.is_file() or not self.cfg.whisper_model.is_file():raise RuntimeError('install whisper.cpp and multilingual base model first')
        name=self.cfg.state/'asr'/uuid.uuid4().hex
        with wave.open(str(name.with_suffix('.wav')),'wb') as f:f.setnchannels(1);f.setsampwidth(2);f.setframerate(16000);f.writeframes(pcm)
        result=subprocess.run([str(self.cfg.whisper),'-m',str(self.cfg.whisper_model),'-f',str(name.with_suffix('.wav')),'-l','zh','-nt','-np','-otxt','-of',str(name)],capture_output=True,timeout=120)
        if result.returncode:raise RuntimeError('whisper.cpp failed')
        text=name.with_suffix('.txt').read_text().strip()
        if not text:raise ValueError('empty ASR transcript')
        return text
