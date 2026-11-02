"""Small streaming Matroska/MJPEG writer with millisecond presentation times.

No encoder process or extra Jetson dependency. One independently decodable JPEG
per cluster keeps a truncated recording readable. No fabricated fixed FPS.
"""
import struct
import os
from pathlib import Path

import cv2


def element(tag, data):
    size = len(data)
    for width in range(1, 9):
        if size < (1 << (7 * width)) - 1:
            return bytes.fromhex(tag) + ((1 << (7 * width)) | size).to_bytes(width, 'big') + data
    raise ValueError('element too large')


def uint(tag, value):
    return element(tag, value.to_bytes(max(1, (value.bit_length()+7)//8), 'big'))


class TimestampVideoWriter:
    def __init__(self, path: Path, width: int, height: int):
        self.width, self.height = width, height
        self.first = self.last = None
        self.stream = path.open('w+b')
        header = (uint('4286', 1) + uint('42F7', 1) + uint('42F2', 4) + uint('42F3', 8)
                  + element('4282', b'matroska') + uint('4287', 4) + uint('4285', 2))
        self.stream.write(element('1A45DFA3', header))
        self.stream.write(bytes.fromhex('18538067') + b'\x01' + b'\xff'*7)
        info = uint('2AD7B1', 1000000) + element('4D80', b'balloon') + element('5741', b'balloon')
        info += element('4489', struct.pack('>d', 0.0))
        packed = element('1549A966', info)
        self.duration_offset = self.stream.tell() + len(packed) - 8
        self.stream.write(packed)
        video = uint('B0', width) + uint('BA', height)
        track = uint('D7', 1) + uint('73C5', 1) + uint('83', 1) + uint('9C', 0)
        track += element('86', b'V_MJPEG') + element('E0', video)
        self.stream.write(element('1654AE6B', element('AE', track)))

    def write(self, frame, timestamp_ms):
        if frame.shape[:2] != (self.height, self.width):
            raise ValueError('frame geometry changed')
        if timestamp_ms < 0 or (self.last is not None and timestamp_ms <= self.last):
            raise ValueError('frame timestamps must be strictly increasing')
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise OSError('JPEG encoding failed')
        if self.first is None:
            self.first = timestamp_ms
        relative = timestamp_ms - self.first
        # Track 1, cluster-relative timestamp 0, independently decodable keyframe.
        block = element('A3', b'\x81\x00\x00\x80' + jpeg.tobytes())
        self.stream.write(element('1F43B675', uint('E7', relative) + block))
        self.last = timestamp_ms

    def flush(self):
        self.stream.flush()

    def sync(self):
        self.stream.flush()
        os.fsync(self.stream.fileno())

    def release(self):
        if self.stream.closed:
            return
        # The final frame has no observed following interval: use 1ms, explicitly
        # reported by the replay tool, rather than inventing another sample gap.
        duration = 0 if self.first is None else self.last - self.first + 1
        self.stream.seek(self.duration_offset)
        self.stream.write(struct.pack('>d', float(duration)))
        self.sync()
        self.stream.close()
