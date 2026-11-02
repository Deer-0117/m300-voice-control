"""Bounded validation of original JPEG bytes; validation never re-encodes."""

import hashlib
import io
import warnings

from PIL import Image

MAX_BYTES = 8 * 1024 * 1024
MAX_WIDTH = 3840
MAX_HEIGHT = 2160
MAX_PIXELS = 8_294_400
DECODE_BUDGET = 64 * 1024 * 1024


class InvalidImage(ValueError):
    """The supplied bytes are not an allowed, complete JPEG."""


def _dimensions(data):
    """Read SOF before entering Pillow, whose header limits are more permissive."""
    if data[:2] != b'\xff\xd8' or data[-2:] != b'\xff\xd9':
        raise InvalidImage('missing JPEG start/end marker')
    pos = 2
    while pos < len(data):
        if data[pos] != 255:
            raise InvalidImage('invalid JPEG marker')
        while pos < len(data) and data[pos] == 255:
            pos += 1
        if pos >= len(data):
            break
        marker = data[pos]
        pos += 1
        if marker in (0, 0xD8, 0xD9, 0xDA):
            break
        if marker == 1 or 0xD0 <= marker <= 0xD7:
            continue
        if pos + 2 > len(data):
            break
        length = int.from_bytes(data[pos:pos + 2], 'big')
        if length < 2 or pos + length > len(data):
            raise InvalidImage('invalid JPEG segment length')
        if marker in (0xC0, 0xC1, 0xC2):
            if length < 8 or data[pos + 2] != 8:
                raise InvalidImage('unsupported JPEG precision')
            height = int.from_bytes(data[pos + 3:pos + 5], 'big')
            width = int.from_bytes(data[pos + 5:pos + 7], 'big')
            channels = data[pos + 7]
            if channels not in (1, 3) or length != 8 + 3 * channels:
                raise InvalidImage('unsupported JPEG channels')
            return width, height
        pos += length
    raise InvalidImage('missing supported JPEG SOF header')


def validate_jpeg(data, width=None, height=None):
    """Validate one grayscale/RGB JPEG within the fixed 4K and 64 MiB budgets.

    The caller must serialize decoding. The budget is an admission estimate,
    not an operating-system memory or time limit on the third-party decoder.
    """
    if not isinstance(data, bytes) or not 0 < len(data) <= MAX_BYTES:
        raise InvalidImage('JPEG bytes must be immutable and between 1 byte and 8 MiB')
    actual_width, actual_height = _dimensions(data)
    if (not 0 < actual_width <= MAX_WIDTH or not 0 < actual_height <= MAX_HEIGHT
            or actual_width * actual_height > MAX_PIXELS
            or 2 * actual_width * actual_height * 3 + MAX_BYTES > DECODE_BUDGET):
        raise InvalidImage('JPEG dimensions exceed decode budget')
    for declared, actual in ((width, actual_width), (height, actual_height)):
        if declared is not None and (type(declared) is not int or declared != actual):
            raise InvalidImage('declared dimensions do not match JPEG')
    try:
        with warnings.catch_warnings():
            warnings.simplefilter('error', Image.DecompressionBombWarning)
            with Image.open(io.BytesIO(data)) as picture:
                if (picture.format != 'JPEG' or picture.mode not in ('L', 'RGB')
                        or getattr(picture, 'n_frames', 1) != 1
                        or picture.size != (actual_width, actual_height)):
                    raise InvalidImage('unsupported image format or mode')
                picture.verify()
            with Image.open(io.BytesIO(data)) as picture:
                picture.load()
    except (OSError, ValueError, SyntaxError, Image.DecompressionBombError,
            Image.DecompressionBombWarning) as exc:
        raise InvalidImage('JPEG failed complete decode') from exc
    return {'width': actual_width, 'height': actual_height,
            'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}
