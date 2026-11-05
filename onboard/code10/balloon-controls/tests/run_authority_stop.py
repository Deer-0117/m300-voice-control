#!/usr/bin/env python3
"""Compile real control adapter against SDK stubs, never DJI libraries."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--sdk-root', required=True, type=Path, help='SDK providing public headers and cJSON source')
args = parser.parse_args()
base = Path(__file__).resolve().parents[1]
module = args.sdk_root / 'samples/sample_c/module_sample'
core = base / 'psdk-overlay/samples/sample_c/module_sample/target_chase'
with tempfile.TemporaryDirectory(prefix='uav-authority-stubs-') as tmp:
    binary = Path(tmp) / 'authority_stop'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu99', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-Wno-unused-function', '-DCJSON_HIDE_SYMBOLS', '-pthread',
                    '-I' + str(args.sdk_root / 'psdk_lib/include'), '-I' + str(module),
                    str(base / 'tests/authority_stop_test.c'), str(core / 'target_chase.c'),
                    str(module / 'utils/cJSON.c'), '-lm', '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True, timeout=30, cwd=tmp)
