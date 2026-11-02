#!/usr/bin/env python3
"""Run the six host isolation cases against functions from the actual overlay.

The surrounding C harness supplies SDK substitutes and fault injection only.
No SDK process, network request or aircraft control is started.
"""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
WIDGET = ROOT / 'psdk-overlay/samples/sample_c/module_sample/balloon_widget/psdk_balloon_widget.c'
FUNCTIONS = (
    'BalloonWidget_HelperBusy',
    'BalloonWidget_ApiTestMonotonicMs',
    'BalloonWidget_ReadApiTestMessage',
    'BalloonWidget_ApiTestTask',
    'BalloonWidget_StartApiTest',
    'BalloonWidget_VoiceWatchTask',
)


def function_span(source, name):
    pattern = re.compile(r'^static [^;{}]*?\b' + re.escape(name) + r'\([^;{}]*?\)\n\{', re.MULTILINE)
    matches = list(pattern.finditer(source))
    if len(matches) != 1:
        raise RuntimeError('Expected one definition of ' + name)
    start = matches[0].start()
    end = source.index('\n}\n', matches[0].end()) + len('\n}\n')
    return start, end


def main():
    production = WIDGET.read_text()
    harness = (ROOT / 'tests/test_widget_isolation.c').read_text()
    for name in FUNCTIONS:
        p_start, p_end = function_span(production, name)
        h_start, h_end = function_span(harness, name)
        harness = harness[:h_start] + production[p_start:p_end] + harness[h_end:]
    with tempfile.TemporaryDirectory(prefix='uav-widget-isolation-') as directory:
        folder = Path(directory)
        source, executable = folder / 'test.c', folder / 'test-widget'
        source.write_text(harness)
        command = shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=gnu11', '-pthread', '-Wno-unused-function',
            '-DTEST_PCM_PATH="' + str(folder / 'voice.pcm') + '"',
            str(source), '-o', str(executable),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True, timeout=15)
    print('PASS: six isolation cases used the packaged Widget function definitions')


if __name__ == '__main__':
    main()
