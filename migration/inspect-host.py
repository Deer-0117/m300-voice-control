#!/usr/bin/env python3
"""Device inventory only. Do not print account JSON, service Environment or keys."""
import ctypes.util
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys

SDK=Path('/home/uav/yanshou_uav/uavs_psdk/psdk_dji_3.9.2_cpp_json')
HOST=SDK/'samples/sample_c++/platform/linux/manifold2'
CALLS=('DjiCore_Init','DjiCore_ApplicationStart','DjiFlightController_Init',
       'DjiWidget_Init','DjiFcSubscription_Init','DjiLiveview_Init',
       'DjiTest_WidgetSpeakerStartService','DjiTest_BalloonWidgetStartService',
       'M300Runtime_StartFromEnvironment')

def command(argv):
    try:
        result=subprocess.run(argv,capture_output=True,text=True,timeout=12)
        return result.stdout.strip() if result.returncode==0 else 'unavailable'
    except (OSError,subprocess.TimeoutExpired):return 'unavailable'

def main():
    if sys.platform!='linux' or platform.machine()!='aarch64':
        raise SystemExit('Run this on the Jetson communication box, not Windows.')
    result={'python':sys.version.split()[0],'machine':platform.machine(),
            'os_release':Path('/etc/os-release').read_text() if Path('/etc/os-release').exists() else None,
            'l4t':Path('/etc/nv_tegra_release').read_text() if Path('/etc/nv_tegra_release').exists() else None,
            'sdk_root':str(SDK),'host_root':str(HOST),'sdk_exists':SDK.is_dir(),
            'libopus':ctypes.util.find_library('opus'),
            'service':command(['systemctl','show','psdk-balloon.service','-p','User','-p','Group','-p','SupplementaryGroups','-p','PrivateTmp','-p','MainPID','-p','WorkingDirectory','-p','ProtectHome']),
            'cuda':command(['/usr/local/cuda/bin/nvcc','--version']),
            'cmake_files':[],'cmake_targets':[],'source_calls':[],'sdk_version_defines':[],'libraries':[],
            'input_paths':{},'whisper_paths':{}}
    service_start=command(['systemctl','show','psdk-balloon.service','-p','ExecStart','--value'])
    executable=re.search(r'\bpath=([^\s;]+)',service_start)
    result['active_service_executable']=executable.group(1) if executable else 'unavailable'
    result['compiler']=command(['cc','--version'])
    version=SDK/'psdk_lib/include/dji_version.h'
    if version.is_file():
        for line in version.read_text(errors='replace').splitlines():
            match=re.match(r'\s*#define\s+(DJI_VERSION_\w+)\s+(\d+)',line)
            if match:result['sdk_version_defines'].append(list(match.groups()))
    for library in (SDK/'psdk_lib/lib').rglob('libpayloadsdk.a'):
        result['libraries'].append(str(library))
    for directory,dirs,files in os.walk(HOST):
        dirs[:]=[d for d in dirs if not d.startswith(('build','.git'))]
        for name in sorted(files):
            file=Path(directory)/name
            if name=='CMakeLists.txt':
                result['cmake_files'].append(str(file))
                for kind,target in re.findall(r'\b(project|add_executable|add_library)\s*\(\s*([^\s\)]+)',file.read_text(errors='replace'),re.I):
                    result['cmake_targets'].append({'file':str(file),'kind':kind,'name':target})
            if file.suffix not in ('.c','.cpp','.cc','.h','.hpp') or file.stat().st_size>1024*1024:continue
            for number,line in enumerate(file.read_text(errors='replace').splitlines(),1):
                for function in CALLS:
                    if re.search(r'\b'+re.escape(function)+r'\s*\(',line):
                        # Output names/locations only, never raw source or arguments.
                        result['source_calls'].append({'file':str(file),'line':number,'function':function})
    for value in ('/tmp/balloon_liveview.h264','/dev/shm/balloon_liveview_latest.tsv',
                  '/dev/shm/balloon_liveview_frames'):
        file=Path(value)
        result['input_paths'][value]={'exists':file.exists(),'symlink':file.is_symlink()}
    for value in ('/opt/whisper.cpp/build/bin/whisper-cli','/opt/whisper.cpp/models/ggml-base.bin',
                  '/opt/whisper.cpp/models/ggml-small.bin'):
        file=Path(value)
        result['whisper_paths'][value]={'exists':file.is_file(),'executable':os.access(value,os.X_OK)}
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
