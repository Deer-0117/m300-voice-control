#!/usr/bin/env python3
"""Create a private source snapshot and build a separate ground-only PSDK host.

Never edits the active SDK, stops services, executes the built host or copies
credentials off the device. New private snapshots contain existing SDK account
headers/configs: do NOT upload or publish the generated staging tree.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile

SDK_DEFAULT=Path('/home/uav/yanshou_uav/uavs_psdk/psdk_dji_3.9.2_cpp_json')
ROOT=Path(__file__).resolve().parents[1]
HOST_REL=Path('samples/sample_c++/platform/linux/manifold2')

def patch_application(text):
    serial='DjiCore_SetSerialNumber('
    marker='#ifdef CONFIG_MODULE_SAMPLE_CAMERA_EMU_ON'
    balloon='returnCode = BalloonWidget_StartService('
    core='returnCode = DjiCore_ApplicationStart();'
    probe='returnCode = BalloonChaseProbe_RunNoFlightProbe();'
    done='USER_LOG_INFO("Application start.");'
    for anchor in (serial,balloon,core,probe,done):
        if text.count(anchor)!=1:raise ValueError('Unexpected source layout at '+anchor.split('(')[0])
    serial_index=text.index(serial)
    begin=text.find(marker,serial_index)
    end=text.index(core)
    if begin<0 or not serial_index<begin<text.index(balloon)<end:
        raise ValueError('Legacy startup boundaries do not match inspected source')
    block=text[begin:end]
    for expected in ('DjiTest_WidgetStartService(', 'DjiTest_WidgetSpeakerStartService(',balloon):
        if block.count(expected)!=1:raise ValueError('Unexpected old startup block: '+expected)
    wrapped=('#if 0 /* MIGRATION: old business modules disabled only in this private copy */\n'
             +block+'\n#endif /* MIGRATION old business modules */\n    ')
    result=text[:begin]+wrapped+text[end:]
    begin=result.index(probe)
    end=result.index(done,begin)
    if end-begin>2000:raise ValueError('Unexpected old probe block size')
    block=result[begin:end]
    result=(result[:begin]+'\n#if 0 /* MIGRATION: do not start the previous chase controller */\n'
            +block+'\n#endif /* MIGRATION old probe */\n    '+result[end:])
    if result.count('DjiCore_ApplicationStart();')!=1 or result.count('DjiCore_SetSerialNumber(')!=1:
        raise ValueError('Core setup changed unexpectedly')
    return result

def copy_application(source,destination):
    destination.mkdir()
    for file in source.rglob('*'):
        if file.is_symlink():raise ValueError('Unexpected symlink in application tree; inspect before copying')
        relative=file.relative_to(source)
        if any(part.startswith(('.git','build','__pycache__')) for part in relative.parts):continue
        if not file.is_file() or file.suffix not in ('.c','.cc','.cpp','.h','.hpp','.json'):continue
        target=destination/relative
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(file,target)

def create_stage(sdk,package,workspace):
    sdk=sdk.resolve(strict=True);package=package.resolve(strict=True)
    original_host=sdk/HOST_REL
    application=original_host/'application/application.cpp'
    header=original_host/'application/application.hpp'
    for path in (application,header,original_host/'CMakeLists.txt',original_host/'hal',
                 sdk/'psdk_lib',sdk/'samples/sample_c',sdk/'samples/sample_c++/module_sample',
                 sdk/'samples/sample_c++/platform/linux/common'):
        if not path.exists():raise ValueError('Required SDK path missing: '+str(path))
    original=application.read_text(encoding='utf-8')
    patched=patch_application(original)
    if 'Application(int argc, char **argv)' not in header.read_text(encoding='utf-8'):
        raise ValueError('Application constructor changed; inspect before proceeding')
    workspace.mkdir(parents=True,exist_ok=True)
    if workspace.is_symlink() or (os.name=='posix' and workspace.stat().st_uid!=os.getuid()):
        raise ValueError('Workspace must be a normal directory owned by the current build user')
    if os.name=='posix':os.chmod(workspace,0o700)
    stage=Path(tempfile.mkdtemp(prefix='stage-',dir=str(workspace)))
    host=stage/HOST_REL;host.mkdir(parents=True)
    links={stage/'psdk_lib':sdk/'psdk_lib',stage/'samples/sample_c':sdk/'samples/sample_c',
           stage/'samples/sample_c++/module_sample':sdk/'samples/sample_c++/module_sample',
           stage/'samples/sample_c++/platform/linux/common':sdk/'samples/sample_c++/platform/linux/common',
           host/'hal':original_host/'hal'}
    for link,target in links.items():link.symlink_to(target,target_is_directory=True)
    copy_application(original_host/'application',host/'application')
    (host/'application/application.cpp').write_text(patched,encoding='utf-8')
    shutil.copy2(original_host/'CMakeLists.txt',host/'original-CMakeLists.txt')
    shutil.copy2(package/'migration/host/CMakeLists.txt',host/'CMakeLists.txt')
    manifest={'source_sdk':str(sdk),'source_host':str(original_host),'stage':str(stage),
              'stage_host':str(host),'build':str(stage/'build'),
              'original_application_sha256':hashlib.sha256(original.encode()).hexdigest(),
              'ground_only':True,'original_sources_modified':False,
              'private_account_files_copied_on_device':True,'host_executed':False}
    (stage/'stage-info.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    return manifest

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-root',type=Path,default=SDK_DEFAULT)
    parser.add_argument('--workspace',type=Path,default=Path('/home/uav/m300-host-work'))
    args=parser.parse_args()
    if sys.platform!='linux' or platform.machine()!='aarch64' or os.getuid()==0:
        raise SystemExit('Run as the normal uav build user on Jetson ARM64, without sudo.')
    for command in ('cmake','gcc','g++'):
        if not shutil.which(command):raise SystemExit('Missing build tool: '+command)
    try:
        manifest=create_stage(args.sdk_root,ROOT,args.workspace)
        print('PRIVATE_STAGE='+manifest['stage'],flush=True)
        build=Path(manifest['build'])
        subprocess.run(['cmake','-S',manifest['stage_host'],'-B',str(build),
                        '-DCMAKE_BUILD_TYPE=Release','-DM300_GROUND_ONLY=ON',
                        '-DDJI_PSDK_ROOT='+str(args.sdk_root.resolve()),
                        '-DM300_PACKAGE_ROOT='+str(ROOT),
                        '-DM300_HOST_SUPPORT='+str(ROOT/'migration/host')],check=True)
        subprocess.run(['cmake','--build',str(build),'--target','m300_voice_psdk_host','--parallel','2'],check=True)
        binary=build/'bin/m300_voice_psdk_host'
        if not binary.is_file():raise RuntimeError('Expected host executable missing')
        manifest['binary']=str(binary)
        manifest['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
        (ROOT/'migration/last-host-build.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
        print('HOST_BUILD_SUCCEEDED='+str(binary),flush=True)
        print('Build only. No binary executed; no service/source/flight settings changed.',flush=True)
    except (OSError,ValueError,RuntimeError,subprocess.CalledProcessError) as exc:
        print('Build stopped: '+str(exc),file=sys.stderr)
        return 1
    return 0

if __name__=='__main__':raise SystemExit(main())
