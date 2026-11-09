#!/usr/bin/env python3
"""Bounded panel checks; no flight commands, service changes or calibration writes.

The PSDK parent owns task mutual exclusion and cancellation generations. A
successful Kalman check authorizes selecting a mode only, not flight and not a
claim that the live estimator is active. Sampling requires an explicit ground
confirmation and the existing PSDK timing watcher.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import stat
import tempfile

from balloon_clock import clock_id, now_ms

SDK = Path('/home/uav/yanshou_uav/uavs_psdk/psdk_dji_3.9.2_cpp_json')
RUNTIME = Path('/home/uav/offline_balloon_strike_test')
MANIFEST = RUNTIME / 'flight_panel_manifest.json'
MESSAGE = Path('/tmp/balloon_panel_message.txt')
WIDGET_RELATIVE = Path('samples/sample_c/module_sample/balloon_widget')
CLOCK_RE = re.compile(r'mono-v1:[0-9a-fA-F]{8}(?:-[0-9a-fA-F]{4}){3}-[0-9a-fA-F]{12}\Z')
ID_RE = re.compile(r'[A-Za-z0-9_.-]{1,79}\Z')
HASH_RE = re.compile(r'[0-9a-f]{64}\Z')

# The v7 manifest remains the installed snapshot until a new build is staged.
WIDGET_ALIASES = {60: 50, 61: 51, 65: 55, 66: 56, 67: 57, 68: 58,
                  91: 81, 92: 82, 93: 83, 94: 84}
WIDGET_TYPES = {54: 'switch', 69: 'switch', 81: 'list', 82: 'list', 83: 'list',
                91: 'list', 92: 'list', 93: 'list', 98: 'switch'}
LEGACY_WIDGET_SCHEMA = (
    [['main', index, 'button'] for index in (50, 51, 55, 56, 57, 58)] +
    [['config', index, WIDGET_TYPES.get(index, 'button')]
     for index in (69, 60, 61, 52, 53, 54, 65, 66, 67, 68)]
)
WIDGET_SCHEMA = (
    [['main', index, WIDGET_TYPES.get(index, 'button')]
     for index in (50, 55, 51, 81, 82, 83, 84, 58)] +
    [['config', index, WIDGET_TYPES.get(index, 'button')]
     for index in (69, 60, 61, 52, 53, 54, 65, 66, 67, 91, 92, 93, 94, 98, 99, 68)]
)
WIDGET_NAMES = {
    'cn': {50: '语音输入任务', 51: 'API联网', 52: '相机/姿态采样', 53: '部署/标定核验',
           54: 'Kalman预测模式', 55: '执行API语音任务', 56: '新增API语音任务',
           57: '清除候选语音任务', 58: '停止测试', 69: '仅显示/维护',
           81: '任务类型', 82: '识别方案', 83: '预测方案', 84: '开始测试',
           98: '测试时录像', 99: '跳过当前目标，继续下一目标'},
    'en': {50: 'Voice task input', 51: 'Connect API', 52: 'Camera / pose sampling',
           53: 'Deployment / calibration check', 54: 'Kalman prediction mode',
           55: 'Execute API voice task', 56: 'New API voice task',
           57: 'Clear voice candidate', 58: 'Stop test', 69: 'Display only / maintenance',
           81: 'Task type', 82: 'Detector', 83: 'Prediction', 84: 'Start test',
           98: 'Record test', 99: 'Skip to next target'},
}

WIDGET_CHOICES = {
    'cn': {81: ['任意气球', '粉→黄'], 82: ['YOLO', '颜色像素检测'],
           83: ['原版像素预测', '运动补偿预测', '运动补偿＋延迟卡尔曼']},
    'en': {81: ['Any balloon', 'Pink → yellow'], 82: ['YOLO', 'Color pixel detection'],
           83: ['Original pixel prediction', 'Motion compensation', 'Motion compensation + delayed Kalman']},
}

REASONS = {
    'missing_calibration': '未配置时间标定',
    'invalid_calibration_file': '标定文件格式无效',
    'timing_unverified': '物理延迟尚未独立验证',
    'timing_invalid_clock': '标定时钟格式无效',
    'timing_clock_mismatch': '标定属于其他开机周期',
    'timing_invalid_calibration_id': '标定编号无效',
    'timing_invalid_delay': '物理延迟数值无效',
    'timing_uncertain': '标定不确定度超过25ms或无效',
    'timing_invalid_window': '标定有效期无效',
    'timing_outside_window': '标定未生效或已过期',
    'aligned': '本次开机时间标定有效',
}


class PanelCancelled(BaseException):
    pass


def _cancel(_signum, _frame):
    # Raising lets capture() remove only its own timing/record request files.
    raise PanelCancelled()


def bounded_bytes(path: Path, limit: int) -> bytes:
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
    with os.fdopen(fd, 'rb') as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > limit:
            raise ValueError('invalid file type or size')
        data = stream.read(limit + 1)
        if len(data) > limit:
            raise ValueError('file too large')
        return data


def _object(pairs):
    value = {}
    for key, entry in pairs:
        if key in value:
            raise ValueError('duplicate JSON key')
        value[key] = entry
    return value


def read_json(path: Path, limit: int = 1024 * 1024):
    def reject_constant(_value):
        raise ValueError('non-finite JSON number')
    return json.loads(bounded_bytes(path, limit).decode('utf-8-sig'),
                      object_pairs_hook=_object, parse_constant=reject_constant)


def publish(message: str, path: Path = MESSAGE) -> None:
    # Atomic, byte-bounded text: the C parent discards it after an epoch change.
    data = message.encode('utf-8')[:239].decode('utf-8', errors='ignore').encode('utf-8') + b'\n'
    fd, temporary = tempfile.mkstemp(prefix='.flight-panel-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def _number(value) -> bool:
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def validate_calibration(value, current_clock: str, current_ms: int) -> str:
    """Mirror LoadKfTiming JSON checks and TargetTiming_Validate bounds.

    verified is evidence supplied by an independently completed calibration;
    this function never creates it or converts a relative phase fit to delays.
    """
    if not isinstance(value, dict) or value.get('schema') != 'host_receipt_delays_v1':
        return 'invalid_calibration_file'
    if value.get('verified') is not True:
        return 'timing_unverified'
    boot = value.get('clock_id')
    if (not isinstance(boot, str) or not CLOCK_RE.fullmatch(boot) or
            not isinstance(current_clock, str) or not CLOCK_RE.fullmatch(current_clock)):
        return 'timing_invalid_clock'
    if boot != current_clock:
        return 'timing_clock_mismatch'
    ident = value.get('calibration_id')
    if not isinstance(ident, str) or not ID_RE.fullmatch(ident):
        return 'timing_invalid_calibration_id'
    for name in ('video_delay_ms', 'pose_delay_ms'):
        if not _number(value.get(name)) or not 0 <= value[name] <= 1500:
            return 'timing_invalid_delay'
    uncertainty = value.get('uncertainty_ms')
    if not _number(uncertainty) or not 0 <= uncertainty <= 25:
        return 'timing_uncertain'
    for name in ('valid_from_ms', 'valid_until_ms'):
        v = value.get(name)
        if not _number(v) or not 1 <= v <= 9007199254740991 or int(v) != v:
            return 'timing_invalid_window'
    if value['valid_until_ms'] <= value['valid_from_ms']:
        return 'timing_invalid_window'
    if not value['valid_from_ms'] <= current_ms <= value['valid_until_ms']:
        return 'timing_outside_window'
    return 'aligned'


def calibration_status(path=None, current_clock=None, current_ms=None):
    name = path if path is not None else os.environ.get('BALLOON_PREDICTION_TIMING_FILE', '')
    if not name:
        return {'ok': False, 'reason': 'missing_calibration'}
    try:
        value = read_json(Path(name), 4095)
        why = validate_calibration(value, clock_id() if current_clock is None else current_clock,
                                   now_ms() if current_ms is None else current_ms)
    except (OSError, ValueError, UnicodeError, RuntimeError):
        return {'ok': False, 'reason': 'invalid_calibration_file'}
    result = {'ok': why == 'aligned', 'reason': why}
    if why == 'aligned':
        result.update(calibration_id=value['calibration_id'], valid_until_ms=value['valid_until_ms'])
    return result


def digest(path: Path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise ValueError('not a regular file')
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            result.update(block)
    return result.hexdigest()


def running_binary_status(binary: Path, expected: str, proc: Path = Path('/proc')):
    checks = []
    for child in proc.iterdir():
        if not child.name.isdecimal():
            continue
        try:
            comm = (child / 'comm').read_text(encoding='ascii').strip()
        except (OSError, UnicodeError):
            continue
        if not comm.startswith(('dji_sdk', 'm300_voice')):
            continue
        try:
            actual = digest(child / 'exe')
            ok = actual == expected
            checks.append({'pid': int(child.name), 'ok': ok})
        except (OSError, ValueError):
            # A vanished process is a change during this snapshot, not proof.
            checks.append({'pid': int(child.name), 'ok': False})
    return {'status': ('not_running' if not checks else
                       'matched' if len(checks) == 1 and checks[0]['ok'] else 'mismatch'),
            'checks': checks}


def _file_row(row, sdk_root: Path, runtime_root: Path, *, binary=False):
    if not isinstance(row, dict):
        raise ValueError('invalid manifest row')
    path = Path(row.get('device_path', ''))
    sha = row.get('sha256')
    if not path.is_absolute() or '..' in path.parts or not isinstance(sha, str) or not HASH_RE.fullmatch(sha):
        raise ValueError('invalid manifest path or hash')
    if sdk_root not in path.parents and (binary or runtime_root not in path.parents):
        raise ValueError('manifest path outside deployment roots')
    if not binary and path.suffix not in ('.c', '.h', '.cpp', '.hpp', '.py', '.json'):
        raise ValueError('unsupported source type')
    if path.is_symlink():
        raise ValueError('source symlink rejected')
    try:
        matches = digest(path) == sha
    except (OSError, ValueError):
        matches = False
    return path, matches


def deployment_status(manifest_path: Path = MANIFEST, *, sdk_root=SDK, runtime_root=RUNTIME,
                      proc=Path('/proc')):
    manifest = read_json(manifest_path)
    if not isinstance(manifest, dict) or not ID_RE.fullmatch(str(manifest.get('release_id', ''))):
        raise ValueError('invalid release manifest')
    rows = manifest.get('files')
    schema = manifest.get('widget_schema')
    if not isinstance(rows, list) or not 3 <= len(rows) <= 128 or not isinstance(schema, list):
        raise ValueError('incomplete release manifest')
    if (schema not in (WIDGET_SCHEMA, LEGACY_WIDGET_SCHEMA) or
            any(type(row[1]) is not int for row in schema)):
        raise ValueError('invalid widget schema')
    checked = {}
    for row in rows:
        path, good = _file_row(row, sdk_root, runtime_root)
        if path in checked:
            raise ValueError('duplicate manifest source')
        checked[path] = good
    widget = sdk_root / WIDGET_RELATIVE
    required = [widget / 'psdk_balloon_widget.c'] + [widget / f'widget_file/{lang}_big_screen/widget_config.json'
                                                    for lang in ('cn', 'en')]
    if any(path not in checked for path in required):
        raise ValueError('manifest omits widget sources')
    issues = []
    if not all(checked.values()):
        issues.append('source_hash_mismatch')
    for language, path in zip(('cn', 'en'), required[1:]):
        try:
            config = read_json(path)
            actual = []
            for page in ('main', 'config'):
                for row in config[page + '_interface']['widget_list']:
                    actual.append([page, row['widget_index'], row['widget_type']])
                    action = WIDGET_ALIASES.get(row['widget_index'], row['widget_index'])
                    if row.get('widget_name') != WIDGET_NAMES[language].get(action):
                        issues.append('widget_name_mismatch')
                    if row['widget_type'] == 'list':
                        items = row['list_item']
                        if [item['item_name'] for item in items] != WIDGET_CHOICES[language].get(action):
                            raise ValueError('invalid choices or order')
                    else:
                        items = [row]
                    for item in items:
                        if page == 'config' and 'icon_file_set' not in item:
                            continue
                        icons = item['icon_file_set']
                        if not isinstance(icons, dict) or not icons:
                            raise ValueError('empty icon set')
                        for icon in icons.values():
                            if (not isinstance(icon, str) or not icon or Path(icon).name != icon or
                                    not (path.parent / icon).is_file()):
                                raise ValueError('invalid or missing icon')
            if actual != schema:
                issues.append('widget_schema_mismatch')
        except (OSError, ValueError, KeyError, TypeError):
            issues.append('widget_layout_or_icon_invalid')
    binary, good = _file_row(manifest.get('binary'), sdk_root, runtime_root, binary=True)
    if not good:
        issues.append('installed_binary_mismatch')
    running = running_binary_status(binary, manifest['binary']['sha256'], proc)
    if running['status'] == 'mismatch':
        issues.append('running_binary_mismatch')
    return {'ok': not issues, 'release_id': manifest['release_id'], 'source_count': len(rows),
            'running': running, 'issues': sorted(set(issues))}


def sample(seconds: int, stationary_aircraft: bool, capture_fn=None):
    if not stationary_aircraft:
        raise ValueError('stationary ground confirmation required')
    if type(seconds) is not int or not 15 <= seconds <= 60:
        raise ValueError('sample duration must be 15..60 seconds')
    if capture_fn is None:
        from capture_camera_timing import capture
        capture_fn = capture
    publish('相机/姿态采样：请保持飞机地面静止；采集中，停止测试可取消')
    capture_fn(seconds)
    return {'ok': True, 'seconds': seconds, 'calibration_created': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument('--sample', action='store_true')
    actions.add_argument('--audit', action='store_true')
    actions.add_argument('--check-kalman', action='store_true')
    parser.add_argument('--seconds', type=int, default=30)
    parser.add_argument('--stationary-aircraft', action='store_true')
    parser.add_argument('--manifest', type=Path, default=MANIFEST)
    args = parser.parse_args(argv)
    if args.sample and (not args.stationary_aircraft or not 15 <= args.seconds <= 60):
        publish('采样未开始：需确认飞机地面静止，采样时长15–60秒')
        return 2
    signal.signal(signal.SIGTERM, _cancel)
    signal.signal(signal.SIGINT, _cancel)
    try:
        if args.sample:
            result = sample(args.seconds, args.stationary_aircraft)
            message = '采样完成：录像与姿态已保存；尚未生成或验证时间标定'
        elif args.check_kalman:
            result = calibration_status()
            message = ('Kalman可选择：标定有效；实际启用仍取决于实时姿态和观测' if result['ok'] else
                       'Kalman未启用：' + REASONS[result['reason']])
        else:
            calibration = calibration_status()
            try:
                deployment = deployment_status(args.manifest)
            except (OSError, ValueError, KeyError, TypeError, UnicodeError):
                deployment = {'ok': False, 'issues': ['manifest_or_files_unavailable'],
                              'running': {'status': 'unverified'}}
            result = {'ok': deployment['ok'] and calibration['ok'], 'deployment': deployment,
                      'calibration': calibration, 'flight_authorized': False}
            running_text = {'not_running': '未运行', 'matched': '运行版本一致',
                            'mismatch': '运行版本异常', 'unverified': '未核验'}[deployment['running']['status']]
            message = ('部署：' + ('一致' if deployment['ok'] else '核验失败') + '｜进程：' + running_text +
                       '｜标定：' + REASONS[calibration['reason']])
        publish(message)
        print(json.dumps(result, ensure_ascii=False, separators=(',', ':')))
        return 0 if result['ok'] else 1
    except PanelCancelled:
        publish('采样/核验已取消；本轮结果无效')
        return 130
    except (OSError, ValueError, RuntimeError):
        # Avoid echoing arbitrary recorder/API text or environment contents.
        publish('采样/核验失败：检查采样日志与设备状态；未修改标定或飞行设置')
        return 3


if __name__ == '__main__':
    raise SystemExit(main())
