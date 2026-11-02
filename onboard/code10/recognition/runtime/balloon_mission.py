"""Widget/YOLO startup handshake. Missing or changed plans never mean 'any'."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import threading
import uuid

from balloon_clock import clock_id, now_ms
from uav_capabilities import validate_actions

ROOT = Path(__file__).resolve().parent
TMP = Path('/tmp')
TOKEN = TMP / 'balloon_mission.token'
REQUEST = TMP / 'balloon_mission.json'
ACK = TMP / 'balloon_mission_ack.json'
READY = TMP / 'balloon_mission.ready'
COMMIT = TMP / 'balloon_mission.commit'
HEALTH = TMP / 'balloon_mission.health'
MESSAGE = TMP / 'balloon_mission_message.txt'
STATUS = TMP / 'balloon_mission_status.txt'
RECORD_REQUEST = TMP / 'balloon_prediction_record.request'
RECORD_ACK = TMP / 'balloon_prediction_record_ack.json'
STOP = TMP / 'balloon_nl_stop.request'
PLAN = ROOT / 'nl_task_plan.json'
STATE = ROOT / 'nl_task_state.json'
EVENTS = ROOT / 'widget_events.jsonl'
REPLACE_RETRIES = 6 if os.name == 'nt' else 1
_FILE_LOCK = threading.RLock()


def atomic_text(path: Path, value: str) -> None:
    # Local recorder/worker threads must not hold a Windows read handle while
    # another thread renames the same receipt. Other processes use bounded retry.
    with _FILE_LOCK:
        _atomic_text(path, value)


def _atomic_text(path: Path, value: str) -> None:
    temporary = path.with_name(path.name + f'.{os.getpid()}.{uuid.uuid4().hex}.tmp')
    try:
        temporary.write_text(value, encoding='utf-8')
        for attempt in range(REPLACE_RETRIES):
            try:
                temporary.replace(path)
                return
            except PermissionError:
                # Windows readers can briefly deny rename; Linux keeps its
                # existing immediate failure behavior. Never remove the old ACK.
                if attempt + 1 == REPLACE_RETRIES:
                    raise
                time.sleep(.01)
    finally:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass


def atomic_json(path: Path, value: dict) -> None:
    atomic_text(path, json.dumps(value, ensure_ascii=False))


def read_json(path: Path) -> dict:
    with _FILE_LOCK:
        value = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError('文件内容必须是对象')
    return value


def digest(plan: dict) -> str:
    return hashlib.sha256(json.dumps(plan, sort_keys=True, separators=(',', ':'),
                                     ensure_ascii=False).encode()).hexdigest()


def token_matches(token: str) -> bool:
    try:
        return TOKEN.read_text(encoding='ascii').strip() == token
    except OSError:
        return False


def event(name: str, **fields) -> None:
    if 'token' not in fields:
        try:
            fields['token'] = TOKEN.read_text(encoding='ascii').strip()
        except OSError:
            pass
    fields['mission_token'] = fields.get('token', '')
    with EVENTS.open('a', encoding='utf-8') as stream:
        stream.write(json.dumps({'event': name, 'ts_ms': now_ms(), 'clock_id': clock_id(),
                                 **fields}, ensure_ascii=False) + '\n')


class MissionGate:
    """Runs inside the actual YOLO worker, including while it is idle."""
    def __init__(self) -> None:
        self.last_error = None
        self.last_token = None
        self.last_publish = -1000
        self.last_transition = None

    def poll(self, sequence) -> bool:
        try:
            request = read_json(REQUEST)
            token = request['token']
            if not token_matches(token) or request['clock_id'] != clock_id():
                raise ValueError('启动请求已取消或属于其他开机周期')
            plan = read_json(PLAN)
            actions = validate_actions(plan['actions'])
            if (not plan.get('plan_id') or plan['plan_id'] != request['plan_id'] or
                    digest(plan) != request['digest']):
                raise ValueError('计划丢失、被替换或内容变化')
            if request['mode'] == 'pink' and actions != strike_actions(('pink', 'yellow')):
                raise ValueError('粉黄顺序不匹配')
            sequence._load_plan()
            if (sequence.plan_id != plan['plan_id'] or sequence.actions != actions or
                    sequence.status != 'active'):
                raise ValueError('执行端计划未就绪或本轮已结束')
            if request['record'] and COMMIT.exists():
                recording = read_json(RECORD_ACK)
                age = now_ms() - recording['ts_ms']
                if (recording.get('token') != token or recording.get('clock_id') != clock_id() or
                        recording.get('status') != 'recording' or not 0 <= age <= 3000):
                    raise ValueError('录像中断或回执过期')
            current = now_ms()
            if token != self.last_token or current - self.last_publish >= 100:
                receipt = dict(token=token, plan_id=plan['plan_id'], digest=request['digest'],
                               clock_id=clock_id(), ts_ms=current, status='loaded',
                               current_index=sequence.current_index, worker_pid=os.getpid())
                atomic_json(ACK, receipt)
                atomic_text(HEALTH, f'{token}\t{clock_id()}\t{current}\n')
                color = actions[sequence.current_index].get('parameters', {}).get('color', 'stop')
                colors = {'pink': '粉色', 'yellow': '黄色', 'any': '任意气球'}
                title = {'pink': '粉→黄', 'any': '任意气球', 'voice': '语音任务', 'next': '继续剩余目标'}[request['mode']]
                recording = '关闭'
                if request['record']:
                    try:
                        rec = read_json(RECORD_ACK)
                        recording = '录制中' if rec.get('token') == token and rec.get('status') == 'recording' else '等待/异常'
                    except (OSError, ValueError):
                        recording = '等待/异常'
                atomic_text(STATUS, f'任务：{title}｜当前：{colors.get(color, color)}（{sequence.current_index + 1}/{len(actions)}）｜录像：{recording}')
                self.last_publish = current
            transition = (token, sequence.current_index, sequence.status, COMMIT.exists())
            if transition != self.last_transition:
                event('mission_transition', token=token, plan_id=plan['plan_id'],
                      current_index=sequence.current_index, status=sequence.status,
                      committed=transition[3], color=actions[sequence.current_index].get('parameters', {}).get('color', 'stop'))
                self.last_transition = transition
            self.last_token = token
            self.last_error = None
            return True
        except (OSError, ValueError, KeyError, IndexError, TypeError) as exc:
            HEALTH.unlink(missing_ok=True)
            ACK.unlink(missing_ok=True)
            # Even after a previously valid plan is deleted, never call the
            # legacy standalone selector. The controller also checks HEALTH.
            if self.last_token and token_matches(self.last_token):
                if not STOP.exists():
                    atomic_text(STOP, 'reason=mission_plan_invalid\n')
                if self.last_error != str(exc):
                    event('mission_gate_closed', reason=str(exc), token=self.last_token)
                    self.last_error = str(exc)
            return False


def strike_actions(colors) -> list[dict]:
    return [dict(capability='strike_balloon', parameters=dict(color=color)) for color in colors]


def prepare(mode: str, token: str, record: bool) -> None:
    if not token or len(token) > 95 or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_" for c in token):
        raise ValueError("invalid mission token")
    if not token_matches(token):
        raise ValueError('启动已取消')
    if mode == 'voice':
        # Confirmation can perform a remote request. The C epoch check and
        # token check below reject a result that arrives after Stop.
        from uav_voice_command import confirm
        previous_id = read_json(PLAN).get('plan_id') if PLAN.exists() else None
        confirm()
        plan = read_json(PLAN)
        if plan.get('plan_id') == previous_id:
            raise ValueError('语音未生成新的本机计划')
        actions = validate_actions(plan['actions'])
    elif mode == 'next':
        old = read_json(PLAN)
        state = read_json(STATE)
        if state.get('plan_id') != old.get('plan_id') or state.get('status') != 'awaiting_confirmation':
            raise ValueError('当前没有等待处理的目标')
        index = state['current_index']
        actions = validate_actions(old['actions'])
        if not isinstance(index, int) or index < 0 or index + 1 >= len(actions):
            raise ValueError('已经是最后一个目标，没有下一目标')
        event('operator_skipped_unconfirmed_target', plan_id=old['plan_id'], index=index,
              physical_outcome='unknown')
        actions = actions[index + 1:]
    else:
        actions = strike_actions(('pink', 'yellow') if mode == 'pink' else ('any',))
    if not token_matches(token):
        raise ValueError('启动已取消，迟到结果已拒绝')
    # A new immutable plan for every start, including explicit any-balloon.
    plan = dict(version=2, plan_id=uuid.uuid4().hex, actions=actions, source='widget_' + mode)
    atomic_json(PLAN, plan)
    atomic_json(STATE, dict(plan_id=plan['plan_id'], status='queued', current_index=0))
    STOP.unlink(missing_ok=True)
    request = dict(token=token, mode=mode, record=record, plan_id=plan['plan_id'],
                   digest=digest(plan), clock_id=clock_id())
    archive = EVENTS.parent / 'mission_records' / token
    archive.mkdir(parents=True, exist_ok=True)
    atomic_json(archive / 'request.json', request)
    atomic_json(archive / 'plan.json', plan)
    atomic_json(REQUEST, request)
    event('mission_requested', **request)
    wait_ack(request)
    atomic_text(READY, token + '\n')
    event('mission_loaded_before_start', **request)


def wait_ack(request: dict, timeout: float = 12.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not token_matches(request['token']):
            raise ValueError('启动已取消')
        try:
            ack = read_json(ACK)
            age = now_ms() - ack['ts_ms']
            if (all(ack.get(k) == request[k] for k in ('token', 'plan_id', 'digest', 'clock_id'))
                    and ack.get('status') == 'loaded' and 0 <= age <= 1500):
                return
        except (OSError, ValueError, KeyError, TypeError):
            pass
        time.sleep(.05)
    raise ValueError('执行端未确认加载计划，请检查YOLO服务后重试')


def wait_record(token: str, timeout: float = 12.0) -> None:
    request = read_json(REQUEST)
    if request['token'] != token:
        raise ValueError('任务已变更')
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not token_matches(token):
            raise ValueError('启动已取消')
        try:
            ack = read_json(RECORD_ACK)
            if ack.get('token') == token and ack.get('clock_id') == clock_id():
                if ack.get('status') == 'failed':
                    raise RuntimeError('录像启动失败：' + str(ack.get('error', '未知原因')))
                if ack.get('status') == 'recording' and 0 <= now_ms() - ack['ts_ms'] <= 2000:
                    wait_ack(request, timeout=2.0)
                    return
        except (OSError, ValueError, KeyError, TypeError):
            pass
        time.sleep(.05)
    raise ValueError('录像未确认开始，任务未放行；检查相机/存储，或关闭录像后重试')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('any', 'pink', 'voice', 'next'))
    parser.add_argument('--token', required=True)
    parser.add_argument('--record', type=int, choices=(0, 1), default=1)
    parser.add_argument('--wait-record', action='store_true')
    args = parser.parse_args()
    try:
        if args.wait_record:
            wait_record(args.token)
        else:
            if not args.mode:
                raise ValueError('缺少任务类型')
            prepare(args.mode, args.token, bool(args.record))
        return 0
    except Exception as exc:
        atomic_text(MESSAGE, str(exc))
        event('mission_start_failed', token=args.token, reason=str(exc))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
