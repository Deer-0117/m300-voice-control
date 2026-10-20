from __future__ import annotations
import hashlib, json, secrets
COLORS = {'red': '红色', 'green': '绿色', 'purple': '紫色'}

class Mission:
    def __init__(self):
        self.revision = 0
        self.targets = []
        self.completed = []
        self.status = 'idle'
        self.pending = None
        self.approved_hash = ''
        self.approved_nonce = ''
        self.history = []

    def interrupt(self):
        self.revision += 1
        self.status = 'analyzing_task'
        self.pending = None
        self.approved_hash = self.approved_nonce = ''
        return self.revision

    def propose(self, response, revision):
        if revision != self.revision: return False
        if not isinstance(response, dict) or set(response) != {'intent', 'targets'}: raise ValueError('invalid task response')
        intent, targets = response['intent'], response['targets']
        if intent == 'pause':
            if targets != self.targets: raise ValueError('pause must preserve current plan')
            self.pause(); return True
        if intent not in ('execute', 'revise'): raise ValueError('invalid intent')
        if not isinstance(targets, list) or not 1 <= len(targets) <= 3: raise ValueError('invalid target count')
        if any(type(c) is not str or c not in COLORS for c in targets) or len(set(targets)) != len(targets): raise ValueError('unsupported or duplicate target')
        if targets[:len(self.completed)] != self.completed: raise ValueError('completed prefix changed')
        digest = hashlib.sha256(json.dumps(targets, separators=(',', ':')).encode()).hexdigest()
        self.pending = {'revision': revision, 'targets': targets[:], 'plan_hash': digest, 'nonce': secrets.token_hex(16)}
        self.status = 'awaiting_confirmation'
        return True

    def confirm(self, event):
        p = self.pending
        if self.status != 'awaiting_confirmation' or not p: return False
        if any(event.get(k) != p[k] for k in ('revision', 'plan_hash', 'nonce')): return False
        self.targets = p['targets'][:]
        self.approved_hash, self.approved_nonce = p['plan_hash'], p['nonce']
        self.pending = None
        self.status = 'active' if self.target is not None else 'done'
        return True

    def reject(self):
        self.pause()

    def pause(self):
        self.revision += 1
        self.status = 'paused'
        self.pending = None
        self.approved_hash = self.approved_nonce = ''

    @property
    def target(self):
        return self.targets[len(self.completed)] if len(self.completed) < len(self.targets) else None

    def mark_complete(self, event):
        if self.status != 'active' or event.get('revision') != self.revision or event.get('target_index') != len(self.completed) or self.target is None: return False
        self.completed.append(self.target)
        if self.target is None: self.status = 'done'
        return True

    def summary(self):
        if self.pending:
            labels = '、'.join(COLORS[c] for c in self.pending['targets'])
            done = '、'.join(COLORS[c] for c in self.completed) or '无'
            return f"版本{self.revision}：依次{labels}。已完成：{done}。请确认或拒绝；重新说话可修改。"
        if self.status == 'paused': return '任务已暂停。保留完成记录。继续任务请重新说话并确认。'
        if self.status == 'done': return '任务完成，保持悬停，请人工接管。'
        return '状态：'+self.status + ('；当前目标：'+COLORS[self.target] if self.target else '')
