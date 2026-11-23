"""Bounded NSH reply isolation, with no device or control access.

A damaged transaction is never parsed or reused. Drain to a terminal prompt
before the caller may retry the fixed command. Missing boundaries disable
diagnostics rather than creating overlapping/mixed-epoch evidence.
"""
from actuator_query import terminal_prompt_seen
from persistent_shell import current_reply


class DiagnosticTransaction:
    reply_timeout = 3.0
    drain_timeout = 3.0
    poll_interval = .25
    max_buffer = 24000
    max_attempts = 2

    def __init__(self):
        self.phase = 'idle'
        self.buffer = ''
        self.payload = None
        self.deadline = self.next_poll = 0.
        self.epoch = 0
        self.failures = []

    @property
    def idle(self):
        return self.phase == 'idle'

    def begin(self, payload, now):
        if not self.idle or not isinstance(payload, bytes) or not payload.endswith(b'\n'):
            raise RuntimeError('Diagnostic boundary not established; no overlapping command')
        self.epoch += 1
        self.payload, self.buffer, self.phase = payload, '', 'pending'
        self.deadline, self.next_poll = now + self.reply_timeout, now + self.poll_interval

    def _drained(self):
        self.phase, self.buffer, self.payload = 'idle', '', None

    def damage(self, reason, now):
        if self.phase != 'pending':
            return
        self.failures.append({'epoch': self.epoch, 'at': now, 'reason': reason,
                              'discarded_tail': self.buffer[-2048:]})
        self.phase, self.deadline, self.next_poll = 'draining', now + self.drain_timeout, now
        if terminal_prompt_seen(self.buffer):
            self._drained()

    def receive(self, text, now):
        self.tick(now)
        if self.phase not in ('pending', 'draining'):
            return  # Unsolicited/late data never starts a new reply.
        self.buffer += text
        if len(self.buffer) > self.max_buffer:
            self.damage('diagnostic_reply_overflow', now)
            self.buffer = self.buffer[-self.max_buffer:]
        if self.phase == 'draining' and terminal_prompt_seen(self.buffer):
            self._drained()

    def tick(self, now):
        if self.phase == 'pending' and now >= self.deadline:
            self.damage('diagnostic_reply_timeout', now)
        if self.phase == 'draining' and now >= self.deadline:
            self.phase = 'blocked'

    def ready_text(self):
        if self.phase == 'pending':
            # Never let parsers consume unsolicited fields before this
            # command's echo (even when a valid prompt follows later).
            return current_reply(self.buffer, self.payload)
        return None

    def accept(self):
        if self.ready_text() is None:
            raise RuntimeError('Complete clean command echo and prompt required')
        self._drained()

    def poll_due(self, now):
        self.tick(now)
        if self.phase in ('pending', 'draining') and now >= self.next_poll:
            self.next_poll = now + self.poll_interval
            return True
        return False
