"""Bounded fixed-listener transactions; diagnostic loss must not kill HOME.

No transport or control commands live here. After a damaged/expired reply,
discard it and drain to a terminal prompt before issuing another listener.
Without that boundary, disable diagnostics rather than mix reply epochs.
"""
import re

from estop_only_policy import LISTENER, parse_actuator_snapshot
from persistent_shell import current_reply
from trs_only_estop_bench import state_valid


_ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
_PROMPT = re.compile(r'(?:^|\n)nsh>[ \t]*\Z')


def terminal_prompt_seen(text):
    return bool(_PROMPT.search(_ANSI.sub('', text).replace('\r', '')))


class ActuatorQuery:
    timeout = 6.0
    drain_timeout = 6.0
    interval = 2.0
    poll_interval = 0.5
    fresh_age = 2.0
    max_buffer = 16384
    max_failures = 3  # Initial failure plus at most two diagnostic retries.

    def __init__(self):
        self.pending_at = None
        self.buffer = ''
        self.draining = False
        self.drain_deadline = None
        self.disabled = False
        self.next_request = self.next_poll = 0.0
        self.failures = []
        self.last_snapshot = None

    @property
    def fault(self):
        return self.failures[0]['reason'] if self.failures else None

    def _finish_drain(self, now):
        self.buffer = ''
        self.draining = False
        self.drain_deadline = None
        self.next_request = now + self.poll_interval
        self.disabled = len(self.failures) >= self.max_failures

    def fail(self, reason, now):
        if self.pending_at is None:
            return
        self.failures.append({'at': now, 'reason': reason,
                              'requested_at': self.pending_at,
                              'incomplete_reply': self.buffer[-self.max_buffer:]})
        self.pending_at = None
        self.draining = True
        self.drain_deadline = now + self.drain_timeout
        self.next_poll = now
        if terminal_prompt_seen(self.buffer):
            self._finish_drain(now)

    def damaged_packet(self, now):
        # A corrupt frame may contain part of this listener; never certify it.
        self.fail('actuator_reply_damaged_packet', now)

    def receive(self, text, now, boot_ms):
        if self.pending_at is not None and now-self.pending_at > self.timeout:
            self.fail('actuator_reply_timeout', now)
        if self.draining and now >= self.drain_deadline:
            self.draining, self.disabled = False, True
        if self.pending_at is None and not self.draining:
            return None  # Unsolicited/late output is not a new observation.
        self.buffer += text
        if len(self.buffer) > self.max_buffer:
            self.fail('actuator_reply_overflow', now)
            self.buffer = self.buffer[-self.max_buffer:]
        if self.draining:
            if terminal_prompt_seen(self.buffer):
                self._finish_drain(now)
            return None  # Even a complete late reply is permanently discarded.
        reply = current_reply(self.buffer, LISTENER)
        if reply is None:
            return None
        snapshot = parse_actuator_snapshot(reply)
        if snapshot and (snapshot['armed'] or snapshot['manual_lockdown']
                         or snapshot['force_failsafe']):
            raise RuntimeError('Unsafe actuator state; maintenance session stopped')
        if not snapshot:
            self.fail('actuator_reply_malformed', now)
            return None
        if (not state_valid(snapshot, boot_ms)
                or (self.last_snapshot and
                    snapshot['timestamp_us'] <= self.last_snapshot['timestamp_us'])):
            self.fail('actuator_reply_stale_or_repeated', now)
            return None
        result = {**snapshot, 'at': now, 'requested_at': self.pending_at,
                  'fc_age_at_receive_s': max(0, (boot_ms*1000-snapshot['timestamp_us'])/1e6)}
        self.last_snapshot = result
        self.pending_at, self.buffer = None, ''
        self.next_request = now + self.interval
        return result

    def action(self, now, identity_fresh):
        """Return request/poll or None. Never return arbitrary shell payloads."""
        if self.pending_at is not None and now-self.pending_at > self.timeout:
            self.fail('actuator_reply_timeout', now)
        if self.draining and now >= self.drain_deadline:
            # No prompt, no safe delimiter: require an explicit new session.
            self.draining, self.disabled = False, True
        if not identity_fresh or self.disabled:
            return None
        if self.draining or self.pending_at is not None:
            if now >= self.next_poll:
                self.next_poll = now + self.poll_interval
                return 'poll'
        elif now >= self.next_request:
            self.pending_at, self.buffer = now, ''
            self.next_poll = now + self.poll_interval
            return 'request'
        return None

    def live_lockdown(self, now):
        row = self.last_snapshot
        if (not row or not 0 <= now-row['at'] <= self.fresh_age
                or row.get('fc_age_at_receive_s', 0)+now-row['at'] > self.fresh_age
                or (self.failures and self.failures[-1]['at'] >= row['at'])):
            return None
        return row['lockdown']
