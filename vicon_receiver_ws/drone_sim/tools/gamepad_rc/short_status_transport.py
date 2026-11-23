"""Single-owner short-status rate transaction. No serial/device ownership.

Injected pump must continue HOME handling. Journal is persisted before SET;
restoration is verified by a fresh interval reply, not assumed from an ACK.
"""
from pymavlink.dialects.v20 import common as m


class ShortStatusTransport:
    def __init__(self, lock, encoder, journal, *, message_id=252, interval_us=200000):
        if (message_id, interval_us) not in ((252,200000),(248,500000),(32,50000),(111,100000)):
            raise ValueError('Only pinned lock/fusion diagnostic rates permitted')
        self.lock, self.encoder, self.journal = lock, encoder, journal
        self.message_id, self.interval_us = message_id, interval_us
        self.original = None
        self.changed = self.restored = self.enabled = False
        self.next_probe_ns = 0
        self.interval = self.ack = None
        self.writes = []

    def summary(self):
        return dict(original_intervals_us={} if self.original is None else {self.message_id:self.original},
                    possibly_changed=[self.message_id] if self.changed else [],
                    restoration_verified=not self.changed or self.restored,
                    writes=list(self.writes))

    def ingest(self, msg, now_ns):
        if (msg.get_srcSystem(), msg.get_srcComponent()) != self.lock.status.source:
            return
        if self.message_id == 252:
            self.lock.ingest(msg, now_ns)
        if msg.get_type() == 'MESSAGE_INTERVAL' and msg.message_id == self.message_id:
            self.interval = (now_ns, int(msg.interval_us))
        if (msg.get_type() == 'COMMAND_ACK' and msg.command == 511
                and (msg.target_system, msg.target_component) == self.lock.clock.target):
            self.ack = (now_ns, msg.result)

    def allowed(self, msg, *, identity, locked, restoring, bootstrap=False):
        if not identity:
            return False
        if msg.get_type() == 'TIMESYNC':
            return bool(self.message_id == 252 and self.enabled and msg.tc1 == 0
                        and self.lock.clock.pending is not None
                        and msg.ts1 == self.lock.clock.pending)
        if (msg.get_type() != 'COMMAND_LONG' or (msg.target_system, msg.target_component) != (1,1)
                or msg.confirmation != 0 or msg.param1 != self.message_id
                or any(getattr(msg, 'param'+str(i)) != 0 for i in range(3,8))):
            return False
        if msg.command == 510:
            return msg.param2 == 0
        if msg.command != 511 or not self.changed or self.original is None:
            return False
        restore = self.original if self.original > 0 else -1
        return msg.param2 == restore if restoring else (locked or bootstrap and self.message_id == 252) and msg.param2 == self.interval_us

    def probe(self, now_ns, send, *, identity, restoring):
        if self.message_id != 252 or not self.enabled or not identity or now_ns < self.next_probe_ns:
            return
        token = self.lock.clock.request(now_ns)
        if token is not None:
            send(self.encoder.timesync_encode(0, token))
            # 10Hz small probes give more opportunities inside the unchanged
            # 500ms reference lifetime. RTT/source/arrival limits are unchanged.
            self.next_probe_ns = now_ns + 100_000_000
        else:
            # An outstanding request still owns the token. Do not postpone a
            # new opportunity by another whole period when no probe was sent.
            self.next_probe_ns = max(now_ns+1_000_000,
                self.lock.clock.pending+self.lock.clock.MAX_RTT_NS+1)

    def _get(self, send, pump, clock):
        self.interval = None
        requested = clock()
        send(self.encoder.command_long_encode(1,1,510,0,self.message_id,0,0,0,0,0,0))
        deadline = requested + 3_000_000_000
        while clock() < deadline:
            pump()
            if self.interval and self.interval[0] >= requested:
                value = self.interval[1]
                if not 0 <= value <= 60_000_000:
                    raise RuntimeError('Unsupported short-status original interval')
                return value
        raise RuntimeError('Short-status interval read timed out')

    def _set(self, value, send, pump, clock):
        self.ack = None
        requested = clock()
        self.writes.append(dict(at_ns=requested, interval_us=value, message_id=self.message_id))
        self.journal(self.summary())  # Durable before any possible SET bytes.
        send(self.encoder.command_long_encode(1,1,511,0,self.message_id,value,0,0,0,0,0))
        deadline = requested + 3_000_000_000
        while clock() < deadline:
            pump()
            if self.ack and self.ack[0] >= requested:
                if self.ack[1] != m.MAV_RESULT_ACCEPTED:
                    raise RuntimeError('Short-status interval rejected')
                break
        else:
            raise RuntimeError('Short-status interval ACK timed out')
        settle = clock() + 150_000_000
        while clock() < settle:
            pump()
        if self._get(send, pump, clock) != (value if value > 0 else 0):
            raise RuntimeError('Short-status interval readback mismatch')

    def configure(self, send, pump, clock):
        if self.changed:
            raise RuntimeError('No repeated short-status setup')
        self.original = self._get(send, pump, clock)
        self.changed = True
        self._set(self.interval_us, send, pump, clock)
        self.enabled = True

    def restore(self, send, pump, clock):
        self.enabled = False
        if self.changed:
            self._set(self.original if self.original > 0 else -1, send, pump, clock)
            self.restored = True
        self.journal(self.summary())


class RateTransactionGroup:
    """One durable obligation for sequential rate transactions on one writer."""
    def __init__(self, journal):
        self.journal = journal
        self.states = {}

    def member(self, name):
        def record(summary):
            from copy import deepcopy
            self.states[name] = deepcopy(summary)
            self.journal(self.summary())
        return record

    def summary(self):
        originals = {}
        changed = set()
        writes = []
        for state in self.states.values():
            originals.update(state['original_intervals_us'])
            changed.update(state['possibly_changed'])
            writes.extend(state['writes'])
        return dict(original_intervals_us=originals, possibly_changed=sorted(changed),
                    restoration_verified=all(s['restoration_verified'] for s in self.states.values()),
                    writes=sorted(writes,key=lambda w:w['at_ns']))
