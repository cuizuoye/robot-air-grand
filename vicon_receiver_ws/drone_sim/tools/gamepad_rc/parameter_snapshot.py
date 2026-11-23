"""Bounded static-bench configuration snapshot; never a flight authorization."""
import math


class ParameterSnapshot:
    def __init__(self):
        self.started = None
        self.failed = False

    def begin(self, now, *, fresh, identity_ok):
        if self.started is not None or not fresh or not identity_ok or not math.isfinite(now):
            raise RuntimeError('Fresh single-use configuration snapshot required')
        self.started = now

    def valid(self, now, *, identity_ok, scope_ok):
        if (self.started is None or not math.isfinite(now) or
                not 0 <= now - self.started < 20 or not identity_ok or not scope_ok):
            self.failed = True
        return not self.failed
