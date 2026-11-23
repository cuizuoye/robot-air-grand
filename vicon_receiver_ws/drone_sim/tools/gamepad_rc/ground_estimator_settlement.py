"""Bounded PRE-CONTROL EKF settlement; never change a flying/active gate.

Only observes automatic selection. No selection command or parameter writes.
Freeze after a healthy selector has been stable 1.5 seconds, at least two
seconds after fusion initialization. The overall window is six seconds.
After freeze, every primary/count change remains fatal, including a switch
away and back. This is never a license to rebase an existing target stream.
"""
import math


class GroundEstimatorSettlement:
    minimum_wait = 2.0
    stable_seconds = 1.5
    maximum_wait = 6.0
    maximum_changes = 2

    def __init__(self, started_at):
        if not math.isfinite(started_at) or started_at < 0:
            raise ValueError('Finite pre-control start required')
        self.started_at = self.last_at = started_at
        self.signature = None
        self.stable_since = None
        self.changes = 0
        self.frozen_signature = None
        self.rows = []

    def observe(self, selected, now, *, targets_sent=0):
        if targets_sent != 0 or not math.isfinite(now) or now < self.last_at:
            raise ValueError('Settlement only before targets; monotonic observation required')
        self.last_at = now
        instance = selected['primary_instance']
        healthy = selected.get('healthy', [])
        if (not 0 <= instance < selected['instances_available'] <= 9
                or len(healthy) != 9 or healthy[instance] is not True
                or selected['gyro_fault_detected'] or selected['accel_fault_detected']):
            raise ValueError('Selected estimator must be explicitly healthy; no IMU fault bypass')
        signature = (instance, selected['instance_changed_count'])
        self.rows.append(dict(at=now, signature=list(signature), selected=selected))
        if self.frozen_signature is not None:
            if signature != self.frozen_signature:
                raise ValueError('Estimator changed after freeze; never rebind active target')
            return True
        if now-self.started_at > self.maximum_wait:
            raise ValueError('Estimator did not settle in bounded pre-control window')
        if self.signature != signature:
            if self.signature is not None:
                self.changes += 1
            if self.changes > self.maximum_changes:
                raise ValueError('Repeated estimator switching during initialization')
            self.signature, self.stable_since = signature, now
        if (now-self.started_at >= self.minimum_wait
                and now-self.stable_since >= self.stable_seconds):
            self.frozen_signature = signature
            return True
        return False


def require_frozen_selector(selected, signature):
    if signature is None or (selected['primary_instance'], selected['instance_changed_count']) != signature:
        raise RuntimeError('Primary estimator or change counter changed after pre-control freeze; restore')
