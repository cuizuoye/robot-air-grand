"""Offline-tested clock bounds for a separate, non-flight bench profile.

Assumes relative clock drift <=1000 ppm during the reference lifetime.
The bench default is 0.5s; the native mission may explicitly select up to 2s.
This engineering bound is not yet validated on the physical link.
Inputs must already pass CRC and current-session vehicle identity checks.
"""
import math
from copy import deepcopy
from collections import Counter, deque


class LockClockProbe:
    MAX_RTT_NS = 100_000_000
    MAX_AGE_NS = 500_000_000
    DRIFT_PPM = 1000

    def __init__(self, source=(1, 1), target=(250, 203), *, max_reference_age_s=.5):
        if (type(max_reference_age_s) not in (int,float)
                or not math.isfinite(max_reference_age_s) or not .5<=max_reference_age_s<=2.):
            raise ValueError('Clock holdover must be between 0.5 and 2 seconds')
        self.MAX_AGE_NS = int(max_reference_age_s*1e9)
        self.source, self.target = source, target
        self.pending = None
        self.last_token = 0
        self.reference = None
        self.fault = None
        self.counts = Counter()
        self.history = deque(maxlen=96)

    def summary(self):
        return dict(max_rtt_ns=self.MAX_RTT_NS,max_reference_age_ns=self.MAX_AGE_NS,
            drift_ppm=self.DRIFT_PPM,counts=dict(self.counts),history=list(self.history),
            pending=self.pending,fault=self.fault)

    def request(self, now_ns):
        if type(now_ns) is not int or now_ns <= self.last_token:
            raise ValueError('Strictly increasing monotonic time required')
        if self.pending is not None and now_ns-self.pending <= self.MAX_RTT_NS:
            return None
        if self.pending is not None:
            self.counts['expired_unanswered_request'] += 1
            self.history.append(dict(event='expired_request',at_ns=now_ns,token=self.pending))
        self.pending = self.last_token = now_ns
        self.counts['requests'] += 1
        self.history.append(dict(event='request',at_ns=now_ns,token=now_ns))
        return now_ns

    def ingest(self, msg, now_ns):
        if msg.get_type() != 'TIMESYNC' or (msg.get_srcSystem(), msg.get_srcComponent()) != self.source:
            return False
        # Legacy dialects omit target extensions. Nonzero targets must match.
        if (getattr(msg, 'target_system', 0) not in (0, self.target[0])
                or getattr(msg, 'target_component', 0) not in (0, self.target[1])):
            return False
        if self.pending is None or msg.ts1 != self.pending or msg.tc1 <= 0:
            self.counts['unmatched_or_unsolicited'] += 1
            self.history.append(dict(event='unmatched_reply',at_ns=now_ns,token=msg.ts1,
                expected_token=self.pending,remote_ns=msg.tc1))
            return False
        sent = self.pending
        self.pending = None  # A matched response is consumed, even if late.
        rtt = now_ns-sent
        if not 0 <= rtt <= self.MAX_RTT_NS:
            self.counts['rtt_rejected'] += 1
            self.history.append(dict(event='rtt_rejected',at_ns=now_ns,token=sent,rtt_ns=rtt))
            return False
        if self.reference and msg.tc1 < self.reference['remote_ns']:
            self.fault = 'remote_clock_regression'
        self.reference = dict(remote_ns=msg.tc1, received_ns=now_ns,
                              lower_ns=msg.tc1-now_ns, upper_ns=msg.tc1-sent,
                              rtt_ns=rtt)
        self.counts['accepted'] += 1
        self.history.append(dict(event='accepted',at_ns=now_ns,token=sent,rtt_ns=rtt))
        return self.fault is None

    def assess(self, sample, now_ns, *, identity_verified=False, state_fault=None):
        reasons = []
        if not identity_verified: reasons.append('identity_not_fresh')
        if self.fault or state_fault: reasons.append(self.fault or state_fault)
        if sample is None: reasons.append('sample_missing')
        if self.reference is None: reasons.append('clock_missing')
        result = dict(valid=False, reasons=reasons, reference=self.reference,
                      source_age_bounds_ms=None)
        if sample is None or self.reference is None:
            return result
        ref = self.reference
        elapsed = now_ns-ref['received_ns']
        received = sample['received']
        # Use the same integer-nanosecond domain as the caller. Converting
        # now_ns back to float seconds can make a new sample <1 ns future.
        arrival_ns = now_ns-int(received*1e9) if math.isfinite(received) else None
        if not 0 <= elapsed < self.MAX_AGE_NS: reasons.append('clock_age')
        if arrival_ns is None or not 0 <= arrival_ns < self.MAX_AGE_NS: reasons.append('arrival_age')
        # Include drift during RTT and since reference, plus ms truncation.
        margin = math.ceil((abs(elapsed)+ref['rtt_ns'])*self.DRIFT_PPM/1_000_000)
        low = now_ns+ref['lower_ns']-sample['stamp']*1_000_000-1_000_000-margin
        high = now_ns+ref['upper_ns']-sample['stamp']*1_000_000+margin
        result['source_age_bounds_ms'] = [low/1e6, high/1e6]
        if low < 0: reasons.append('source_age_uncertain')
        if high >= 2_000_000_000: reasons.append('source_expired_or_uncertain')
        bits = sample['bits']
        if not bits & 4 or bits & (2|8|16|32): reasons.append('unsafe_state')
        result['valid'] = not reasons
        return result


class DeferredLockAudit:
    """Bench-only bounded recheck, NOT a live control authorization cache.

    An uncertain row remains false until a NEW matched clock reply arrives.
    Preserve the original arrival/source time and reapply all age/state gates.
    No waiting by host time alone, no clamping, no replaying old lock evidence.
    """
    WAIT_NS = 400_000_000  # Below the unchanged 500 ms arrival-age limit.
    CAPACITY = 4

    def __init__(self):
        self.pending = []

    def add(self, row, sample, now_ns):
        if row['detail']['reasons'] != ['source_age_uncertain'] or not row['accepted']:
            return
        if len(self.pending) >= self.CAPACITY:
            row['deferred_failure'] = 'queue_full'
            return
        ref = row['detail']['reference']
        row['initial_detail'] = deepcopy(row['detail'])
        row['deferred'] = True
        self.pending.append(dict(row=row, sample=deepcopy(sample),
                                 deadline=now_ns+self.WAIT_NS,
                                 initial_reference_ns=ref['received_ns']))

    def advance(self, clock, now_ns, *, identity_verified, state_fault=None, latest_sample=None):
        remaining = []
        unsafe_latest = latest_sample is not None and (
            not latest_sample['bits'] & 4 or latest_sample['bits'] & (2|8|16|32))
        for item in self.pending:
            row = item['row']
            failure = (state_fault or clock.fault or ('unsafe_later_state' if unsafe_latest else None)
                       or ('identity_not_fresh' if not identity_verified else None))
            if failure or now_ns > item['deadline']:
                row['deferred_failure'] = failure or 'clock_recheck_timeout'
                continue
            ref = clock.reference
            if (ref is None or ref['received_ns'] <= item['initial_reference_ns']
                    or ref['received_ns'] < int(item['sample']['received']*1e9)):
                remaining.append(item)
                continue
            detail = clock.assess(item['sample'], now_ns,
                                  identity_verified=identity_verified, state_fault=state_fault)
            row['detail'] = detail
            row['rechecked_at_ns'] = now_ns
            if detail['valid']:
                row['locked_fresh'] = True
                row['deferred_verified_at_ns'] = now_ns
            elif detail['reasons'] == ['source_age_uncertain']:
                remaining.append(item)
            else:
                row['deferred_failure'] = 'recheck_failed'
        self.pending = remaining

    def finish(self):
        for item in self.pending:
            item['row']['deferred_failure'] = 'trial_ended_before_clock_proof'
        self.pending = []
