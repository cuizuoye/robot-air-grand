"""Candidate current-time lock gate for static fusion, with no device I/O.

Not flight authorization. Caller still owns identity/HOME, physical-scope,
pose, attitude, parameter and restoration guards. CRC-invalid input must not
reach the decoder. A previously proved sample is usable only after a fresh
assessment with its ORIGINAL timestamps; historical success alone grants
nothing. Once all valid evidence is lost, stop is latched.
"""
from copy import deepcopy
from lock_clock_probe import LockClockProbe
from output_lock_status import OutputLockStatus


class StaticLockGate:
    def __init__(self, source=(1, 1), target=(250, 213)):
        self.status = OutputLockStatus(source)
        self.clock = LockClockProbe(source, target)
        self.started = False
        self.fault = None
        self.uncertain_sample = None
        self.proven_sample = None

    def ingest(self, msg, now_ns):
        if msg.get_type() == 'TIMESYNC':
            return self.clock.ingest(msg, now_ns)
        return self.status.ingest(msg, now_ns / 1e9)

    def assess(self, now_ns, *, identity_verified=False):
        detail = self.clock.assess(
            self.status.sample, now_ns, identity_verified=identity_verified,
            state_fault=self.status.fault)
        sample = self.status.sample
        key = None if sample is None else (sample['stamp'], sample['received'])
        if self.uncertain_sample and self.uncertain_sample[0] != key:
            self.uncertain_sample = None
        if 'source_age_uncertain' in detail['reasons'] and self.uncertain_sample is None:
            self.uncertain_sample = (key, self.clock.reference['received_ns'])
        if self.uncertain_sample:
            ref = self.clock.reference
            # Host time passing alone cannot resolve an uncertain sample.
            if (ref is None or ref['received_ns'] <= self.uncertain_sample[1]
                    or ref['received_ns'] < int(sample['received'] * 1e9)):
                detail['reasons'].append('await_new_clock_proof')
            elif not detail['reasons']:
                self.uncertain_sample = None
        # Only timing uncertainty of an otherwise safe new sample may use
        # earlier evidence. Identity loss, decoder faults, unsafe bits, missing
        # state, stale clocks or arrival expiry NEVER take this branch.
        latest_detail = deepcopy(detail)
        if (self.started and self.proven_sample is not None
                and detail['reasons']
                and set(detail['reasons']) <= {'source_age_uncertain', 'await_new_clock_proof'}):
            previous = self.clock.assess(
                self.proven_sample, now_ns, identity_verified=identity_verified,
                state_fault=self.status.fault)
            if previous['valid']:
                detail = previous
                detail['evidence_selection'] = 'previous_still_fresh'
                detail['evidence_sample'] = deepcopy(self.proven_sample)
                detail['latest_pending'] = latest_detail
                # Do not refresh or promote this older sample.
        elif not detail['reasons'] and self.fault is None:
            self.proven_sample = deepcopy(sample)
        if self.started and detail['reasons'] and self.fault is None:
            self.fault = 'static_stream_lock_lost:' + detail['reasons'][0]
        if self.fault:
            detail['reasons'].append(self.fault)
        detail['valid'] = not detail['reasons']
        return detail

    def begin(self, now_ns, *, identity_verified=False):
        if self.started or self.fault:
            return False
        if not self.assess(now_ns, identity_verified=identity_verified)['valid']:
            return False
        self.started = True
        return True

    def stop(self):
        self.fault = self.fault or 'static_stream_closed'
