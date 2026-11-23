"""Read-only candidate decoder. Not wired into live acceptance or flight.

Caller must supply CRC-validated MAVLink messages and current-session identity
verification. Clock updates are from the same identified FC's ATTITUDE stream.
Reboot/clock regression requires a NEW object and new identity/HOME evidence.
"""
import math

NAME = 'OUTLOCK1'
MAGIC = 0x4c530100


class OutputLockStatus:
    def __init__(self, source=(1, 1)):
        self.source = source
        self.sample = None
        self.clock = None
        self.fault = None

    def update_clock(self, boot_ms, now):
        if not math.isfinite(now) or type(boot_ms) is not int or boot_ms < 0:
            self.fault = 'invalid_clock'
            return
        if self.clock and (boot_ms < self.clock[0] or now < self.clock[1]):
            self.fault = 'clock_regression_or_reboot'
        self.clock = (boot_ms, now)

    def ingest(self, msg, now):
        if msg.get_type() != 'NAMED_VALUE_INT' or (msg.get_srcSystem(), msg.get_srcComponent()) != self.source:
            return False
        if msg.name != NAME:
            return False
        bits, stamp = msg.value, msg.time_boot_ms
        if not math.isfinite(now) or bits & 0xffffff00 != MAGIC or not bits & 1 or stamp == 0:
            self.sample = None
            return False
        if self.sample:
            if stamp < self.sample['stamp'] or now < self.sample['received']:
                self.fault = 'sample_regression'
            if stamp == self.sample['stamp'] and bits != self.sample['bits']:
                self.fault = 'conflicting_same_timestamp'
        self.sample = dict(stamp=stamp, bits=bits, received=now)
        return not self.fault

    def locked_disarmed(self, now, *, identity_verified=False):
        return self.assess(now,identity_verified=identity_verified)['valid']

    def assess(self, now, *, identity_verified=False):
        reasons=[];result={'clock_age_s':None,'arrival_age_s':None,'source_age_ms':None,
                          'clock':self.clock,'fault':self.fault}
        if not identity_verified:reasons.append('identity_not_fresh')
        if self.fault:reasons.append(self.fault)
        if not self.sample:reasons.append('sample_missing')
        if not self.clock:reasons.append('clock_missing')
        if not math.isfinite(now):reasons.append('invalid_host_time')
        if self.sample and self.clock and math.isfinite(now):
            clock_ms,clock_at=self.clock
            age_ms=clock_ms+(now-clock_at)*1000-self.sample['stamp']
            result.update(clock_age_s=now-clock_at,arrival_age_s=now-self.sample['received'],source_age_ms=age_ms)
            if not 0<=now-clock_at<.5:reasons.append('clock_age')
            if not 0<=now-self.sample['received']<.5:reasons.append('arrival_age')
            if not 0<=age_ms<2000:reasons.append('source_future' if age_ms<0 else 'source_expired')
            if not self.sample['bits']&4 or self.sample['bits']&(2|8|16|32):reasons.append('unsafe_state')
        return dict(result,valid=not reasons,reasons=reasons)
