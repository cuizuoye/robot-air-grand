"""Nonblocking fixed-read EKF/Commander collector for ONE serial owner.

Reuses existing NSH transaction isolation and topic parsers. Does not open a
port, wait/sleep, alter rates/parameters, publish poses or send controls.
Missing/slow/CRC-damaged replies cannot become a new audit. The owner must
continue servicing HOME and Vicon between every call.
"""
from collections import deque
from copy import deepcopy
import math
import re
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/uav_coord_bridge'))

from diagnostic_transaction import DiagnosticTransaction
from estop_only_policy import parse_actuator_snapshot
from ekf_compare_readonly_policy import parse_selector
from ground_estimator_settlement import GroundEstimatorSettlement
from usb_fusion_policy import parse_topic, aid_topic_for_estimator
import usb_combined_fusion_policy as combined
import compact_vision
from height_readonly_policy import absent_reply
from rc_watch import NATIVE_AUDIT_MAX_AGE_S, NATIVE_COMMANDER_REFRESH_S, NATIVE_RC_REFRESH_S, NATIVE_AUX_STATUS_MAX_AGE_S
from lock_clock_probe import LockClockProbe


COMMANDS = {**combined.COMMANDS, 'commander': b'listener vehicle_status -n 1\n'}
COMMANDS['actuator'] = b'listener actuator_armed -n 1\n'
COMMANDS['visual_series'] = compact_vision.SERIES_COMMAND
COMMANDS['visual_series_ground'] = b'listener vehicle_visual_odometry -n 13\n'
COMMANDS['visual_snapshot'] = b'listener vehicle_visual_odometry -n 1\n'
COMMANDS['critical_status'] = b'listener vehicle_local_position -n 1;listener vehicle_status -n 1\n'
NATIVE_TOPICS = ('input_rc','manual_control_setpoint','manual_control_switches','failsafe_flags')
COMMANDS.update({'native_'+topic: f'listener {topic} -n 1\n'.encode() for topic in NATIVE_TOPICS})


def message(encoder, key):
    payload = b'' if key == 'poll' else COMMANDS[key]
    return encoder.serial_control_encode(10, 2, 0, 0, len(payload), payload.ljust(70, b'\0'))


def allowed(msg):
    return bool(msg.get_type() == 'SERIAL_CONTROL'
        and (msg.device,msg.flags,msg.timeout,msg.baudrate) == (10,2,0,0)
        and any(msg.count == len(payload) and bytes(msg.data) == payload.ljust(70,b'\0')
                for payload in (b'', *COMMANDS.values())))


def parse(key, text):
    if key == 'critical_status':
        # Do not let a missing field in the first topic be supplied by the
        # second topic's timestamp or similarly named field.
        blocks=re.split(r'(?=TOPIC:)',text)
        def one(topic):
            matches=[b for b in blocks if re.match(r'TOPIC:\s*'+topic+r'\b',b)]
            return matches[0] if len(matches)==1 else ''
        local=parse('local_position',one('vehicle_local_position'))
        commander=parse('commander',one('vehicle_status'))
        return dict(local=local,commander=commander,timestamp=local['timestamp']) if local and commander else None
    if key.startswith('native_'):
        from rc_watch import NativeRCMonitor
        topic=key[7:]; ints,bools=NativeRCMonitor.TOPICS[topic]
        return parse_topic(text,topic,integer_fields=ints,bool_fields=bools)
    if key == 'visual_series':
        return compact_vision.visual_series(text)
    if key == 'visual_series_ground':
        return compact_vision.visual_series(text,samples=13)
    if key == 'visual_snapshot':
        return compact_vision.visual_series(text,samples=1)
    if key == 'actuator':
        return parse_actuator_snapshot(text)
    if key == 'selector':
        return parse_selector(text)
    if key == 'commander':
        return parse_topic(text,'vehicle_status',('timestamp','arming_state','nav_state'),
                           ('pre_flight_checks_pass',))
    if key == 'local_position':
        local = combined.parse_local_position(text)
        resets = parse_topic(text,'vehicle_local_position',
                            ('xy_reset_counter','z_reset_counter','heading_reset_counter'))
        return {**local,**resets} if local and resets else None
    if key == 'failsafe':
        return combined.parse_failsafe(text)
    for prefix, parser in (('flags_',combined.parse_flags),('status_',combined.parse_status),
                           ('aid_',combined.parse_aid),('height_aid_',combined.parse_height_aid),
                           ('yaw_aid_',combined.parse_yaw_aid)):
        if key.startswith(prefix):
            return parser(text)
    raise ValueError('No parser for non-fixed audit key')


class MissionAuditCollector:
    """ActuatorQuery-compatible nonblocking audit source, not a ready flag.

    One initial healthy selector determines queries, aid topic slots are
    discovered by estimator_instance (never assumed equal to EKF index), and
    a final selector brackets each row. Fresh actuator reads are interleaved.
    Full diagnostic rows have a separate budget from fast state feedback.
    """
    fresh_age = .5
    def __init__(self, heading_provider=None, *, ground_status_max_age=.5,
                 offline_observe_armed=False, native_rc=None):
        # A bounded ground-monitor tolerance, never a mission/flight gate.
        if type(ground_status_max_age) not in (int,float) or ground_status_max_age not in (.5,1.):
            raise ValueError('Ground status age must be 0.5 or 1.0 seconds')
        self.fresh_age = float(ground_status_max_age)
        if type(offline_observe_armed) is not bool or (offline_observe_armed and self.fresh_age != .5):
            raise ValueError('Armed observation requires explicit strict offline mission scope')
        self.offline_observe_armed = offline_observe_armed
        self.transaction = DiagnosticTransaction()
        self.heading_provider = heading_provider
        self.key = None
        self.requested_at = None
        self.attempts = 0
        self.failures = []
        self.last_snapshot = None
        self.queue = deque(['actuator','selector'])
        self.row = {}
        self.row_started = None
        self.mapping = {}
        self.discover = {}
        self.signature = None
        self.completed = deque()
        self.receive_groups = deque()
        self.rejected_rows = []
        self.unavailable_aid_slots = {}
        self.noncurrent_actuator_reads = []
        self.actuator_retry_after = 0.
        self.reads = 0
        self.last_now = None
        self._disabled = False
        self._full_audit_enabled = True
        self._restoration_only = False
        self._visual_snapshots = 0
        self.next_visual_at = 0.
        self.native_rc = native_rc
        self.status_updates = deque()
        self.critical_samples = {}
        self.reply_durations = {}

    def query_budget(self, key):
        """Nonpreemptible request-to-prompt cost, not a freshness extension.

        Cold-start estimates cover archived 1.14.3 full prints (not the
        small parser field subset). Native missions use a decaying high-water
        estimate, so one scheduling stall cannot permanently starve audits.
        Estimates include a margin but are not guarantees.
        """
        family=re.sub(r'_\d+$','',key)
        initial={'actuator':.04,'native_input_rc':.06,'native_manual_control_setpoint':.06,
            'native_manual_control_switches':.06,'native_failsafe_flags':.10,
            'failsafe':.10,'flags':.12,'status':.14,'selector':.08,
            'aid':.06,'height_aid':.06,'yaw_aid':.06,'local_position':.08,
            'commander':.09,'critical_status':.16,'visual_snapshot':.06}.get(family,.15)
        return self.reply_durations[family]+.005 if family in self.reply_durations else initial

    @property
    def pending_at(self):
        return self.requested_at if self.transaction.phase == 'pending' else None

    @property
    def buffer(self):
        return self.transaction.buffer

    @property
    def draining(self):
        return self.transaction.phase == 'draining'

    @property
    def disabled(self):
        return self._disabled or self.transaction.phase == 'blocked'

    @property
    def fault(self):
        if self.disabled:
            return self.failures[0]['reason'] if self.failures else 'audit_reply_boundary_or_retry_budget_lost'
        return None

    def _clock(self, now):
        if not math.isfinite(now) or now < 0 or self.last_now is not None and now < self.last_now:
            self._disabled = True
            raise ValueError('Monotonic audit owner clock required')
        self.last_now = now

    def _damage(self, reason, now):
        if self.transaction.phase != 'pending':
            return
        self.failures.append(dict(at=now,reason=reason,key=self.key,requested_at=self.requested_at,
                                  reply_tail=self.transaction.buffer[-4096:]))
        self.transaction.damage(reason,now)

    def damaged_packet(self, now):
        self._clock(now)
        self._damage('audit_reply_damaged_packet',now)

    def live_lockdown(self, now, *, max_age=None):
        row = self.last_snapshot
        limit = self.fresh_age if max_age is None else min(self.fresh_age,max_age)
        if (not row or not 0 <= now-row['at'] <= limit
                or row['fc_age_at_receive_s']+now-row['at'] > limit+(.5 if self.native_rc else 0)):
            return None
        return row['lockdown']

    def action(self, now, identity_fresh, *, full_audit=True, restoration_only=False):
        self._clock(now)
        if restoration_only and not self._restoration_only:
            self._restoration_only=True
            self.queue.clear();self.row={};self.row_started=None
            self.completed.clear();self.receive_groups.clear()
            if self.key is not None and self.key!='actuator':
                # Stop interpreting this reply, but retain its evidence and
                # wait for the real prompt. Never overlap shell commands.
                self._damage('audit_cancelled_for_ground_restoration',now)
        previous_failures = len(self.transaction.failures)
        self.transaction.tick(now)
        if len(self.transaction.failures) != previous_failures:
            self.failures.append(dict(at=now,reason='audit_reply_timeout',key=self.key))
        if not identity_fresh or self.disabled:
            return None
        if not self.transaction.idle:
            return 'poll' if self.transaction.poll_due(now) else None
        if self._restoration_only:
            full_audit=False
            if self.key!='actuator':self.key=None;self.attempts=0
            self.queue.clear();self.queue.append('actuator')
        if self.key is None and full_audit != self._full_audit_enabled:
            # Do not request unpublished EV topics before fusion starts. Never
            # abandon an in-flight shell reply or reuse a partial old row.
            visual_pending = next((key for key in self.queue if key.startswith('visual_')),None)
            self.queue.clear()
            if visual_pending:self.queue.append(visual_pending)
            self.queue.append('actuator')
            if full_audit:self.queue.append('selector')
            self.row = {};self.row_started = None
            self._full_audit_enabled = full_audit
        if self.key is not None:  # Damaged transaction drained: same command, clean epoch only.
            if self.attempts >= self.transaction.max_attempts:
                self._disabled = True
                return None
        else:
            if not self.queue and full_audit:
                self.queue.extend(('selector',) if self.native_rc else ('actuator','selector'))
            if not self._restoration_only and self.native_rc is not None:
                # Refresh the earliest expiry before a full RC batch or an EKF
                # query consumes its remaining budget. Never overlap a reply.
                due=sorted(((self.native_rc.topic_timing(t,now,max_age=NATIVE_RC_REFRESH_S)['query_headroom_s'],i,t)
                    for i,t in enumerate(NATIVE_TOPICS)))
                # Only native topics use the RTT-dependent upper clock bound.
                # Reserve one accepted RTT for a later bound change before a
                # nonpreemptible reply. Other feedback deadlines stay unchanged.
                deadlines=[(remaining-LockClockProbe.MAX_RTT_NS/1e9,'native_'+t)
                           for remaining,_,t in due]
                actor=self.last_snapshot
                actor_headroom=(min(.5-(now-actor['at']),
                    1.-actor['fc_age_at_receive_s']-(now-actor['at'])) if actor else -math.inf)
                deadlines.append((actor_headroom,'actuator'))
                if full_audit:
                    for name,(at,boot,stamp,limit) in self.critical_samples.items():
                        reply_age=NATIVE_COMMANDER_REFRESH_S if name=='commander' else .5
                        headroom=min(reply_age-(now-at),limit-(boot-stamp)/1e6-(now-at))
                        # Reuse individual reads; do not re-read both topics
                        # in one long transaction when only one is due.
                        deadlines.append((headroom,'local_position' if name=='local' else name))
                # Actor and critical-state queries share the same deadline
                # ordering as RC; a periodic actor read must not cut ahead
                # of a reset-counter or selected-manual sample about to expire.
                candidate=self.queue[0] if self.queue else None
                cost=self.query_budget(candidate) if candidate else 0.
                # Reserving only a fixed 150ms before expiry allowed a 100ms
                # audit to start ahead of an 80ms RC read. Compare latest
                # *start* times, accounting for both complete replies.
                # Project outstanding deadlines once, not a fixed RC batch.
                # A queued fast read is counted in cost already. Refresh only
                # one earliest-deadline topic when the diagnostic cannot fit.
                ordered=sorted(deadlines);total=cost;keys=[]
                for remaining,key in ordered:
                    if key!=candidate:total+=self.query_budget(key)
                    if remaining<=total+.02:
                        keys=[ordered[0][1]]
                        break
                self.queue=deque(k for k in self.queue if k not in keys)
                self.queue.extendleft(reversed(keys))
                if not self.queue:return None  # No busy-loop actor polling during idle preparation.
            if not self.queue:
                self.queue.extend(('actuator','selector') if full_audit else ('actuator',))
            if self.queue[0]=='visual_snapshot' and now<self.next_visual_at:
                return None  # RC refresh above still runs; leave this one-shot read queued.
            # Native proactive refresh is ordered by deadline above; only
            # force an actor query here when its actual budget is exhausted.
            # Legacy ground-only reads keep their existing headroom.
            refresh_age=(.5 if self.native_rc is not None else
                (.2 if self.queue[0].startswith('visual_series') else .6) if self.fresh_age==1. else self.fresh_age)
            lock_current=self.live_lockdown(now,max_age=refresh_age)
            needs_actuator = self.queue[0]=='actuator' or lock_current is None
            if needs_actuator and now<self.actuator_retry_after:return None
            # A slow diagnostic never licenses reusing an old actuator snapshot.
            if self.queue[0] != 'actuator' and lock_current is None:
                self.key = 'actuator'
            else:
                self.key = self.queue.popleft()
            self.attempts = 0
        self.attempts += 1
        self.requested_at = now
        self.transaction.begin(COMMANDS[self.key],now)
        return self.key

    def receive(self, text, now, boot_ms, *, boot_us=None):
        self._clock(now)
        self.transaction.receive(text,now)
        reply = self.transaction.ready_text()
        if reply is None:
            return None
        family=re.sub(r'_\d+$','',self.key)
        previous=self.reply_durations.get(family,0)
        self.reply_durations[family]=max(previous*(.8 if self.native_rc else 1.),now-self.requested_at)
        try:
            aid_key = self.key.startswith(('aid_','height_aid_','yaw_aid_'))
            absent = absent_reply(reply) if aid_key else None
            parsed = parse(self.key,reply)
            latest = parsed[-1] if isinstance(parsed,list) and parsed else parsed
            stamp = latest.get('timestamp_us',latest.get('timestamp')) if latest else None
            if self.key=='actuator' and parsed and (
                    parsed['armed'] and not self.offline_observe_armed
                    or self.native_rc is None and (parsed['manual_lockdown'] or parsed['force_failsafe'])):
                self._disabled=True
                raise RuntimeError('Unsafe actuator state; stop-only monitor only')
            cached_aid = bool(aid_key and type(boot_ms) is int and type(stamp) is int
                              and boot_ms/1000-stamp/1e6 > 2)
            if aid_key and (absent or cached_aid):
                self.unavailable_aid_slots[self.key]=dict(at=now,reason=absent or 'cached_aid_not_current')
                self._accept(None,now,boot_ms)
                self.transaction.accept();self.key=None;self.reads+=1
                return None  # Missing aid NEVER becomes a valid fusion audit.
            if self.key=='actuator' and parsed and type(boot_ms) is int and type(stamp) is int:
                # The owner uses microseconds to admit this same snapshot.
                # USB replies can arrive before the slower RF clock estimate
                # catches up. Never accept/clamp a slightly future sample here
                # only to have the input reducer reject it as current proof.
                current_boot_us=boot_ms*1000 if boot_us is None else boot_us
                if type(current_boot_us) is not int or current_boot_us//1000!=boot_ms:
                    raise ValueError('Consistent exact owner boot clock required')
                source_age_s=(current_boot_us-stamp)/1e6
                repeated = self.last_snapshot and stamp==self.last_snapshot['timestamp_us']
                source_limit=NATIVE_AUX_STATUS_MAX_AGE_S+.5 if self.native_rc else self.fresh_age
                cached = source_limit < source_age_s <= 2
                clock_ahead = -.05 <= source_age_s < 0
                if repeated and self.native_rc and any(parsed[k]!=self.last_snapshot[k]
                        for k in ('armed','lockdown','manual_lockdown','force_failsafe')):
                    raise ValueError('Conflicting actuator values at one source timestamp')
                if (repeated and not self.native_rc) or cached or clock_ahead:
                    reason='repeated' if repeated else 'ahead_of_owner_clock' if clock_ahead else 'outside_ground_status_age'
                    self.noncurrent_actuator_reads.append(dict(at=now,reason=reason,
                        maximum_age_s=source_limit,source_age_s=source_age_s,
                        source_timestamp_us=stamp,owner_boot_us=current_boot_us))
                    self.noncurrent_actuator_reads=self.noncurrent_actuator_reads[-100:]
                    self.transaction.accept();self.key=None;self.reads+=1
                    self.actuator_retry_after=now+.1
                    return None  # Do NOT refresh last_snapshot or its FC/receive timestamp.
            if (parsed is None or type(boot_ms) is not int or type(stamp) is not int
                    or not -.05 <= boot_ms/1000-stamp/1e6 <= 2):
                raise ValueError('Fresh same-boot parsed topic required')
            self.unavailable_aid_slots.pop(self.key,None)
            if self.key == 'actuator':
                if self.last_snapshot and stamp < self.last_snapshot['timestamp_us']:
                    raise ValueError('Actuator source timestamp regressed')
                if abs(boot_ms/1000-stamp/1e6) > (NATIVE_AUX_STATUS_MAX_AGE_S+.5 if self.native_rc else self.fresh_age):
                    raise ValueError('Actuator state outside ground status age bound')
            if self.native_rc is not None:
                if self.key.startswith('native_'):
                    self.native_rc.observe_topic(self.key[7:],reply,now)
                    if (self.key=='native_failsafe_flags' and self.row and not self.row.get('_finished')
                            and self.requested_at>=self.row_started):
                        full=combined.parse_failsafe(reply)
                        if full:
                            # This is the same full uORB print, not an inferred
                            # RC-only health result. Reuse it within this row.
                            self.row['failsafe']=full
                            self.queue=deque(k for k in self.queue if k!='failsafe')
                elif self.key=='actuator':self.native_rc.observe_topic('actuator_armed',reply,now)
            snapshot = self._accept(parsed,now,boot_ms)
        except (ValueError,TypeError,KeyError) as exc:
            self._damage('audit_reply_invalid: '+str(exc),now)
            return None
        self.transaction.accept()
        self.key = None
        self.reads += 1
        return snapshot

    def _accept(self, parsed, now, boot_ms):
        key = self.key
        if key.startswith('native_'):return None
        if self.native_rc is not None and key in ('critical_status','local_position','commander','selector'):
            updates=(dict(local=parsed['local'],commander=parsed['commander']) if key=='critical_status'
                     else {('local' if key=='local_position' else key):parsed})
            self.status_updates.append(dict(values=deepcopy(updates),requested_at=self.requested_at,now=now))
            for name,value in updates.items():
                if name in ('local','commander'):
                    self.critical_samples[name]=(now,boot_ms*1000,value['timestamp'],.5 if name=='local' else NATIVE_COMMANDER_REFRESH_S+.5)
                    if self.row and not self.row.get('_finished') and self.requested_at>=self.row_started:
                        self.row[name]=deepcopy(value)
                        queued='local_position' if name=='local' else name
                        self.queue=deque(k for k in self.queue if k!=queued)
            # Fast reads between audits must not create a partial row before
            # its opening selector. Open rows were updated above already.
            if key in ('critical_status','local_position','commander'):return None
        if key=='visual_snapshot':
            self.receive_groups.append(dict(rows=deepcopy(parsed),requested_at=self.requested_at,now=now,query_key=key))
            self._visual_snapshots+=1
            self.next_visual_at=now+.2
            if self._visual_snapshots<3:
                # Release the shell after each single print. Interleave the
                # existing RC/actor reads; do not occupy it for 12 updates.
                self.queue.append(key)
            return None
        if key.startswith('visual_series'):
            self.receive_groups.append(dict(rows=deepcopy(parsed),requested_at=self.requested_at,now=now,query_key=key))
            return None
        if key == 'actuator':
            row = {**parsed,'at':now,'requested_at':self.requested_at,
                   'fc_age_at_receive_s':max(0,boot_ms/1000-parsed['timestamp_us']/1e6)}
            self.last_snapshot = row
            if self.row.get('_finished'):
                self._complete(now)
            return row
        if key == 'selector':
            GroundEstimatorSettlement(now).observe(parsed,now)
            signature = parsed['primary_instance'],parsed['instance_changed_count']
            if not self.row:
                if self.signature is not None and signature != self.signature:
                    self.mapping.clear()  # Pre-control switch requires explicit new slot discovery.
                self.row_started = self.requested_at
                self.signature = signature
                self.row['selector'] = deepcopy(parsed)
                primary,count = parsed['primary_instance'],parsed['instances_available']
                self.discover = {}
                for prefix in ('aid','height_aid','yaw_aid'):
                    if prefix not in self.mapping:
                        self.discover[prefix] = {}
                        self.queue.extend(f'{prefix}_{slot}' for slot in range(count))
                    else:
                        self.queue.append(f'{prefix}_{self.mapping[prefix]}')
                self.queue.extend((f'flags_{primary}',f'status_{primary}','failsafe',
                                   'local_position','commander','selector','actuator'))
            else:
                if signature != self.signature:
                    raise ValueError('EKF selector changed inside audit')
                self.row['selector'] = deepcopy(parsed)
                self.row['_finished'] = True
                if self.native_rc is not None and self.live_lockdown(now) is not None:
                    # The current actor snapshot is already admitted by the
                    # same owner. Do not delay a closed audit to read it again.
                    if 'actuator' in self.queue:self.queue.remove('actuator')
                    self._complete(now)
            return None
        if key == 'commander':
            self.row['commander'] = deepcopy(parsed)
        elif key == 'local_position' or key == 'failsafe':
            self.row['local' if key == 'local_position' else key] = deepcopy(parsed)
        elif key.startswith(('flags_','status_')):
            self.row[key.split('_')[0]] = deepcopy(parsed)
        else:
            prefix,slot = key.rsplit('_',1)
            if prefix in self.discover:
                self.discover[prefix][int(slot)] = deepcopy(parsed)
                if not any(queued.startswith(prefix+'_') for queued in self.queue):
                    candidates=[v for v in self.discover[prefix].values()
                                if v and v['estimator_instance']==self.signature[0]]
                    if not candidates:
                        self.row[prefix]=None
                    else:
                        found = aid_topic_for_estimator(self.discover[prefix],self.signature[0])
                        self.mapping[prefix] = found
                        self.row[prefix] = self.discover[prefix][found]
            else:
                if parsed is None:
                    self.mapping.pop(prefix,None);self.row[prefix]=None
                    return None
                if parsed['estimator_instance'] != self.signature[0]:
                    raise ValueError('Aid topic no longer belongs to selected EKF')
                self.row[prefix] = deepcopy(parsed)
        return None

    def _complete(self, now):
        row = deepcopy(self.row)
        row.pop('_finished',None)
        commander = row.pop('commander')
        comparison = self.heading_provider(now) if self.heading_provider else None
        row['yaw_comparison'] = comparison
        if any(row.get(prefix) is None for prefix in ('aid','height_aid','yaw_aid')):
            self.rejected_rows.append(dict(at=now,reason='current_ev_aid_unavailable_not_a_link_fault'))
        elif now-self.row_started <= (NATIVE_AUDIT_MAX_AGE_S if self.native_rc else 2.):
            self.completed.append(dict(row=row,commander=commander,requested_at=self.row_started,now=now))
        else:
            self.rejected_rows.append(dict(at=now,reason='full_diagnostic_row_expired',
                                           duration_s=now-self.row_started))
        self.row = {}
        self.row_started = None

    def take_audits(self):
        rows = list(self.completed)
        self.completed.clear()
        return rows

    def request_visual_series(self):
        key=('visual_snapshot' if self.native_rc is not None else
             'visual_series_ground' if self.fresh_age==1. else 'visual_series')
        if self.key != key and key not in self.queue:
            self.queue.appendleft(key)  # Starts only after the current command boundary.
