"""Nonblocking temporary telemetry lifecycle, reusing the measured 10/5/5Hz profile.

No device access. One owner feeds packets and transmits returned exact actions.
GET originals -> SET/ACK/GET verify -> observe -> explicit restore/ACK/GET.
Never changes HEARTBEAT, UART rate, parameters, pose, mode or arming. Every
possibly changed stream is journaled BEFORE its SET can leave this object.
"""
from copy import deepcopy
import math

from pymavlink.dialects.v20 import common as m
from trs_message_interval_trial import CANDIDATES, interval_message, allowed, rate_stats


class TemporaryTelemetryRates:
    timeout = 3.
    apply_wait = .15

    def __init__(self, *, component=202, persist=None, profile='telemetry'):
        if profile not in ('telemetry','timesync','mission'):raise ValueError('Unknown pinned rate profile')
        self.profile=profile
        self.candidates={111:100000} if profile=='timesync' else dict(CANDIDATES)
        if profile=='mission':
            # Set landed-state feedback first and restore it last. The owner
            # needs fresh ground state throughout setup and restoration.
            self.candidates={245:CANDIDATES[245],**CANDIDATES,111:100000}
        self.encoder=m.MAVLink(None,srcSystem=250,srcComponent=component)
        self.component=component
        self.persist=persist
        self.phase='READ_ORIGINALS'
        self.original={}
        self.changed=[]
        self.restored={}
        self.pending=None
        self.outgoing=None
        self.last_now=None
        self.error=None
        self.restore_errors=[]
        self.restoring=False
        self.active_at=self.active_end=None
        self.samples={key:[] for key in self.candidates}
        self.acks=[]
        self.wait_until=0.
        self.current=None

    def _clock(self,now):
        if not math.isfinite(now) or now<0 or self.last_now is not None and now<self.last_now:
            raise ValueError('Monotonic telemetry owner clock required')
        self.last_now=now

    def _save(self):
        if self.persist is not None:self.persist(self.summary())

    def _fail(self,reason):
        if self.restoring:
            self.restore_errors.append(reason)
            if self.current is not None:self.restored[self.current]=False
            self.phase='RESTORE_NEXT'
        else:
            self.error=self.error or reason
            self.phase='BLOCKED'
        self.pending=self.outgoing=None
        self._save()

    def permits(self,msg,*,fresh,locked):
        if self.outgoing is None or msg.to_dict()!=self.outgoing.to_dict():return False
        if not self.pending or msg.param1!=self.pending['id']:return False
        original=self.original.get(int(msg.param1))
        if msg.param1==111 and 111 in self.candidates:
            if (not fresh or msg.get_type()!='COMMAND_LONG'
                    or (msg.target_system,msg.target_component,msg.confirmation,msg.param1)!=(1,1,0,111)
                    or any(getattr(msg,f'param{i}')!=0 for i in range(3,8))):return False
            if msg.command==510:return msg.param2==0
            expected=(original if original and original>0 else -1) if self.restoring else 100000
            return bool(msg.command==511 and locked and original is not None and msg.param2==expected)
        return allowed(msg,safe=fresh,locked=locked,original=original)

    def _message(self,msgid,interval=None):
        if msgid not in self.candidates:raise ValueError('Message outside pinned rate profile')
        if msgid!=111:return interval_message(self.encoder,msgid,interval)
        return self.encoder.command_long_encode(1,1,510 if interval is None else 511,0,
            msgid,0 if interval is None else interval,0,0,0,0,0)

    def _get(self,msgid,now,purpose):
        self.pending=dict(kind='GET',id=msgid,at=now,purpose=purpose)
        self.outgoing=self._message(msgid)
        return self.outgoing

    def _set(self,msgid,value,now):
        self.current=msgid
        self.pending=dict(kind='SET',id=msgid,value=value,at=now)
        self.outgoing=self._message(msgid,value)
        if not self.restoring and msgid not in self.changed:self.changed.append(msgid)
        self._save()  # Durable caller journal precedes transmission, including uncertain writes.
        return self.outgoing

    def action(self,now,*,fresh=False,locked=False):
        self._clock(now)
        if self.pending is not None:
            if now-self.pending['at']>self.timeout:self._fail('interval_transaction_timeout_'+str(self.pending['id']))
            return None
        if not fresh or self.phase in ('ACTIVE','CLOSED','BLOCKED','RESTORE_FAILED'):return None
        if self.phase=='READ_ORIGINALS':
            missing=[key for key in self.candidates if key not in self.original]
            if missing:return self._get(missing[0],now,'original')
            self.phase='SET_NEXT'
        if self.phase=='SET_NEXT':
            if not locked:return None
            remaining=[key for key in self.candidates if key not in self.changed
                and not (self.profile in ('timesync','mission') and self.original[key]==self.candidates[key])]
            if remaining:return self._set(remaining[0],self.candidates[remaining[0]],now)
            self.phase='ACTIVE';self.active_at=now;self._save();return None
        if self.phase=='WAIT_APPLY' and now>=self.wait_until:
            return self._get(self.current,now,'restore' if self.restoring else 'candidate')
        if self.phase=='RESTORE_NEXT':
            remaining=[key for key in reversed(self.changed) if key not in self.restored]
            if remaining:
                if not locked:return None
                key=remaining[0];value=self.original[key] if self.original[key]>0 else -1
                return self._set(key,value,now)
            self.phase='RESTORE_FAILED' if self.restore_errors else 'CLOSED'
            self._save()
        return None

    def ingest(self,msg,now):
        self._clock(now)
        if (msg.get_srcSystem(),msg.get_srcComponent())!=(1,1):return
        if self.phase=='ACTIVE' and msg.get_msgId() in self.samples:
            self.samples[msg.get_msgId()].append(now)
        if not self.pending or not 0<=now-self.pending['at']<=self.timeout:return
        pending=self.pending
        if (pending['kind']=='SET' and msg.get_type()=='COMMAND_ACK' and msg.command==511
                and (msg.target_system,msg.target_component)==(250,self.component)):
            self.acks.append(dict(at=now,id=pending['id'],result=msg.result,restoring=self.restoring))
            if msg.result!=m.MAV_RESULT_ACCEPTED:
                self._fail('interval_set_rejected_'+str(pending['id']));return
            self.phase='WAIT_APPLY';self.wait_until=now+self.apply_wait
            self.pending=self.outgoing=None
        elif (pending['kind']=='GET' and msg.get_type()=='MESSAGE_INTERVAL'
              and msg.message_id==pending['id']):
            value=int(msg.interval_us)
            if not 0<=value<=60_000_000:
                self._fail('invalid_interval_readback');return
            if pending['purpose']=='original':
                self.original[pending['id']]=value
                self.pending=self.outgoing=None;self._save()
            else:
                key=pending['id']
                expected=self.original[key] if self.restoring else self.candidates[key]
                if value!=expected:
                    self._fail('interval_readback_mismatch_'+str(key));return
                if self.restoring:self.restored[key]=True
                self.phase='RESTORE_NEXT' if self.restoring else 'SET_NEXT'
                self.pending=self.outgoing=None;self._save()

    def begin_restore(self,now):
        self._clock(now)
        if self.restoring:return
        self.active_end=now if self.active_at is not None else None
        self.restoring=True
        self.pending=self.outgoing=None
        self.phase='RESTORE_NEXT'
        self._save()

    def dispatch_failed(self,reason):
        self._fail('interval_dispatch_uncertain: '+str(reason))

    def restoration_incomplete(self,reason):
        self.restore_errors.append(str(reason))
        self.phase='RESTORE_FAILED'
        self.pending=self.outgoing=None
        self._save()

    def summary(self):
        observed={str(key):(rate_stats(times) if len(times)>1 and times[-1]>times[0]
                            else dict(samples=len(times),rate_hz=None,max_arrival_gap_ms=None))
                  for key,times in self.samples.items()}
        # Check window boundaries too; a burst followed by silence is NOT a pass.
        end=self.active_end if self.active_end is not None else self.last_now
        measured=bool(self.active_at is not None and end-self.active_at>=5)
        for key,times in self.samples.items():
            row=observed[str(key)]
            row['max_gap_including_window_boundaries_ms']=(max(
                [times[0]-self.active_at,end-times[-1],
                 *[b-a for a,b in zip(times,times[1:])]])*1000 if times and self.active_at is not None else None)
            measured=bool(measured and row['rate_hz'] is not None and row['rate_hz']>=4
                          and row['max_gap_including_window_boundaries_ms']<=450)
        return deepcopy(dict(phase=self.phase,original_intervals_us=self.original,
            requested_intervals_us=self.candidates,possibly_changed=self.changed,restored=self.restored,
            error=self.error,restore_errors=self.restore_errors,acks=self.acks,
            active_at=self.active_at,active_end=self.active_end,observed=observed,
            measured_profile_verified=measured and self.error is None,
            restoration_verified=bool(self.restoring and self.phase=='CLOSED'
                and all(self.restored.get(key) is True for key in self.changed)),
            heartbeat_rate_writes=0,parameter_writes=0,flight_authorized=False))
