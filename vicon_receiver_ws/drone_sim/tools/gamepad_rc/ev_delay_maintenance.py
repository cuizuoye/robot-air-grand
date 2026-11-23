"""OFFLINE maintenance-motion scope, NOT HOME static/flying acceptance.

No port, executor, encoder, HOME command, transport or live permit consumer.
Consumes the existing StartupHome and StaticLockGate proofs. The original
stationary sender is NOT called with a forged stationary=True observation.
Manual motion requires props removed or explicit installed-prop motor-power isolation;
its only output is a packet-policy decision, never a flight-ready grant.
"""
from dataclasses import dataclass
from copy import deepcopy
import math
import struct

import compact_vision
import low_bandwidth_vision
from mavlink_readonly_probe import readonly_message
from ekf_ground_acceptance import result

LIVE_ENTRY_ENABLED = False
WINDOW_NS = 15_000_000_000 # <= existing 20s source window, no extension
POSE_LIMIT = 480          # <= existing 640 packets, at most existing 32Hz


@dataclass(frozen=True)
class MaintenanceObservation:
    now_ns: int
    uid: str
    firmware: str
    fc_boot: str
    identity_received_ns: int
    attitude_received_ns: int
    vicon_received_ns: int
    gamepad_generation: str
    tilt_difference_deg: float
    props_removed: bool = False
    independent_cutoff_ready: bool = False
    scope_manually_approved: bool = False
    log_ready: bool = False
    source_valid: bool = False
    config_readonly_unchanged: bool = False
    recorder_healthy: bool = False
    attitude_source_fresh: bool = False
    # Optional real-owner continuity proof: original UID receipt is retained.
    # A fresh CRC-valid attitude on the continuously owned, same-boot route
    # supplies continuity; it must never be replaced by the assessment time.
    identity_continuity_received_ns: int | None = None
    props_installed: bool = False
    battery_disconnected: bool = False
    usb_only: bool = False
    motor_power_isolated: bool = False
    # Host arrival alone is NOT proof the FC produced a fresh attitude sample.


class IsolatedGroundState:
    """Actual unarmed heartbeat + existing clock proof, never output-lock proof.

    Only the explicitly motor-isolated maintenance scope may use this state.
    It issues no commands and never fabricates HOME, RC or lockdown bits.
    """
    def __init__(self, source=(1, 1), target=(250, 213)):
        from lock_clock_probe import LockClockProbe
        self.source, self.status = source, self
        self.clock = LockClockProbe(source, target)
        self.heartbeat = None
        self.fault = None
        self.started = False

    def ingest(self, msg, now_ns):
        if (msg.get_srcSystem(), msg.get_srcComponent()) != self.source: return
        if msg.get_type() == 'TIMESYNC': return self.clock.ingest(msg, now_ns)
        if msg.get_type() == 'HEARTBEAT':
            self.heartbeat = (now_ns, bool(msg.base_mode & 128))
            if self.heartbeat[1]: self.fault = 'armed_during_isolated_capture'

    def assess(self, now_ns, *, identity_verified=False):
        ref = self.clock.reference
        reasons = []
        if not identity_verified: reasons.append('identity_missing')
        if not self.heartbeat or not 0 <= now_ns-self.heartbeat[0] < 2_000_000_000:
            reasons.append('unarmed_heartbeat_missing_or_stale')
        elif self.heartbeat[1]: reasons.append('armed')
        if not ref or not 0 <= now_ns-ref['received_ns'] < self.clock.MAX_AGE_NS:
            reasons.append('clock_missing_or_stale')
        if self.clock.fault: reasons.append(self.clock.fault)
        if self.fault: reasons.append(self.fault)
        if self.started and reasons: self.fault = self.fault or reasons[0]
        return dict(valid=not reasons, reasons=reasons, output_lock_proven=False)

    def begin(self, now_ns, *, identity_verified=False):
        if self.started or not self.assess(now_ns, identity_verified=identity_verified)['valid']: return False
        self.started = True
        return True

    def stop(self): self.fault = self.fault or 'isolated_capture_closed'


class MaintenanceCapture:
    """One shot, fault latched. All inputs are proof results, not hardware grants."""
    def __init__(self, *, home, lock, uid, fc_boot, gamepad_generation, created_ns, start_policy='home'):
        if start_policy not in ('home', 'motor_isolated'):
            raise ValueError('unknown_maintenance_start_policy')
        if start_policy == 'motor_isolated' and (home is not None or type(lock) is not IsolatedGroundState or gamepad_generation):
            raise ValueError('explicit_isolated_state_without_home_or_gamepad_required')
        if not uid or not fc_boot or (start_policy == 'home' and not gamepad_generation) or type(created_ns) is not int:
            raise ValueError('bound_session_identity_required')
        self.start_policy = start_policy
        self.home, self.lock = home, lock
        self.uid, self.boot, self.pad = uid, fc_boot, gamepad_generation
        self.created_ns, self.previous_ns = created_ns, created_ns
        self.state, self.fault = 'WAITING', None
        self.started_ns = self.last_pose_ns = None
        self.pose_count = 0
        self.last_heartbeat_ns = None

    def fail(self, reason):
        self.fault = self.fault or reason
        self.state='FAILED'
        return False

    def _check(self, obs):
        if self.state in ('CLOSED','FAILED'):
            return False
        now=obs.now_ns
        if type(now) is not int or now<self.previous_ns:
            return self.fail('host_clock_regression')
        self.previous_ns=now
        if (obs.uid!=self.uid or obs.firmware!='1.14.3' or obs.fc_boot!=self.boot
                or obs.gamepad_generation!=self.pad):
            return self.fail('identity_boot_or_gamepad_changed')
        removed = obs.props_removed is True and obs.props_installed is False
        isolated = (obs.props_removed is False and obs.props_installed is True
                    and obs.battery_disconnected is True and obs.usb_only is True
                    and obs.motor_power_isolated is True)
        if not (removed or isolated):
            return self.fail('props_removed_or_isolated_usb_required')
        if self.start_policy == 'motor_isolated' and not all(getattr(obs,k) is True for k in
                ('battery_disconnected','usb_only','motor_power_isolated')):
            return self.fail('motor_power_isolation_required')
        for name in ('independent_cutoff_ready','scope_manually_approved',
                     'log_ready','source_valid','config_readonly_unchanged','recorder_healthy',
                     'attitude_source_fresh'):
            if getattr(obs,name) is not True:
                return self.fail(name)
        identity_stamp = obs.identity_received_ns
        if obs.identity_continuity_received_ns is not None:
            if (type(identity_stamp) is not int or not self.created_ns <= identity_stamp <= now
                    or obs.identity_continuity_received_ns != obs.attitude_received_ns
                    or obs.identity_continuity_received_ns < identity_stamp):
                return self.fail('identity_continuity_invalid')
            identity_stamp = obs.identity_continuity_received_ns
        if type(identity_stamp) is not int or not 0 <= now-identity_stamp < 500_000_000:
            return self.fail('identity_received_ns_stale')
        for name,limit in (
                           ('attitude_received_ns',500_000_000),('vicon_received_ns',150_000_000)):
            stamp=getattr(obs,name)
            if type(stamp) is not int or not 0<=now-stamp<limit:
                return self.fail(name+'_stale')
        # Before motion, retain the stationary mounting/alignment check. During
        # this isolated maintenance capture, latest FC and Vicon samples are NOT
        # time-paired: their difference contains the very delay being measured.
        # Dynamic agreement is assessed from the recorded, paired ULog offline;
        # it cannot be an instantaneous 5-degree veto on valid manual motion.
        if (not math.isfinite(obs.tilt_difference_deg)
                or not 0 <= obs.tilt_difference_deg <= 180
                or self.started_ns is None and obs.tilt_difference_deg > 5):
            return self.fail('source_FC_tilt_disagreement')
        # HOME changes hardware state (PX4 command185,param1=2); never auto-issued.
        # Proof must have been established AFTER this maintenance session began.
        if self.start_policy == 'home' and (not self.home.verified or self.home.aborted or self.home.press is None
                or self.home.release is None or self.home.ack_at is None
                or int(self.home.press*1e9)<self.created_ns
                or not self.created_ns<=int(max(self.home.ack_at,self.home.release)*1e9)<=now):
            return self.fail('no_current_manual_HOME_proof')
        if not self.lock.assess(now,identity_verified=True)['valid']:
            return self.fail('disarmed_locked_or_TIMESYNC_invalid')
        if self.started_ns is not None and now-self.started_ns>=WINDOW_NS:
            return self.fail('maintenance_window_ended')
        if self.last_pose_ns is not None and now-self.last_pose_ns>150_000_000:
            return self.fail('source_send_gap')
        return True

    def begin(self, obs):
        if self.state!='WAITING' or not self._check(obs):
            return False
        if not self.lock.begin(obs.now_ns,identity_verified=True):
            return self.fail('lock_begin_failed')
        self.started_ns=obs.now_ns
        self.state='CAPTURING'
        return True

    def check_preparation(self, obs):
        """Same HOME/isolation/freshness checks, without starting the 15s window."""
        return self.state == 'WAITING' and self.started_ns is None and self._check(obs)

    def check(self, obs):
        return self.state=='CAPTURING' and self._check(obs)

    def permits(self, msg, obs, *, noise_parameters=None):
        """Pure TX whitelist; original transport owner must enforce it.

        No manual HOME command or shell is available inside dynamic capture.
        Only same-session requested TIMESYNC, original GCS heartbeat (<=1Hz)
        and existing VPE encoding. No new telemetry-rate commands.
        Every forbidden attempt fails closed. Normal/exceptional close is silent.
        """
        if not self.check(obs):
            return False
        if msg.get_type()=='HEARTBEAT':
            if (not readonly_message(msg) or self.last_heartbeat_ns is not None
                    and obs.now_ns-self.last_heartbeat_ns<1_000_000_000):
                return self.fail('heartbeat_fields_or_rate')
            self.last_heartbeat_ns=obs.now_ns
            return True
        if msg.get_type()=='TIMESYNC':
            ok=(msg.tc1==0 and msg.ts1==self.lock.clock.pending
                and 0 <= obs.now_ns-msg.ts1 < self.lock.clock.MAX_RTT_NS
                and getattr(msg,'target_system',0) in (0,1)
                and getattr(msg,'target_component',0) in (0,1))
            return bool(ok) if ok else self.fail('unrequested_TIMESYNC')
        if msg.get_type()!='VISION_POSITION_ESTIMATE':
            return self.fail('forbidden_TX:'+msg.get_type())
        valid=(compact_vision.pose_allowed(msg) if noise_parameters is None
               else low_bandwidth_vision.pose_allowed(msg,noise_parameters))
        if not valid or self.pose_count>=POSE_LIMIT:
            return self.fail('pose_encoding_or_budget')
        if self.last_pose_ns is not None and obs.now_ns-self.last_pose_ns<31_250_000:
            return self.fail('pose_rate_over_32Hz')
        self.pose_count+=1
        self.last_pose_ns=obs.now_ns
        return True

    def close(self):
        # No restore transaction, PARAM_SET, HOME clear or retry, even on error.
        if self.state!='FAILED':
            self.state='CLOSED'
        self.lock.stop()
        return result(state=self.state,reasons=[self.fault] if self.fault else [],
                      pose_count=self.pose_count,parameter_writes=0,flight_permit_created=False)


class MaintenanceRecordingSession:
    """Offline-tested callback glue, NOT a live executor or hardware permit.

    The original single transport owner must call before_tx with its packed
    ORIGINAL message and after_tx with the decoded EXACT successful wire
    message, before its write and only after a
    full successful write, and retain its original reader/CRC/identity guards.
    There is no sender, serial/ROS API, clock generator or logger lifecycle
    here. Creating this object never enables LIVE_ENTRY_ENABLED.

    Raw source callbacks need not be stationary in this separate maintenance
    scope. They do NOT bypass or invoke the normal static acceptance sender.
    All callbacks are serialized by the owner; no receiver thread is added.
    """
    def __init__(self, gate, recorder):
        from ev_delay_sources import AtomicViconAdapter
        self.gate, self.recorder = gate, recorder
        self.adapter = AtomicViconAdapter(recorder)
        self.pending = None
        self.closed = False
        self.sent_poses = 0
        self._report = None

    def abort(self, reason):
        self.gate.fail(reason)
        self.recorder.fault = self.recorder.fault or self.gate.fault
        # No recovery write, HOME command, retry or transport action.
        self.gate.lock.stop()
        self.pending = None
        return False

    def begin(self, observation):
        if self.closed or self.gate.state != 'WAITING':
            return False
        if self.recorder.fault or not self.gate.begin(observation):
            return self.abort(self.recorder.fault or self.gate.fault or 'maintenance_begin_failed')
        return True

    def observe_frame(self, sample, *, callback_ns):
        if self.closed or self.gate.state != 'CAPTURING':
            return False
        if not self.adapter.observe(sample, callback_ns=callback_ns):
            return self.abort(self.adapter.fault or 'atomic_source_failed')
        return True

    def observe_clock(self, *, at_ns):
        if self.closed or self.gate.state != 'CAPTURING':
            return False
        try:
            if not self.recorder.observe_clock_decision(self.gate.lock.clock, at_ns=at_ns):
                return self.abort('clock_event_missing_or_recorder_overflow')
        except (ValueError, TypeError, KeyError) as exc:
            return self.abort('clock_record:' + str(exc))
        return True

    def before_tx(self, message, observation, *, noise_parameters=None):
        """Policy decision only; owner remains responsible for the real write.

        Use the owner's original encoder object for the existing encoding
        policy. after_tx uses the decoded float32 wire object for matching;
        these have intentionally different floating-point representations.
        One outstanding write only. No message is sent by this method.
        """
        if self.closed or self.gate.state != 'CAPTURING':
            return False
        if self.pending is not None:
            return self.abort('previous_write_not_completed')
        if self.recorder.fault or not self.gate.permits(message, observation,
                                                       noise_parameters=noise_parameters):
            return self.abort(self.recorder.fault or self.gate.fault or 'TX_policy_failed')
        try:
            packet = bytes(message.get_msgbuf())
            if not packet:
                raise ValueError('original_packed_wire_bytes_required')
            self.pending = (self._wire_fields(message.to_dict()), observation.now_ns, packet)
        except (AttributeError, TypeError, ValueError, OverflowError, struct.error) as exc:
            return self.abort('wire_message_snapshot:' + str(exc))
        return True

    @staticmethod
    def _wire_fields(value):
        # The three allowed message types have float32 or integer fields.
        # Preserve wire precision and NaN covariance without loosening the
        # existing original-encoder policy or comparing NaN == NaN.
        if type(value) is float:
            return struct.pack('<f', value)
        if isinstance(value, dict):
            return {k: MaintenanceRecordingSession._wire_fields(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [MaintenanceRecordingSession._wire_fields(v) for v in value]
        return value

    def after_tx(self, message, observation, *, tx_start_ns, tx_end_ns,
                 callback_ns, source_stamp_ns=None, source_frame_id=None):
        """Record a completed write without inventing send/source timestamps.

        If the write is partial or fails the owner calls abort(), never this
        method. A late exact FrameSample is joined by the existing adapter.
        USB import time will remain separate from all of these timestamps.
        """
        if self.closed or self.gate.state != 'CAPTURING':
            return False
        pending, self.pending = self.pending, None
        try:
            if (pending is None or self._wire_fields(message.to_dict()) != pending[0]
                    or bytes(message.get_msgbuf()) != pending[2]):
                raise ValueError('unapproved_or_changed_write')
            if (any(type(v) is not int for v in
                    (tx_start_ns, tx_end_ns, callback_ns, observation.now_ns))
                    or not pending[1] <= tx_start_ns <= tx_end_ns <= callback_ns
                    or observation.now_ns != callback_ns):
                raise ValueError('actual_TX_observation_order_required')
            kind = message.get_type()
            if not self.recorder.record('calibration_tx', at_ns=callback_ns,
                    message_type=kind, route=self.adapter.route, tx_start_ns=tx_start_ns,
                    tx_end_ns=tx_end_ns):
                raise ValueError('recorder_overflow')
            if kind == 'VISION_POSITION_ESTIMATE':
                if type(source_stamp_ns) is not int or not source_frame_id:
                    raise ValueError('exact_source_header_required')
                if not self.adapter.observe_sent(message, source_stamp_ns=source_stamp_ns,
                        source_frame_id=source_frame_id, tx_start_ns=tx_start_ns,
                        tx_end_ns=tx_end_ns, callback_ns=callback_ns, route=self.adapter.route):
                    raise ValueError(self.adapter.fault or 'sent_source_join_failed')
                self.sent_poses += 1
            if not self.gate.check(observation):
                raise ValueError(self.gate.fault or 'post_TX_proof_failed')
        except (ValueError, TypeError, KeyError, AttributeError, OverflowError, struct.error) as exc:
            return self.abort('TX_observation:' + str(exc))
        return True

    def close(self):
        """No I/O or analysis. Export recorder only AFTER owner stops inputs.

        callbacks_closed_cleanly means callback bookkeeping completed, not that
        dynamics, native ULog, clock uncertainty or EV_DELAY were validated.
        It can never create a flight/control preparation authorization.
        """
        if self._report is not None:
            return deepcopy(self._report)
        if self.pending is not None:
            self.abort('exit_with_uncompleted_write')
        source = self.adapter.finish()
        if self.gate.state == 'WAITING':
            self.abort('session_never_started')
        if not source['complete'] or self.recorder.fault:
            self.abort(self.recorder.fault or self.adapter.fault or 'source_join_incomplete')
        if self.sent_poses != self.gate.pose_count:
            self.abort('permitted_and_completed_pose_counts_differ')
        if not self.sent_poses:
            self.abort('no_completed_pose_write')
        policy = self.gate.close()
        self.closed = True
        self._report = result(callbacks_closed_cleanly=policy['state'] == 'CLOSED',
            reasons=policy['reasons'], maintenance=policy, atomic_source=source,
            completed_pose_writes=self.sent_poses, parameter_writes=0,
            parameter_approved=False, flight_permit_created=False,
            calibration_performed=False, real_IO_integrated=False)
        return deepcopy(self._report)
