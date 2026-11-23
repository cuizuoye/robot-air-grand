"""Single-owner RX/audit/fusion/core preparation composition, NO live CLI.

The caller owns the ONLY serial descriptor, services HOME first, validates
PoseGate and rigid calibration, and feeds validated packets here. Fusion send
is off by default and additionally requires a separate per-message authorizer.
There is no flight authorization, auto-start, unlock, rate or arming command.
"""
from copy import deepcopy
from collections import deque

from pymavlink.dialects.v20 import common as m

from mission_live_inputs import age
from mission_audit_collector import MissionAuditCollector, message as audit_message
from mission_fusion_service import ContinuousFusionService, context_from_inputs
from flight_mission_core import MissionCore
from mavlink_readonly_probe import decode_parameter
import trs_vision_reference_policy as vision
import usb_combined_fusion_policy as fusion_policy
from ground_estimator_settlement import GroundEstimatorSettlement


class PreparationCoordinator:
    def __init__(self, inputs, writer, *, fusion_authorizer=None, ground_fusion_only=False,
                 fixed_configuration=False, fixed_profile=None):
        if inputs.component != MissionCore.component:
            raise ValueError('One shared component 220 required for HOME/mission ACK correlation')
        self.inputs = inputs
        self.ground_fusion_only=ground_fusion_only is True
        self.status_age=1. if self.ground_fusion_only else .5
        self.ground_audits=deque(maxlen=3)
        self.restore_reads_requested=self.restore_reads_started=False
        self.diagnostic_history=[]
        self.pending_receipt=None
        self.receipt_clock_waits=0
        self.receipt_query_trace=None
        self.fusion_started_at=None
        self.writer = writer
        self.fixed_configuration = fixed_configuration is True
        if fixed_profile is not None and not self.fixed_configuration:
            raise ValueError('A fixed profile must never fall back to dynamic experiments')
        if self.fixed_configuration:
            from fixed_mission_fusion import FixedMissionFusionService
            from ekf_static_profile import READ_PARAMS
            self.fusion = FixedMissionFusionService(writer, profile=fixed_profile, authorizer=fusion_authorizer)
            specs = self.fusion.profile.get('parameters', {}) if isinstance(self.fusion.profile, dict) else {}
            # Only the explicit EV-primary, barometer-disabled configuration
            # removes the legacy barometer requirement. Profile validation and
            # same-boot readback remain prerequisites for starting this service.
            self.inputs.require_baro_height = not (
                specs.get('EKF2_HGT_REF', {}).get('value') == 3
                and specs.get('EKF2_BARO_CTRL', {}).get('value') == 0)
            self.read_params = READ_PARAMS
        else:
            self.fusion = ContinuousFusionService(writer,authorizer=fusion_authorizer)
            self.read_params = vision.READ_PARAMS
        self.core = MissionCore(writer)
        self.audit = MissionAuditCollector(heading_provider=self._heading,ground_status_max_age=self.status_age)
        self.encoder = m.MAVLink(None,srcSystem=250,srcComponent=220)
        self.parameters,self.parameter_times,self.parameter_requests,self.parameter_types = {},{},{},{}
        self.configuration_wait_at = None
        self.source_pose = None
        self.waiting_reason = 'fusion_preparation_disabled'
        self.failed = None
        self.context_provider = None

    def context(self,now):
        if self.context_provider is not None:
            return self.context_provider(now)
        return context_from_inputs(self.inputs,now,ground_status_max_age=self.status_age)

    def request_restore_reads(self):
        self.restore_reads_requested=True
        self.pending_receipt=None

    def _receive_group(self,group,now):
        c=self.context(now)
        if group.get('query_key')=='visual_snapshot':
            if not self.fixed_configuration or self.inputs.native_rc is None or len(group['rows'])!=1:
                raise ValueError('Native fixed-profile single snapshot required')
            stamp=group['rows'][0].get('timestamp')
            # Reuse the existing bounded clock catch-up for a completed ground
            # snapshot. A slightly ahead publication is not yet receipt proof.
            if (type(stamp) is int and self.fusion.phase=='RECEIVE_WARMUP'
                    and c.preparation_ground() and 0<stamp-c.boot_ms*1000<=50_000):
                if self.pending_receipt is not None:raise ValueError('One pending receipt transaction only')
                self.pending_receipt=dict(group=deepcopy(group),deadline=now+.15)
                self.receipt_clock_waits+=1
                return
            self.fusion.confirm_snapshot_reception(c,group['rows'][0],requested_at=group['requested_at'])
            if self.fusion.phase=='FUSING':self.fusion_started_at=now
            return
        if group.get('query_key')=='visual_series_ground':
            if not self.ground_fusion_only or len(group['rows'])!=13:
                raise ValueError('Explicit ground-only 13-snapshot transaction required')
            self.receipt_query_trace=dict(query_key=group['query_key'],
                raw_timestamps_us=[row['timestamp'] for row in group['rows']],
                excluded_initial_timestamp_us=group['rows'][0]['timestamp'],
                selected_count=12,reason='Exclude initial possibly cached uORB snapshot; validate next 12 without filtering')
            # PX4 listener prints an existing cached sample first. Query one
            # extra publication and exclude ONLY that initial snapshot, never
            # arbitrary bad samples within the new 12-sample stream.
            group={**group,'rows':group['rows'][1:]}
        # An independently received USB publication may be a few ms ahead of
        # the RF-derived boot estimate. Do NOT accept future proof. In this
        # explicit ground mode only, retain the completed transaction briefly
        # and let the original clock/strict predicate catch up while the same
        # pose owner keeps streaming. Every other failed gate remains terminal.
        result=self.fusion.reception_checks(c,group['rows'],requested_at=group['requested_at'])
        if (self.ground_fusion_only and result['failed']==['latest_sample_current']
                and -.05<=result['latest_age_s']<0):
            if self.pending_receipt is not None:raise ValueError('One pending receipt transaction only')
            self.pending_receipt=dict(group=deepcopy(group),deadline=now+.15)
            self.receipt_clock_waits+=1
            return
        self.fusion.confirm_reception(c,group['rows'],requested_at=group['requested_at'])
        if self.fusion.phase == 'FUSING':self.fusion_started_at=now

    def _ground_audit(self,item,now):
        row=item['row'];boot=self.inputs._boot_us(now)
        if boot is None or not 0<=now-item['requested_at']<=2:
            raise ValueError('Current bounded ground fusion audit required')
        topics=('selector','flags','local','status','failsafe','aid','height_aid','yaw_aid')
        if any(type(row[k]['timestamp']) is not int or not 0<=boot-row[k]['timestamp']<=2_000_000 for k in topics):
            raise ValueError('Fresh same-boot ground fusion topics required')
        GroundEstimatorSettlement(now).observe(row['selector'],now)
        signature=tuple(row['selector'][k] for k in ('primary_instance','instance_changed_count'))
        if self.ground_audits:
            old=self.ground_audits[-1]['row']
            old_signature=tuple(old['selector'][k] for k in ('primary_instance','instance_changed_count'))
            if signature!=old_signature:self.ground_audits.clear()
            elif any(row[k]['timestamp']<=old[k]['timestamp'] for k in topics):return
        self.ground_audits.append(dict(row=deepcopy(row),at=now))

    def ground_verified(self,now):
        if (not self.ground_fusion_only or len(self.ground_audits)!=3
                or not 0<=now-self.ground_audits[-1]['at']<=2
                or not self.fusion.receive_verified or self.fusion.phase!='FUSING'
                or not self.context(now).preparation_ground()):return False
        rows=[item['row'] for item in self.ground_audits]
        return fusion_policy.fusion_evidence_ok(rows,rows[-1]['selector']['primary_instance'],
            require_baro_height=self.inputs.require_baro_height)

    def note_parameter_request(self, name, now):
        if name not in self.read_params:
            raise ValueError('Only existing fusion-scope parameter reads')
        self.parameter_requests[name] = now

    def observe_source_pose(self, stamp_ns, position_frd, quaternion_wxyz):
        # PoseGate + existing rigid/IMU transform remain the owner's source.
        if stamp_ns != self.inputs.pose_stamp:
            raise ValueError('Transformed pose must be the exact accepted current PoseGate sample')
        self.source_pose = (stamp_ns,deepcopy(position_frd),deepcopy(quaternion_wxyz))

    def _heading(self, now):
        if self.source_pose is None or self.inputs.attitude is None:
            return None
        _,position,q = self.source_pose
        if self.fusion.transferred_core is not None:
            # After handoff the service is deliberately no longer a sender.
            # Judge the actual core send timestamp, not the stopped service's
            # old last_send. Do not refresh either timestamp artificially.
            import compact_vision
            c = self.context(now)
            sent = self.core.last_vision
            if not c.link_source_ok() or sent is None or not self.fusion._bound(c):
                return None
            _, aligned_q = compact_vision.aligned_pose(position,q,self.fusion.alignment)
            return fusion_policy.compare_heading(aligned_q,self.inputs.attitude['q'],
                attitude_age_ms=c.attitude_age*1000,sent_age_ms=(now-sent)*1000,
                **self.fusion.heading_age_limits(c))
        return self.fusion.heading_comparison(self.context(now),
            position,q,self.inputs.attitude['q'])

    def ingest(self, msg, now):
        self.inputs.ingest(msg,now)
        if msg.get_type() == 'BAD_DATA':
            self.audit.damaged_packet(now)
            return
        if (msg.get_srcSystem(),msg.get_srcComponent()) != (1,1):
            return
        context = self.context(now)
        kind = msg.get_type()
        if kind == 'PARAM_VALUE':
            name = msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id,bytes) else msg.param_id.rstrip('\0')
            if name in self.parameter_requests and 0 <= now-self.parameter_requests[name] <= 4:
                self.parameters[name] = decode_parameter(msg)
                self.parameter_times[name] = now
                self.parameter_types[name] = msg.param_type
            if self.fixed_configuration:self.fusion.observe_parameter(msg,context)
            previous_phase=self.fusion.phase
            self.fusion.ingest(msg,context)
            if previous_phase!='FUSING' and self.fusion.phase=='FUSING':self.fusion_started_at=now
        elif kind == 'COMMAND_ACK':
            self.core.ack(msg,now)
        elif kind == 'SERIAL_CONTROL':
            boot_us = self.inputs._boot_us(now) if self.inputs.attitude else None
            boot_ms = boot_us//1000 if boot_us is not None else None
            snap = self.audit.receive(bytes(msg.data[:msg.count]).decode('utf8','replace'),now,boot_ms,boot_us=boot_us)
            if snap:
                self.inputs.observe_actuator(snap,now,max_age=self.status_age)
            while self.audit.status_updates:
                update=self.audit.status_updates.popleft()
                if not self.restore_reads_requested:
                    self.inputs.observe_status(update['values'],requested_at=update['requested_at'],now=now)
            # RX is processed before this turn's returning Vicon pose. Keep a
            # completed native audit in the existing queue until a current
            # heading comparison is available, rather than poisoning it with
            # None during a tolerated loop pause. Explicit bad comparisons
            # still enter the reducer; pose loss is checked independently.
            rows=(() if self.inputs.native_rc is not None and not self.restore_reads_requested
                  and self._heading(now) is None else self.audit.take_audits())
            for row in rows:
                if self.restore_reads_requested:continue
                # The final actuator reply has just been applied above.
                # Compare against that current state, not the old snapshot
                # still present while the collector was assembling the row.
                row['row']['yaw_comparison']=self._heading(now)
                if self.ground_fusion_only:self._ground_audit(row,now)
                else:self.inputs.observe_audit(row['row'],row['commander'],requested_at=row['requested_at'],now=now)
            while self.audit.receive_groups:
                group = self.audit.receive_groups.popleft()
                if self.restore_reads_requested:continue
                self._receive_group(group,now)

    def service_diagnostics(self, now, *, full_audit=True, offline_active_mission=False,
                            restoration_only=False):
        """At most one fixed diagnostic/poll; call AFTER HOME and pose service."""
        # A lost preparation input stops localization, not the ground-only
        # state reads needed to verify a safe restoration. Uncertain writes
        # remain terminal: do not overlap an unknown shell transaction.
        if self.failed=='fixed_read_dispatch_uncertain':
            return
        if (self.ground_fusion_only and full_audit and self.fusion_started_at is not None
                and 0<=now-self.fusion_started_at<1.):
            # Newly enabled EKF aid topics can initially contain unpublished
            # or transitioning values. Keep pose + actor monitoring active,
            # but collect the strict full audit after a bounded 1s settling
            # period. Waiting is not fusion proof and cannot enable a mission.
            full_audit=False
        if self.restore_reads_requested and not self.restore_reads_started and self.audit.transaction.idle:
            old=self.audit
            self.diagnostic_history.append(dict(failures=deepcopy(old.failures),key=old.key,
                transaction_epoch=old.transaction.epoch,reason='explicit ground restoration reads only'))
            self.audit=MissionAuditCollector(heading_provider=self._heading,ground_status_max_age=self.status_age,
                native_rc=old.native_rc,offline_observe_armed=old.offline_observe_armed)
            self.audit.transaction=old.transaction  # Proven idle/prompt boundary, not a reset of unknown shell state.
            self.restore_reads_started=True
        context = self.context(now)
        identity_fresh = bool(self.inputs.identity and context.heartbeat_age < 2
                              and context.epoch_valid and self.inputs.hb
                              and (self.inputs.hb['armed'] is False
                                   or offline_active_mission is True))
        action = self.audit.action(now,identity_fresh,full_audit=full_audit,
            restoration_only=bool(restoration_only or self.failed or self.restore_reads_requested))
        if action is not None:
            try:
                if self.writer(audit_message(self.encoder,action)) is not True:
                    raise OSError('Full fixed-read write acceptance not confirmed')
            except Exception:
                self.failed = 'fixed_read_dispatch_uncertain'
                raise

    def step(self, now, *, enable_fusion_preparation=False, service_diagnostics=True):
        """One nonblocking turn AFTER the owner's HOME service and RX drain.

        Audit reads use the same writer; fusion remains default-denied. This
        method never calls core.start or core.cycle and cannot fly the aircraft.
        """
        if self.failed:
            return
        context = self.context(now)
        if service_diagnostics:
            self.service_diagnostics(now)
        if not enable_fusion_preparation:
            return
        if self.fusion.authorizer is None:
            self.waiting_reason = 'separate_fusion_dispatch_authorizer_missing'
            return  # Not even an injected writer gets pose/parameter traffic.
        if self.fixed_configuration and not self.inputs.preparation_verified(now):
            self.waiting_reason=('native_rc_or_kill_state_not_ready' if self.inputs.native_rc else 'current_session_HOME_proof_missing')
            return
        if self.fusion.phase == 'NEW':
            if self.fixed_configuration:
                from ekf_static_profile import PARAMETER_MAX_AGE
                if self.configuration_wait_at is None:self.configuration_wait_at=now
                if now-self.configuration_wait_at>PARAMETER_MAX_AGE:
                    self.failed='fixed_configuration_read_timeout'
                    return
            if (not context.preparation_ground() or self.source_pose is None or self.inputs.attitude is None
                    or self.ground_fusion_only and context.actuator_age>.2
                    or set(self.read_params)-set(self.parameters)
                    or any(age(now,self.parameter_times.get(name)) > 4 for name in self.read_params)):
                self.waiting_reason = 'current_locked_state_pose_or_complete_parameter_baseline_missing'
                return
            if self.fixed_configuration:
                barrier=self.inputs.preparation_barrier()
                result=self.fusion.check_configuration(context,self.parameters,self.parameter_types,
                    self.parameter_times,self.parameter_requests,home_barrier=barrier)
                if not result['current_configuration_verified']:
                    self.waiting_reason='fixed_configuration_not_verified'
                    if (result.get('reason') or result.get('profile_errors') or result.get('mismatched')
                            or result.get('type_mismatch') or result.get('invalid_records')
                            or not result.get('configuration_usable')
                            or not result.get('firmware_binding_reviewed') or not result.get('identity_match')
                            or not result.get('task_aiding_compatible')):
                        self.failed=self.waiting_reason
                    return
                self.fusion.begin_fixed(context,parameters=self.parameters,parameter_types=self.parameter_types,
                    parameter_times=self.parameter_times,parameter_requests=self.parameter_requests,
                    home_barrier=barrier,source_q=self.source_pose[2],fc_q=self.inputs.attitude['q'])
            else:
                self.fusion.begin(context,parameters=self.parameters,parameter_times=self.parameter_times,
                                  source_q=self.source_pose[2],fc_q=self.inputs.attitude['q'])
            self.audit.request_visual_series()
        if self.fusion.phase != 'HANDED_OFF' and self.source_pose is not None:
            stamp,p,q = self.source_pose
            if stamp == self.inputs.pose_stamp:
                self.fusion.pose(context,stamp//1000,p,q)
        if self.pending_receipt is not None and self.fusion.phase=='RECEIVE_WARMUP':
            pending=self.pending_receipt;group=pending['group']
            if context.boot_ms*1000>=group['rows'][-1]['timestamp'] or now>=pending['deadline']:
                self.pending_receipt=None
                if group.get('query_key')=='visual_snapshot':
                    self.fusion.confirm_snapshot_reception(context,group['rows'][0],requested_at=group['requested_at'])
                else:
                    self.fusion.confirm_reception(context,group['rows'],requested_at=group['requested_at'])
                if self.fusion.phase=='FUSING':self.fusion_started_at=now
        self.fusion.tick(context)
        if self.ground_fusion_only:
            self.waiting_reason='ground_fusion_verified_no_handoff' if self.ground_verified(now) else self.fusion.phase
            return  # Never mark READY, prepare the mission core, or hand off.
        observation,missing = self.inputs.build(now)
        if self.fusion.phase == 'HANDED_OFF':
            if self.core.phase != 'WAIT_EXPLICIT_START':
                raise RuntimeError('An active mission needs the separately reviewed live runner, not preparation.step')
            if (observation is None or self.core.last_vision is None
                    or now-self.core.last_vision > self.core.maximum_vision_gap):
                self.core.fault = self.core.fault or 'inputs_lost_after_preparation_handoff'
                self.core.phase = 'BLOCKED'
                self.failed = self.core.fault
                return
            stamp,p,q = self.source_pose
            if stamp == self.inputs.pose_stamp:
                self.core.vision_message(observation,stamp//1000,p,q,self.fusion.alignment)
        if self.fusion.phase == 'FUSING' and observation is not None and not missing:
            self.fusion.mark_ready(context,self.inputs)
        if self.fusion.phase == 'READY' and observation is not None:
            if self.core.phase == 'PREPARE':
                self.core.prepare(observation,self.inputs.selector)
            if self.core.phase == 'WAIT_EXPLICIT_START':
                self.fusion.handoff(context,self.core)
        self.waiting_reason = None if self.fusion.phase == 'HANDED_OFF' else self.fusion.phase

    def status(self):
        return dict(scope='single-owner software preparation, not a flight runner',
                    configuration_mode='fixed' if self.fixed_configuration else 'legacy_dynamic_experiment',
                    ground_fusion_only=self.ground_fusion_only,ground_audit_rows=len(self.ground_audits),
                    restoration_diagnostic_history=self.diagnostic_history,
                    receipt_clock_waits=self.receipt_clock_waits,
                    receipt_clock_wait_pending=self.pending_receipt is not None,
                    receipt_query_trace=deepcopy(self.receipt_query_trace),
                    fusion_started_at=self.fusion_started_at,
                    waiting_reason=self.waiting_reason,failure=self.failed,
                    fusion=self.fusion.status(),core_phase=self.core.phase,
                    flight_authorized=False,auto_start_present=False)
