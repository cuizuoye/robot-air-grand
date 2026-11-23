"""One-use, current-state start preview. No device or automatic confirmation."""
from copy import deepcopy
import math
import secrets
from ground_estimator_settlement import require_frozen_selector
from follow_target_generator import NORMAL_DESCENT_SPEED_MPS, LAND_HANDOFF_TOLERANCE_M


class FlightStartConfirmation:
    lifetime=20.

    def __init__(self,runtime):
        self.runtime=runtime;self.preview=None;self.used=False
        self.native_start_check=None

    def _observation(self):
        r=self.runtime
        if self.native_start_check is not None:self.native_start_check()
        if r.now is None:raise PermissionError('No current observations')
        if not r.ground_mode_confirmed():
            raise PermissionError('Ground Position needs ACK and actual mode confirmation')
        try:require_frozen_selector(r.inputs.selector,r.core.frozen_selector)
        except (ValueError,RuntimeError,TypeError,KeyError):
            raise PermissionError('Current estimator no longer matches the frozen start frame')
        if r.pilot_link is not None and (r.pilot_sample_at!=r.now or not r.pilot_centered or not r.pilot_rc_received()):
            raise PermissionError('Fresh centered sticks and actual RC_CHANNELS receipt required before explicit start')
        o,missing=r.inputs.build(r.now)
        if (o is None or missing or r.failure or r.stop_latched or r.closing or r.closed
                or r.cancel_requested or r.core.phase!='WAIT_EXPLICIT_START' or not o.navigation_ok() or not o.preflight_ready
                or not o.preparation_ground_ok() or not o.start_pilot_ok()
                or r.core._binding(o)!=r.core.binding):
            raise PermissionError('Fresh unchanged locked/landed readiness required, not an old report')
        if r.core.cooperative is not None:
            try:r.core.cooperative.check_start(o)
            except ValueError as exc:raise PermissionError(str(exc)) from exc
        return o

    def create(self):
        if self.used:raise PermissionError('Start already consumed; no automatic retry')
        o=self._observation();r=self.runtime
        if self.preview is not None and r.now<=self.preview['expires_at']:
            return deepcopy(self.preview)
        # An UNUSED expired token can be replaced against current readiness.
        # Old text is invalidated; the mission's frozen origin is not rebased.
        nonce=secrets.token_hex(4)
        self.preview=dict(issued_at=r.now,expires_at=r.now+self.lifetime,
            boot_session=o.boot_session,rigid_revision=o.rigid_revision,
            reset_counters=list(o.reset_counters),selector=list(r.core.frozen_selector),
            origin_ned_m=list(o.position_ned),target_ned_m=[o.position_ned[0],o.position_ned[1],o.position_ned[2]-r.core.preview.height],
            yaw_ned_rad=o.yaw_ned,height_m=r.core.preview.height,hover_seconds=r.core.preview.hover_seconds,
            arrival_criteria=r.core.preview.arrival_criteria(),
            horizontal_motion=r.core.motion.describe() if r.core.motion is not None else None,
            cooperative=r.core.cooperative.describe() if r.core.cooperative is not None else None,
            normal_landing=(dict(reference_speed_mps=NORMAL_DESCENT_SPEED_MPS,
                resting_z_ned=r.core.origin.position_ned[2],handoff_tolerance_m=LAND_HANDOFF_TOLERANCE_M,
                completion='fresh_landed_and_disarmed')
                if r.pilot_source=='native_rc' and r.core.cooperative is None else None),
            maximum_task_seconds=FlightRunController.task_seconds(r),
            confirmation_text='TAKEOFF '+nonce,flight_authorized=False,
            warning='Explicit intent only; physical per-frame authorization is separately required')
        return deepcopy(self.preview)

    def confirm(self,text):
        if self.used or self.preview is None:raise PermissionError('Missing/used start preview')
        p=self.preview;r=self.runtime
        o=self._observation()
        if (type(text) is not str or text!=p['confirmation_text']
                or not p['issued_at']<=r.now<=p['expires_at']
                or (o.boot_session,o.rigid_revision,list(o.reset_counters))!=(p['boot_session'],p['rigid_revision'],p['reset_counters'])
                or list(r.core.frozen_selector)!=p['selector']
                or (r.core.motion.spec.to_dict() if r.core.motion is not None else None)
                    != (p['horizontal_motion']['settings'] if p['horizontal_motion'] else None)
                or (r.core.cooperative.plan_sha256 if r.core.cooperative is not None else None)
                    != (p['cooperative']['plan_sha256'] if p['cooperative'] else None)
                or math.dist(o.position_ned,p['origin_ned_m'])>.03
                or abs(math.atan2(math.sin(o.yaw_ned-p['yaw_ned_rad']),math.cos(o.yaw_ned-p['yaw_ned_rad'])))>.05):
            raise PermissionError('Confirmation text/age/pose/epoch changed; never silently rebase the target')
        self.used=True  # BEFORE a possibly failed or partial output-enable write.
        r.request_start(operator_confirmed=True)
        return True


class FlightRunController:
    """One stateful service for future approved native hosts, no resource opens.

    Ordinary faults request LAND; HOME remains the runtime's priority path.
    An unconfirmed landing/unknown airborne state stays resident and requests
    operator intervention, not automatic cleanup or an invented safe landing.
    """
    maximum_task_seconds=120.

    @classmethod
    def task_seconds(cls, runtime):
        if runtime.core.cooperative is not None:
            return 600.  # Same bounded session as the confirmed car entry.
        motion = runtime.core.motion
        # Preparation time is still excluded. Derive a long route's budget
        # from its duration instead of cutting it off at the old hover budget.
        descent = runtime.core.normal_descent_budget_s if runtime.pilot_source=='native_rc' else 0.
        return (max(cls.maximum_task_seconds, motion.nominal_seconds+
                    runtime.core.preview.climb_timeout+runtime.core.preview.hover_timeout+10.)
                if motion else cls.maximum_task_seconds) + descent

    def __init__(self,runtime,io,*,review_slots=None):
        if io.runtime is not runtime:raise ValueError('One runtime/I/O owner required')
        self.runtime,self.io=runtime,io
        self.review_slots=review_slots
        self.start=FlightStartConfirmation(runtime)
        self.started_at=None;self.done=False;self.journal_finalized=False;self.failure_snapshot=None
        self.resources=None

    def step(self,now):
        if self.done:return self.status()
        r=self.runtime
        self.started_at=r.start_requested_at
        if (self.started_at is not None and now-self.started_at>self.task_seconds(r)
                and not r.cancel_requested and not r.core.pilot_control
                and r.core.phase in ('WAIT_EXPLICIT_START','WARMUP','WAIT_OFFBOARD','WAIT_ARM','ASCEND','HOVER','DESCEND')):
            r.request_cancel(reason='flight_duration_timeout')
        self.io.step(now)
        if (not r.closed and not r.closing and (r.failure or r.stop_latched)
                and not r.core.arm_attempted):
            no_effects=self.io.journal is None or not self.io.journal.record['events']
            if no_effects or r.context().restore_ground():
                try:r.request_close()
                except (PermissionError,ValueError):pass
        if r.closed:
            if self.io.journal is not None:self.journal_finalized=self.io.journal.finalize(r,self.io)
            self.done=True
        return self.status()

    def close(self):
        # Unknown/airborne state must not tear down the localization owner.
        # Resource ownership is attached only by the explicit native factory.
        if not self.io.closed:self.io.close()
        if self.resources is not None:
            self.resources.close()
            self.resources=None

    def cancel(self):self.runtime.request_cancel()

    def install_bootstrap_review(self,path):
        if self.review_slots is None:raise PermissionError('No physical review slots installed')
        self.review_slots.install_bootstrap(path)
        self.runtime.sync_rates_enabled=True

    def install_execution_review(self,path):
        if self.review_slots is None:raise PermissionError('No physical review slots installed')
        self.review_slots.install_execution(path)

    def start_car(self):
        """Explicit car approach/alignment; TAKEOFF remains a separate action."""
        r=self.runtime;joint=r.core.cooperative
        grant=self.review_slots.execution if self.review_slots is not None else None
        # Ground-side alignment moves only the car. The aircraft's existing
        # bootstrap boundary continues to prohibit motor/mode/target commands;
        # do not require (or manufacture) a flight grant for this operation.
        if (grant is None and self.review_slots is not None and joint is not None
                and joint.sequence.ground_start):
            ground=self.review_slots.bootstrap
            if ground is not None and ground.scope=='native-rc-protected-ground-bootstrap-v1':
                grant=ground
        if (joint is None or r.now is None or r.failure or r.cancel_requested or r.stop_latched
                or r.closing or r.closed or r.coordinator.fusion.phase!='READY'
                or grant is None or not grant.start_available() or not grant._profile_ok()
                or not grant._hashes_ok() or grant.usb_present()):
            raise PermissionError('小车出发需要当前联合计划审核、定位准备和车机静止状态')
        grant.check_port_identity()
        o,missing=r.inputs.build(r.now)
        if missing:raise PermissionError('小车出发前状态缺失：'+';'.join(missing))
        joint.request_car_start(o)
        return dict(action='ground_alignment_requested' if joint.sequence.ground_start else 'loaded_car_approach_requested',
                    takeoff_point=joint.car_task.plan.route[0],resolved_start=joint.sequence.resolved_start,
                    flight_started=False)

    def require_native_start(self):
        """Native preview/confirmation prerequisite; never creates a grant.

        The existing per-message authorizer still checks hashes, expiry and
        current state at dispatch. A stale report cannot fill these slots.
        Already being in Position needs no duplicate mode request.
        """
        grant=self.review_slots.execution if self.review_slots is not None else None
        if grant is None or not grant.start_available():
            raise PermissionError('Install a current execution review before preview or TAKEOFF')
        r=self.runtime;hb=r.inputs.hb
        if (r.now is None or not hb or hb.get('custom_mode')!=3<<16
                or not 0<=r.now-hb['at']<2 or hb['armed'] is not False):
            raise PermissionError('Fresh disarmed Position heartbeat required before preview or TAKEOFF')

    def status(self):
        r=self.runtime
        missing=r.inputs.describe(r.now)['missing_conditions'] if r.now is not None else ['current_inputs_missing']
        pilot_request=r.pilot_link.mode_request if r.pilot_link else None
        pilot_attention=bool((r.core.pilot_control or pilot_request and pilot_request.get('takeover') is False) and (r.pilot_fault or pilot_request and
            pilot_request['state'] in ('REJECTED','TIMEOUT','DELIVERY_UNCERTAIN')))
        if r.pilot_source=='native_rc' and r.core.pilot_control:
            pilot_attention=not r.native_takeover_confirmed(r.now) and not r.context().restore_ground()
        failure=r.failure or r.core.fault or r.pilot_fault or (
            pilot_request.get('timeout_reason') if pilot_request else None)
        if r.core.cooperative is not None:
            failure=failure or r.core.cooperative.describe()['fault']
        if failure and self.failure_snapshot is None and r.now is not None:
            from dataclasses import asdict
            c=asdict(r.context())
            c={k:(None if isinstance(v,float) and not math.isfinite(v) else v) for k,v in c.items()}
            from rc_watch import NATIVE_COMMANDER_MAX_AGE_S
            boot=r.inputs._boot_us(r.now)
            ages={name:dict(reply_s=r.now-row['at'],
                source_s=(boot-row['value']['timestamp'])/1e6 if boot is not None else None)
                for name,row in r.inputs.critical_status.items()}
            self.failure_snapshot=dict(at=r.now,phase=r.core.phase,reason=failure,
                missing_conditions=list(missing),context=r.failure_context or c,
                status_ages=ages,limits_s=dict(pose=.15,attitude=.5,actuator=r.context().actuator_max_age(),heartbeat=2.,
                    commander_reply=NATIVE_COMMANDER_MAX_AGE_S if r.pilot_source=='native_rc' else .5),
                command_confirmation=r.command_confirmation(),
                authorization_rejection=getattr(self.io.authorizer,'last_rejection',None))
        operator_action=None
        hb=r.inputs.hb
        if (r.pilot_source=='native_rc' and r.start_requested_at is None and not r.failure
                and hb and r.now is not None and 0<=r.now-hb['at']<2 and hb['armed'] is False):
            if hb['custom_mode']==6<<16:
                operator_action='当前遗留 Offboard；请用遥控器退出该模式，定位就绪后选择 Position。程序不会自动切模或解锁。'
            elif r.core.phase=='WAIT_EXPLICIT_START' and hb['custom_mode']!=3<<16:
                operator_action='定位准备已就绪；请用遥控器选择 Position 后再生成起飞确认口令。'
        return dict(phase=r.core.phase,done=self.done,missing_conditions=missing,
            horizontal_motion=r.core.motion.describe() if r.core.motion is not None else None,
            cooperative=r.core.cooperative.describe() if r.core.cooperative is not None else None,
            sync_rates=r.sync_rates.summary() if r.sync_rates else None,
            command_confirmation=r.command_confirmation(),
            landing_confirmation=r.core.status()['landing_confirmation'],
            failure=failure,failure_snapshot=self.failure_snapshot,operator_action=operator_action,
            explicit_start_pending=r.core.phase=='WAIT_EXPLICIT_START' and r.start_requested_at is None,
            intervention_required=pilot_attention or r.core.phase=='LAND_TIMEOUT' or self.io.delivery_uncertain
                or bool(r.sync_rates and r.sync_rates.restore_errors)
                or r.core.phase=='STOP_LATCHED' and not r.context().restore_ground(),
            journal_verified_closed=self.journal_finalized,io=self.io.status(),flight_authorized=False)


def build_guarded_session(*,revision,origin,orientation,port,poll_gamepad,poll_pose,now_ros_ns,
                          check_port_identity,usb_present,origin_monotonic,recovery_path,
                          height=.3,hover_seconds=3.,fixed_profile=None,gamepad_calibration=None,pilot_source='gamepad',
                          cooperative_execution=None):
    """Wire an EXTERNALLY owned native/fake resource set. Opens no devices.

    Both review slots start empty. The caller retains all resource lifetime
    responsibility and must keep servicing the owner during operator review.
    No physical check, approval file or authorized flag is manufactured here.
    """
    from continuous_mission_runtime import ContinuousMissionRuntime
    from continuous_execution_io import ContinuousExecutionIO
    from flight_execution_journal import FlightExecutionJournal
    from flight_execution_review import ExecutionReviewSlots
    from preparation_authorization import require_no_pending_recovery
    from pathlib import Path
    if fixed_profile is not None:
        from ekf_static_profile import require_mission_profile
        require_mission_profile(fixed_profile,pilot_source=pilot_source,height=height,hover_seconds=hover_seconds)
    path=Path(recovery_path)
    require_no_pending_recovery(path.parent.parent)
    path.parent.mkdir(parents=True,exist_ok=True)
    runtime=ContinuousMissionRuntime(revision,body_origin_in_rigid=origin,
        body_orientation_in_rigid=orientation,height=height,hover_seconds=hover_seconds,
        fixed_configuration=fixed_profile is not None,fixed_profile=fixed_profile,pilot_source=pilot_source)
    if cooperative_execution is not None:
        if cooperative_execution.origin_monotonic != origin_monotonic:
            raise ValueError('car and UAV must share the owner monotonic origin')
        runtime.attach_cooperative(cooperative_execution)
    if gamepad_calibration is not None:runtime.enable_pilot(gamepad_calibration)
    journal=FlightExecutionJournal(path,boot_session=runtime.inputs.boot_session,rigid_revision=revision)
    if runtime.sync_rates is not None:
        from preparation_authorization import RateRecoveryJournal
        runtime.sync_rates.persist=RateRecoveryJournal(path.parent/'rates_recovery.json',
            scope=('fixed-point-wifi-feedback-and-timesync-rates-v1' if runtime.sync_rates.profile=='mission'
                   else 'fixed-point-wifi-timesync-rate-v1'))
    io=ContinuousExecutionIO(runtime,port,poll_gamepad=poll_gamepad,poll_pose=poll_pose,
        now_ros_ns=now_ros_ns,check_port_identity=check_port_identity,usb_present=usb_present,journal=journal,
        fixed_profile=fixed_profile)
    slots=ExecutionReviewSlots(runtime,io,origin_monotonic=origin_monotonic,
        usb_present=usb_present,check_port_identity=check_port_identity)
    io.authorizer=slots  # Empty -> denies every effect. No native flight CLI enables it.
    return FlightRunController(runtime,io,review_slots=slots)
