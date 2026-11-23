"""Default-unapproved physical review contract, no approval CLI/device access.

Hardware evidence MUST be supplied/reviewed later. A software test result is
not a substitute. This module never writes an approved record or clears holds.
"""
import datetime
import hashlib
import json
import math
from pathlib import Path
import time

from preparation_authorization import exclusive_json,load_backup,CODE_NAMES,TOOLS
from rigid_body_revision import digest,REGISTRY
from trs_uart_readonly_check import UID2
from ekf_static_profile import fingerprint

REQUIRED=('battery_only_wireless_verified','fresh_feedback_rates_verified',
    'dynamic_vicon_timing_verified',
    'current_rigid_body_and_extrinsics_reviewed','onboard_loss_action_verified',
    'protection_independent_of_pc_reviewed','protected_flight_area_ready',
    'complete_parameter_backup_verified','existing_holds_physically_reviewed',
    'first_flight_envelope_reviewed','operator_present','explicit_start_required')


def required_code_paths(pilot_source='gamepad', *, cooperative=False):
    extra=('continuous_mission_runtime.py','continuous_execution_io.py','flight_execution_journal.py',
        'flight_execution_review.py','flight_start_confirmation.py','mission_wire_transport.py',
        'fixed_mission_fusion.py','ekf_static_profile.py','vicon_fixed_point_entry.py',
        'continuous_device_host.py','preparation_host_bindings.py','gamepad_rc.py','gamepad_calib.json','px4_modes.py',
        'mavlink_link.py','rc_watch.py','ev_delay_vicon_source.py','follow_target_generator.py')
    root=TOOLS.parents[1]/'ros2_ws/src/uav_coord_bridge/uav_coord_bridge'
    names=set((*CODE_NAMES,*extra))
    if cooperative:names.update(('air_ground_execution.py','car_route_adapter.py','air_ground_resources.py'))
    if pilot_source=='native_rc':
        names-= {'gamepad_rc.py','gamepad_calib.json','gamepad_estop_monitor.py'}
    bridge_names={'frames.py','mavlink_vision.py','mission_preview.py'}
    if cooperative:bridge_names.update(('target_mapping.py','world_reference.py'))
    return ({str((TOOLS/name).resolve()) for name in names}
        |{str((root/name).resolve()) for name in bridge_names})


def flight_confirmations(profile):
    # A trial must disclose the approximation, not falsely certify tuned delay.
    if isinstance(profile,dict) and 'initial_trial' in profile:
        return tuple('approximate_layout_and_untuned_delay_accepted_for_initial_trial'
            if key=='dynamic_vicon_timing_verified' else key for key in REQUIRED)
    return REQUIRED


def flight_scope(pilot_source, profile, air_ground_config=None):
    if air_ground_config is not None:
        if pilot_source!='native_rc':raise ValueError('Cooperative flight requires native RC')
        return 'single-air-ground-native-rc-flight-review-v1'
    if isinstance(profile, dict) and profile.get('horizontal_motion') is not None:
        if pilot_source != 'native_rc':
            raise ValueError('Horizontal flight review requires native RC')
        return 'single-horizontal-native-rc-flight-review-v1'
    return ('single-vertical-native-rc-flight-review-v1' if pilot_source == 'native_rc'
            else 'single-vertical-physical-flight-review-v1')


def required_evidence_paths(record):
    registry=json.loads(REGISTRY.read_text())
    values={REGISTRY,TOOLS/'known_incidents/physical_safety_hold.json',
        TOOLS/'known_incidents/micoair743_v1143_shell_crash.json',
        Path(registry['heading_candidate_path']),Path(registry['board_candidate_path'])}
    if record.get('preflight_report'):values.add(Path(record['preflight_report']))
    if record.get('backup'):
        values|={Path(record['backup'])/'parameters_verified.json',Path(record['backup'])/'parameters_manifest.json'}
    return {str(p.resolve()) for p in values}


def draft(*,boot_session,rigid_revision,code_paths,evidence_paths,preflight_report=None,backup=None,fixed_profile=None,pilot_source='gamepad',air_ground_config=None):
    if pilot_source not in ('gamepad','native_rc'):raise ValueError('Explicit pilot source required')
    codes=required_code_paths(pilot_source,cooperative=air_ground_config is not None)|{str(Path(p).resolve()) for p in code_paths}
    registry=json.loads(REGISTRY.read_text())
    evidence={REGISTRY,TOOLS/'known_incidents/physical_safety_hold.json',
        TOOLS/'known_incidents/micoair743_v1143_shell_crash.json',
        Path(registry['heading_candidate_path']),Path(registry['board_candidate_path']),
        *map(Path,evidence_paths)}
    if preflight_report:evidence.add(Path(preflight_report))
    if backup:evidence|={Path(backup)/'parameters_verified.json',Path(backup)/'parameters_manifest.json'}
    if isinstance(fixed_profile,dict) and 'initial_trial' in fixed_profile:
        evidence|={Path(ref['path']) for ref in fixed_profile['initial_trial']['evidence'].values()}
    return dict(schema=1,scope=flight_scope(pilot_source,fixed_profile,air_ground_config),
        uid2=UID2,firmware='1.14.3',pilot_source=pilot_source,authorized=False,issued_unix=None,expires_unix=None,
        boot_session=boot_session,rigid_revision=rigid_revision,
        fixed_profile_sha256=fingerprint(fixed_profile) if fixed_profile is not None else None,
        air_ground_plan_sha256=(hashlib.sha256(json.dumps(air_ground_config,sort_keys=True,allow_nan=False).encode()).hexdigest()
                               if air_ground_config is not None else None),
        preflight_report=str(Path(preflight_report).resolve()) if preflight_report else None,
        backup=str(Path(backup).resolve()) if backup else None,
        confirmations=dict.fromkeys(flight_confirmations(fixed_profile),False),
        code_hashes={p:digest(p) for p in sorted(codes)},
        evidence_hashes={str(p.resolve()):digest(p) for p in evidence},
        warning='UNAPPROVED. Populate only after explicit physical review; software/SITL cannot approve flight.')


class ReviewedExecutionGrant:
    scope='single-vertical-physical-flight-review-v1'
    required=REQUIRED
    maximum_lifetime=180
    def __init__(self,path,runtime,*,usb_present,check_port_identity,origin_monotonic=None,
                 clock=time.monotonic,wall=time.time,execution_io=None):
        self.path=Path(path);self.record=json.loads(self.path.read_text())
        self.runtime=runtime;self.usb_present=usb_present;self.check_port_identity=check_port_identity
        self.clock,self.wall=clock,wall;self.origin=origin_monotonic
        self.original_record=json.dumps(self.record,sort_keys=True,allow_nan=False)
        self.review_hash=digest(self.path);self.consumed=False;self.fault=None
        self.start_window=None;self.last_rejection=None
        self.execution_io=execution_io
        if runtime.pilot_source=='native_rc':
            self.scope=('native-rc-protected-ground-bootstrap-v1' if self.scope=='battery-locked-ground-bootstrap-v1'
                        else flight_scope(runtime.pilot_source,runtime.coordinator.fusion.profile,
                            runtime.core.cooperative.sequence.config if runtime.core.cooperative is not None else None))
        if self.required==REQUIRED and runtime.fixed_configuration:
            self.required=flight_confirmations(runtime.coordinator.fusion.profile)

    def consume(self):
        d=self.record;r=self.runtime;now=self.wall()
        if (self.consumed or r.start_requested_at is not None or d.get('schema')!=1 or d.get('scope')!=self.scope
                or d.get('authorized') is not True or d.get('uid2')!=UID2 or d.get('firmware')!='1.14.3'
                or d.get('pilot_source','gamepad')!=r.pilot_source
                or d.get('boot_session')!=r.inputs.boot_session or d.get('rigid_revision')!=r.inputs.rigid_revision
                or type(d.get('confirmations')) is not dict
                or any(d['confirmations'].get(k) is not True for k in self.required)
                or type(d.get('code_hashes')) is not dict or not d['code_hashes']
                or not required_code_paths(r.pilot_source,cooperative=r.core.cooperative is not None)<=set(d['code_hashes'])
                or type(d.get('evidence_hashes')) is not dict or not d['evidence_hashes']
                or not required_evidence_paths(d)<=set(d['evidence_hashes'])
                or type(d.get('issued_unix')) not in (int,float) or type(d.get('expires_unix')) not in (int,float)
                or not d['issued_unix']<=now<d['expires_unix'] or not 0<d['expires_unix']-d['issued_unix']<=self.maximum_lifetime
                or type(self.origin) not in (int,float) or not math.isfinite(self.origin)
                or type(r.now) not in (int,float) or not math.isfinite(r.now)
                or not math.isfinite(self.clock()-self.origin-r.now) or self.clock()<self.origin+r.now
                or self.usb_present() or not self._profile_ok() or not self._hashes_ok()
                or not self._trial_evidence_ok() or not self._physical_evidence_ok()):
            raise PermissionError('Current explicit physical review/evidence missing; no flight grant consumed')
        self.check_port_identity()
        # Review expiry limits when a new flight may start. Preparation time
        # must not consume that flight's run budget or its LAND recovery.
        remaining=d['expires_unix']-self.wall()
        if remaining<=0:raise PermissionError('Review expired during validation; no flight grant consumed')
        self.start_window=(r.now,self.clock()-self.origin+remaining)
        exclusive_json(str(self.path)+'.consumed.json',dict(review_sha256=self.review_hash,
            boot_session=r.inputs.boot_session,rigid_revision=r.inputs.rigid_revision,consumed_unix=now))
        self.consumed=True
        return self

    def _profile_ok(self):
        r=self.runtime
        joint=r.core.cooperative
        # The existing fixed-hover/circle record cannot silently authorize a
        # car route or a platform landing. Bind the same review to this plan.
        if joint is not None:
            if (self.record.get('air_ground_plan_sha256')!=joint.plan_sha256
                    or joint.sequence.config.get('geometry_is_example') is not False):return False
        elif self.record.get('air_ground_plan_sha256') is not None:return False
        expected=self.record.get('fixed_profile_sha256')
        if not r.fixed_configuration:return expected is None
        from ekf_static_profile import fingerprint, require_mission_profile
        try:
            profile=r.coordinator.fusion.profile
            require_mission_profile(profile,pilot_source=r.pilot_source,
                height=r.core.preview.height,hover_seconds=r.core.preview.hover_seconds)
            motion = r.core.motion.spec.to_dict() if r.core.motion is not None else None
            return (motion == profile.get('horizontal_motion')
                    and expected==fingerprint(profile)==r.coordinator.fusion.profile_sha256)
        except (ValueError,TypeError,KeyError,PermissionError):return False

    def _trial_evidence_ok(self):
        """Once per review, never in the localization/target packet loop."""
        r=self.runtime
        if not r.fixed_configuration:return True
        profile=r.coordinator.fusion.profile
        if 'initial_trial' not in profile:return True
        from ekf_static_profile import inspect_initial_trial_evidence
        try:
            trial=profile['initial_trial']
            if any(self.record['evidence_hashes'].get(str(Path(ref['path']).resolve()))!=ref['sha256']
                   for ref in trial['evidence'].values()):return False
            inspect_initial_trial_evidence(profile,revision=r.inputs.rigid_revision,
                origin=trial['origin'],orientation=trial['orientation'])
            return True
        except (OSError,ValueError,TypeError,KeyError,PermissionError):return False

    def _load_backup(self):
        if self.runtime.fixed_configuration:
            from preparation_authorization import load_fixed_backup
            return load_fixed_backup(self.record['backup'],self.runtime.coordinator.fusion.profile,self.wall())
        return load_backup(self.record['backup'],self.wall())

    def _physical_evidence_ok(self):
        """Only an actual battery-static report, never the synthetic matrix.

        This is a historical prerequisite. Current FC fusion/epoch/HOME still
        come exclusively from the running reactor, not this JSON report.
        """
        try:
            d=self.record
            io=self.execution_io;r=self.runtime
            if io is not None:
                # Same-process bootstrap proof avoids a second HOME/static
                # program. Never accept a loaded report as CURRENT readiness.
                from continuous_execution_io import ContinuousExecutionIO
                o,missing=r.inputs.build(r.now)
                registry=json.loads(REGISTRY.read_text())
                joint=r.core.cooperative
                approach_ready=False
                if joint is not None and joint.car_task is not None and r.coordinator.fusion.phase=='READY':
                    try:
                        joint.check_car_start(o)
                        approach_ready=True
                    except ValueError:pass
                if (type(io) is not ContinuousExecutionIO or io.runtime is not r or io.closed
                        or io.delivery_uncertain or io.rx_bytes<=0 or io.complete_messages<=0
                        or io.journal is None or o is None or missing or not r.context().preparation_ground()
                        or not (approach_ready or r.core.phase=='WAIT_EXPLICIT_START' and
                                r.coordinator.fusion.phase=='HANDED_OFF') or r.core.arm_attempted
                        or r.start_requested_at is not None
                        or registry.get('current_runtime_revision',registry['active_revision_id'])!=r.inputs.rigid_revision or not d.get('backup')):
                    return False
                self._load_backup()
                return True
            if r.fixed_configuration:return False
            if not d.get('preflight_report') or not d.get('backup'):return False
            path=Path(d['preflight_report']);proof=json.loads(path.read_text())
            if str(path.resolve()) not in d['evidence_hashes']:return False
            at=datetime.datetime.fromisoformat(proof['captured_at_utc']).timestamp()
            registry=json.loads(REGISTRY.read_text())
            permit=Path(proof['permit_path'])
            consumed=json.loads(Path(str(permit)+'.consumed.json').read_text())
            if (proof.get('one_home_preflight_verified') is not True
                    or proof.get('parameter_restoration_verified') is not True
                    or proof.get('physical_scope')!='battery-only/props-installed-protected/disarmed/locked'
                    or proof.get('fc_usb_present_at_end') is not False
                    or proof.get('arming_output_unlock_mode_or_motion_commands_sent') is not False
                    or proof.get('global_holds_modified') is not False
                    or proof.get('identity',{}).get('uid2')!=UID2
                    or proof.get('error') or proof.get('restore_errors')
                    or not 0<=self.wall()-at<=3600
                    or registry.get('current_runtime_revision',registry['active_revision_id'])!=self.runtime.inputs.rigid_revision
                    or Path(proof['heading_path']).resolve()!=Path(registry['heading_candidate_path']).resolve()
                    or Path(proof['board_path']).resolve()!=Path(registry['board_candidate_path']).resolve()
                    or consumed.get('permit_sha256')!=digest(permit)):
                return False
            self._load_backup()
            return True
        except (OSError,ValueError,KeyError,TypeError):return False

    def _hashes_ok(self):
        try:
            return (json.dumps(self.record,sort_keys=True,allow_nan=False)==self.original_record
                and digest(self.path)==self.review_hash and all(digest(p)==h for group in
                ('code_hashes','evidence_hashes') for p,h in self.record.get(group,{}).items())
            )
        except (OSError,ValueError,TypeError,AttributeError):return False

    def start_available(self):
        return bool(self.consumed and not self.fault and self.start_window
            and self.record['issued_unix']<=self.wall()<self.record['expires_unix']
            and self.start_window[0]<=self.clock()-self.origin<self.start_window[1])

    def _reject(self,reason,*,lag=None,context=None):
        self.last_rejection=dict(reason=reason,at=self.runtime.now,phase=self.runtime.core.phase,
            elapsed_since_turn_s=lag if lag is not None and math.isfinite(lag) else None)
        if context is not None:
            self.last_rejection['ages_s']={name:(value+lag if math.isfinite(value+lag) else None)
                for name in ('heartbeat_age','actuator_age','source_age','attitude_age')
                for value in (getattr(context,name),)}
            self.last_rejection['limits_s']=dict(heartbeat_age=2.,actuator_age=context.actuator_max_age(),source_age=.15,attitude_age=.5)
        return False

    def __call__(self,runtime,record,msg):
        if runtime is not self.runtime or not self.consumed or self.fault:return self._reject('review_not_active')
        r=runtime;c=r.context();d=self.record
        restoring=msg.get_type()=='PARAM_SET' and r.closing and c.restore_ground()
        safety=(msg.get_type()=='COMMAND_LONG' and (msg.command==21 or msg.command==185 and msg.param1==2)
            or msg.get_type()=='VISION_POSITION_ESTIMATE' and r.core.phase in ('LAND','LAND_TIMEOUT','PILOT_CONTROL') or restoring)
        # Hash complete source/evidence files at review consumption and before
        # state-changing commands, not on every localization/target/RC packet.
        # A disk scan must not delay LAND or the stop path. Live identity,
        # freshness, takeover and envelope checks below remain per-message.
        check_files = not safety and msg.get_type() in ('COMMAND_LONG','PARAM_SET')
        if (self.usb_present() or (c.boot_session,c.rigid_revision)!=(d['boot_session'],d['rigid_revision'])
                or c.uid2!=UID2 or c.firmware!='1.14.3' or not c.epoch_valid
                or record['at']!=r.now or not self._profile_ok()
                or json.dumps(d,sort_keys=True,allow_nan=False)!=self.original_record
                or check_files and not self._hashes_ok()):
            return self._reject('review_identity_profile_or_candidate_changed')
        try:self.check_port_identity()
        except OSError:self.fault='port_identity_changed';return self._reject(self.fault)
        # Include hash/port checks and journal sync in actual ages. A separate
        # 50 ms loop deadline rejected still-current data and even LAND.
        lag=self.clock()-self.origin-r.now
        if not math.isfinite(lag) or lag<0:return self._reject('send_clock_invalid',lag=lag)
        started=r.start_requested_at
        active_start=bool(self.scope!='battery-locked-ground-bootstrap-v1'
            and self.scope!='native-rc-protected-ground-bootstrap-v1' and started is not None
            and self.start_window and self.start_window[0]<=started<self.start_window[1])
        pilot=r.pilot_candidate_allowed(msg)
        stationary_pose_wait=(self.scope in ('battery-locked-ground-bootstrap-v1',
            'native-rc-protected-ground-bootstrap-v1')
            and msg.get_type()=='VISION_POSITION_ESTIMATE'
            and not r.core.arm_attempted and c.preparation_ground())
        from flight_start_confirmation import FlightRunController
        if active_start:
            if not safety and not pilot and r.now+lag-started>FlightRunController.task_seconds(r):
                return self._reject('flight_duration_timeout',lag=lag,context=c)
        elif not safety and not pilot and not stationary_pose_wait and not self.start_available():
            return self._reject('start_review_expired',lag=lag,context=c)
        initial_stop=(msg.get_type()=='COMMAND_LONG' and msg.command==185 and msg.param1==2
            and not r.core.arm_attempted and not r._handed_off() and r.home_check_requested
            and r.inputs.hb and r.inputs.hb['armed'] is False and 0<=c.heartbeat_age+lag<2)
        if initial_stop:return True  # Ground lock only, never startup/arm readiness.
        if not (0<=c.actuator_age+lag<=c.actuator_max_age() and 0<=c.heartbeat_age+lag<2):
            return self._reject('send_link_state_stale',lag=lag,context=c)
        pilot_current=c.pilot_current()
        if r.pilot_source=='native_rc':
            native=r.inputs.pilot_status(r.now+lag)
            pilot_current=bool(native and native['native_input_observed']
                and native['kill_requested'] is False and native['stop_active'] is False)
        elif pilot_current:
            pilot_current=bool(r.inputs.pad_event_at is not None and 0<=r.now+lag-r.inputs.pad_event_at<=30)
        if pilot:
            # Manual recovery must not depend on Vicon remaining available.
            # Identity/epoch, USB, physical review, pad and FC state stay live.
            allowed=bool(not r.stop_latched and pilot_current and 0<=c.attitude_age+lag<=.5)
            return allowed or self._reject('manual_recovery_state_stale',lag=lag,context=c)
        if restoring:
            allowed=bool(r.inputs.land and 0<=r.now-r.inputs.land['at']+lag<=.5)
            return allowed or self._reject('restore_ground_state_stale',lag=lag,context=c)
        if safety:
            if msg.get_type()=='VISION_POSITION_ESTIMATE':
                return 0<=c.source_age+lag<=.15 or self._reject('send_pose_stale',lag=lag,context=c)
            return True
        allowed=bool(not r.failure and not r.stop_latched and pilot_current
            and 0<=c.source_age+lag<=.15 and 0<=c.attitude_age+lag<=.5)
        return allowed or self._reject('send_pose_attitude_or_pilot_invalid',lag=lag,context=c)

    def status(self):
        return dict(consumed=self.consumed,fault=self.fault,flight_authorized=False,
            last_rejection=self.last_rejection,
            scope='Current reviewed per-message boundary, not a claim of physical flight safety')


GROUND_REQUIRED=('battery_only_stationary_protected','operator_present',
    'existing_holds_physically_reviewed','complete_parameter_backup_verified',
    'no_motor_commands_approved','props_treated_as_installed','current_rigid_body_and_extrinsics_reviewed')


def draft_ground(**kwargs):
    record=draft(**kwargs)
    record.update(scope=('native-rc-protected-ground-bootstrap-v1' if record['pilot_source']=='native_rc'
                         else 'battery-locked-ground-bootstrap-v1'),confirmations=dict.fromkeys(GROUND_REQUIRED,False))
    return record


class ReviewedGroundBootstrapGrant(ReviewedExecutionGrant):
    """Reviewed protected-stationary phase, NEVER a flight-output grant.

    New distinct scope; cannot consume old static permits or widen their
    bounds. Uses the same preparation review lifetime as execution, then needs
    a separate explicit current flight review and start confirmation.
    """
    scope='battery-locked-ground-bootstrap-v1'
    required=GROUND_REQUIRED

    def _physical_evidence_ok(self):
        try:
            registry=json.loads(REGISTRY.read_text())
            if registry.get('current_runtime_revision',registry['active_revision_id'])!=self.runtime.inputs.rigid_revision:return False
            if not self.record.get('backup'):return False
            self._load_backup()
            return True
        except (OSError,ValueError,KeyError):return False

    def __call__(self,runtime,record,msg):
        if runtime.core.arm_attempted:return False
        kind=msg.get_type()
        stop=kind=='COMMAND_LONG' and msg.command==185 and msg.param1==2
        if stop and runtime.context().armed not in (None,False):return False
        if not stop:
            rate=kind=='COMMAND_LONG' and msg.command in (510,511) and runtime.sync_rate_allowed(msg)
            if kind=='RC_CHANNELS_OVERRIDE':
                if not runtime.pilot_candidate_allowed(msg) or not runtime.pilot_centered:return False
            elif kind=='COMMAND_LONG':
                if not rate and (not runtime.ground_position_allowed() or not runtime.pilot_candidate_allowed(msg)):return False
            elif kind not in ('PARAM_SET','VISION_POSITION_ESTIMATE'):return False
            c=runtime.context()
            restoring=kind=='PARAM_SET' and runtime.closing
            if not (c.restore_ground() if restoring or rate else c.preparation_ground()):return False
            if kind=='PARAM_SET':
                import trs_vision_reference_policy as vision
                pending=runtime.coordinator.fusion.pending
                if not pending:return False
                expected=vision.parameter_message(runtime.encoder,pending['name'],pending['value'])
                if msg.to_dict()!=expected.to_dict():return False
        return super().__call__(runtime,record,msg)


class ExecutionReviewSlots:
    """Two reviewed phases, same live owner/HOME proof; no automatic upgrade."""
    def __init__(self,runtime,io,*,origin_monotonic,usb_present,check_port_identity):
        if io.runtime is not runtime:raise ValueError('One live runtime/transport binding required')
        self.runtime,self.io=runtime,io
        self.options=dict(origin_monotonic=origin_monotonic,usb_present=usb_present,
            check_port_identity=check_port_identity,execution_io=io)
        self.bootstrap=self.execution=None

    def install_bootstrap(self,path):
        if self.bootstrap is not None or self.execution is not None:raise PermissionError('Review slot already consumed')
        self.bootstrap=ReviewedGroundBootstrapGrant(path,self.runtime,**self.options).consume()

    def install_execution(self,path):
        if self.bootstrap is None or self.execution is not None:raise PermissionError('One explicit upgrade after bootstrap required')
        self.execution=ReviewedExecutionGrant(path,self.runtime,**self.options).consume()

    def __call__(self,runtime,record,msg):
        grant=self.execution if self.execution is not None else self.bootstrap
        return bool(grant is not None and grant(runtime,record,msg))

    @property
    def last_rejection(self):
        grant=self.execution if self.execution is not None else self.bootstrap
        return grant.last_rejection if grant is not None else {'reason':'no_review_installed'}
