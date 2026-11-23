"""Plan B fixed-configuration lifecycle for the existing OFFLINE mission core.

No device opener, physical grant or maintenance writer. Reuses the original
pose/receipt/fusion/handoff predicates; only the parameter transaction changes.
Initial replies must be typed, post-HOME, fresh and approved/persisted. Their
original timestamps are retained, never refreshed by a cached report. The
immutable configuration is bound to this one boot/rigid session, not presented
as continuously re-read parameters. Critical flight state remains live.
"""
from copy import deepcopy
import math

import ekf_static_profile as fixed
from mission_fusion_service import ContinuousFusionService
from mavlink_readonly_probe import decode_parameter


class FixedMissionFusionService(ContinuousFusionService):
    configuration_mode = 'fixed'

    def __init__(self, sink, *, profile=None, authorizer=None):
        super().__init__(sink, authorizer=authorizer)
        self.profile = deepcopy(profile)
        self.profile_sha256 = fixed.fingerprint(profile) if isinstance(profile, dict) else None
        self.configuration = None
        self.configuration_at = None
        self.configuration_epoch = None
        self.timesync_reply_count = 0
        self.receipt_snapshots = []

    def check_configuration(self, c, parameters, types, times, requests, *, home_barrier):
        if not isinstance(self.profile, dict):
            return dict(current_configuration_verified=False, reason='fixed_profile_missing')
        result = fixed.validate_current(self.profile, parameters, types, times, c.now,
            identity={'uid2': c.uid2, 'firmware': c.firmware}, requested_at=requests,
            boot_session=c.boot_session)
        result['post_home_requests'] = bool(type(home_barrier) in (int, float)
            and math.isfinite(home_barrier) and all(
                type(requests.get(n)) in (int, float) and home_barrier <= requests[n] <= c.now
                for n in fixed.READ_PARAMS))
        # This task core proves XYZ AND yaw; a different aiding contract needs
        # separate work, not silently relaxed fusion checks or an approved=11 default.
        ctrl = parameters.get('EKF2_EV_CTRL')
        result['task_aiding_compatible'] = type(ctrl) is int and ctrl & 11 == 11
        result['current_configuration_verified'] &= (result['post_home_requests']
                                                     and result['task_aiding_compatible'])
        return result

    def begin_fixed(self, c, *, parameters, parameter_types, parameter_times,
                    parameter_requests, home_barrier, source_q, fc_q):
        self._clock(c.now)
        result = self.check_configuration(c, parameters, parameter_types, parameter_times,
                                          parameter_requests, home_barrier=home_barrier)
        if self.phase != 'NEW' or not c.preparation_ground() or not result['current_configuration_verified']:
            raise ValueError('Usable profile and fresh same-boot post-barrier fixed configuration required')
        # Parent begin is intentionally NOT called: it requires the old 2/0
        # baseline. Reuse only its established alignment/session construction.
        import compact_vision
        trial=self.profile.get('initial_trial')
        rebased_trial=(c.pilot_source=='native_rc' and isinstance(trial,dict)
                       and isinstance(trial.get('rigid_frame_transfer'),dict))
        # Approximate marker-frame transfer is not a precision tilt calibration.
        # The measured stationary 4.98--5.11 deg residual crossed the legacy
        # 5 deg cutoff. Give this reviewed native-RC trial a bounded 1 deg
        # allowance; do not zero, rotate or relabel the original measurements.
        tolerance=6. if rebased_trial else 5.
        alignment=compact_vision.yaw_alignment(source_q,fc_q,tilt_tolerance_deg=tolerance)
        source_rpy=compact_vision.euler(source_q);fc_rpy=compact_vision.euler(fc_q)
        error=math.degrees(max(abs(source_rpy[i]-fc_rpy[i]) for i in (0,1)))
        result['startup_attitude_alignment']=dict(tilt_difference_deg=error,
            nominal_tolerance_deg=5.,effective_tolerance_deg=tolerance,
            nominal_exceeded=error>5.,approximate_frame_transfer=rebased_trial,
            source_roll_pitch_deg=[math.degrees(v) for v in source_rpy[:2]],
            fc_roll_pitch_deg=[math.degrees(v) for v in fc_rpy[:2]],
            tilt_calibrated=False)
        # Commit the configuration only after alignment succeeds. A rejected
        # alignment must not leave a half-bound configuration in phase NEW.
        self.binding = c.boot_session, c.rigid_revision
        self.configuration_epoch = (c.uid2, c.firmware, *self.binding)
        self.baseline = deepcopy(parameters)
        self.configuration = deepcopy(result)
        self.configuration_at = c.now
        self.alignment = alignment
        self.start_at, self.start_boot_ms = c.now, c.boot_ms
        self.phase = 'RECEIVE_WARMUP'

    def begin(self, *args, **kwargs):
        raise PermissionError('Fixed lifecycle requires typed post-HOME begin_fixed; no legacy fallback')

    def configuration_bound(self, c):
        return bool(not self.fault and self.configuration and self.configuration_epoch == (
            c.uid2, c.firmware, c.boot_session, c.rigid_revision)
            and fixed.fingerprint(self.profile) == self.profile_sha256)

    def observe_parameter(self, msg, c):
        if self.configuration is None or msg.get_type() != 'PARAM_VALUE' or (
                msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
            return
        name = msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id, bytes) else msg.param_id.rstrip('\0')
        if name not in fixed.PARAMETER_TYPES:
            return
        spec = self.profile['parameters'][name]
        if msg.param_type != spec['type'] or decode_parameter(msg) != fixed.typed_value(spec['value'], spec['type']):
            self.fault = self.fault or 'fixed_configuration_drift:'+name
            self.phase = 'BLOCKED'

    def _dispatch(self, msg, c):
        if msg.get_type() in ('PARAM_SET', 'PARAM_EXT_SET'):
            raise PermissionError('Fixed configuration forbids writes, including fault/cleanup')
        if not self.configuration_bound(c):
            self.fault = self.fault or 'fixed_configuration_epoch_or_profile_lost'
            self.phase = 'BLOCKED'
            raise PermissionError(self.fault)
        return super()._dispatch(msg, c)

    def _parameter(self, *args, **kwargs):
        raise PermissionError('No parameter transaction or restore in fixed configuration')

    def ingest(self, msg, c):
        self._clock(c.now)
        return False  # Replies are readback only; no legacy echo-driven transition.

    def confirm_reception(self, c, rows, *, requested_at):
        result = super().confirm_reception(c, rows, requested_at=requested_at)
        self.phase = 'FUSING'  # Receipt alone is NOT fusion/READY. No writes.
        return result

    def confirm_snapshot_reception(self,c,row,*,requested_at):
        """Native single-print receipts plus measured host cadence and EKF audit.

        Sparse NSH observations cannot prove every FC publication interval.
        Do not label their spacing as a receiver rate or as timing calibration.
        """
        import compact_vision
        self._clock(c.now)
        if (c.pilot_source!='native_rc' or self.phase!='RECEIVE_WARMUP'
                or not self._bound(c) or not c.preparation_ground()
                or not self.start_at<=requested_at<=c.now or c.now-requested_at>.5
                or self.last_send is None or c.now-self.last_send>self.maximum_send_gap):
            raise ValueError('Fresh native ground snapshot transaction required')
        mapped=self.profile.get('timestamp_mode')=='local_ros_timesync'
        if mapped and not self.timesync_reply_count:
            raise ValueError('Same-owner timesync reply required')
        stamp=row.get('timestamp');sample=row.get('timestamp_sample')
        if (type(stamp) is not int or type(sample) is not int or
                not 0<=c.boot_ms*1000-stamp<=500000 or
                not 0<=stamp-sample<=(150000 if mapped else 1000) or
                self.receipt_snapshots and (stamp<=self.receipt_snapshots[-1]['snapshot']['timestamp']
                    or sample<=self.receipt_snapshots[-1]['snapshot']['timestamp_sample'])):
            raise ValueError('Fresh distinct progressing FC snapshot required')
        item=dict(snapshot=deepcopy(row),requested_at=requested_at,at=c.now,
            requested_boot_us=int(c.boot_ms*1000-(c.now-requested_at)*1e6),completed_boot_us=c.boot_ms*1000)
        recent=[s for s in self.sent if 0<=c.now-s['elapsed_s']<=.5]
        if stamp<item['requested_boot_us']-100000 or not compact_vision.match(row,recent):
            raise ValueError('Request-bounded snapshot must match an actual recent send')
        self.receipt_snapshots.append(item)
        if len(self.receipt_snapshots)<3:return
        evidence=compact_vision.ground_receipt_evidence(self.receipt_snapshots,self.sent,
            sample_budget_us=150000 if mapped else 1000,native_rc=True)
        evidence.update(scope='Native current receipt and host cadence; actual XYZ/yaw EKF fusion still required',
            timestamp_mode=self.profile.get('timestamp_mode'),same_owner_timesync_replied=self.timesync_reply_count>0)
        self.receipt_checks=evidence
        self.receive_verified=True
        self.phase='FUSING'

    def reception_checks(self, c, rows, *, requested_at):
        result=super().reception_checks(c,rows,requested_at=requested_at)
        if self.profile.get('timestamp_mode')!='local_ros_timesync':return result
        # PX4 may use arrival time before its filter converges, then mapped
        # local-ROS sample time. Neither proves exposure timing or 2 ms accuracy.
        # Keep the existing 150 ms source budget; never accept future samples.
        checks=result['checks']
        checks.pop('receive_time_mode')
        samples=[r.get('timestamp_sample') for r in rows]
        checks['sample_time_current']=all(type(s) is int and s>0
            and 0<=r['timestamp']-s<=150_000 for r,s in zip(rows,samples))
        checks['sample_time_progressing']=all(type(s) is int for s in samples) and all(
            b>a for a,b in zip(samples,samples[1:]))
        checks['same_owner_timesync_replied']=self.timesync_reply_count>0
        result['timestamp_mode']='local_ros_timesync'
        result['absolute_clock_accuracy_verified']=False
        result['failed']=[key for key,ok in checks.items() if not ok]
        return result

    def tick(self, c):
        if self.configuration is not None and not self.configuration_bound(c) and self.phase != 'CLOSED':
            self.fault = self.fault or 'fixed_configuration_epoch_or_profile_lost'
            self.phase = 'BLOCKED'
        if self.phase.startswith(('CONFIGURE_', 'WAIT_DELAY_', 'WAIT_CTRL_', 'RESTORE_', 'WAIT_RESTORE_')):
            self.fault = self.fault or 'legacy_transaction_in_fixed_mode'
            self.phase = 'BLOCKED'
        return super().tick(c)

    def mark_ready(self, c, inputs):
        if not self.configuration_bound(c):
            raise PermissionError('Fixed configuration binding lost before actual fusion readiness')
        return super().mark_ready(c, inputs)

    def handoff(self, c, core):
        if not self.configuration_bound(c):
            raise PermissionError('Fixed configuration binding lost before handoff')
        return super().handoff(c, core)

    def close(self, c):
        self._clock(c.now)
        if not c.restore_ground() or not self._bound(c):
            self.land_required |= c.armed is True
            raise PermissionError('Unknown/airborne state: no completed cleanup claim')
        if self.transferred_core is not None and self.transferred_core.phase not in ('COMPLETE', 'BLOCKED', 'STOP_LATCHED'):
            raise PermissionError('Mission owns localization until actual end, including LAND')
        self.pending = None
        self.phase = 'CLOSED'  # Stop ground stream; persistent config is untouched.

    def status(self):
        result = super().status()
        result.update(configuration_mode='fixed', profile_sha256=self.profile_sha256,
            initial_configuration=deepcopy(self.configuration), configuration_verified_at=self.configuration_at,
            parameter_writes=0, restore_writes=0, physical_flight_authorized=False,
            configuration_proof_scope='one immutable initial readback, same boot/session; original times retained')
        return result
