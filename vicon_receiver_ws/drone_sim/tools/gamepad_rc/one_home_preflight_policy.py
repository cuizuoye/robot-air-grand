"""Separate battery-only one-HOME static acceptance. Never flight permission."""
from copy import deepcopy
from pathlib import Path
import json
import time

from preparation_authorization import (EvidenceSeal, RecoveryJournal, evidence_paths,
    exclusive_json, load_backup, require_no_pending_recovery)
from rigid_body_revision import digest
from trs_uart_readonly_check import UID2
from trs_only_estop_bench import state_valid

SCOPE='battery-only-one-home-static-preflight-no-flight-v1'
RECEIVE_SCOPE='battery-only-one-home-receive-rate-diagnostic-no-fusion-v1'
COMPACT_SCOPE='battery-only-compact-ground-fusion-no-flight-v1'
SHORT_SCOPE='battery-only-short-status-ground-fusion-no-flight-v1'
BINARY_SCOPE='battery-only-binary-diagnostics-ground-fusion-no-flight-v1'
LOW_BANDWIDTH_SCOPE='battery-only-low-bandwidth-binary-ground-fusion-no-flight-v1'
FROZEN_TARGET_SCOPE='battery-only-frozen-fc-target-stream-no-mode-no-flight-v1'
FIXED_SCOPE='fixed-ekf-ground-offline-candidate-v1'
FIXED_LIVE_SCOPE='fixed-ekf-props-off-wireless-ground-no-flight-v1'


def allowed(msg,**context):
    import trs_powered_fusion_policy as base
    diagnostic_only=context.pop('receive_rate_only',False)
    low_bandwidth=context.pop('low_bandwidth_vision',False)
    noise_parameters=context.pop('vision_noise_parameters',None)
    fixed_config=context.get('fixed_config') is True
    if fixed_config and msg.get_type() in ('PARAM_SET','PARAM_EXT_SET'):
        return False  # Before any low-bandwidth/legacy policy exception.
    if diagnostic_only and msg.get_type()=='PARAM_SET':return False
    if low_bandwidth and msg.get_type()=='VISION_POSITION_ESTIMATE':
        import low_bandwidth_vision as low
        return bool(context.get('authorized') is True and context.get('identity_fresh') is True
            and context.get('locked_static') is True and low.pose_allowed(msg,noise_parameters or {}))
    extras=(b'listener vehicle_visual_odometry -n 13\n',
            b'uorb top -1 vehicle_visual_odometry\n')
    if (msg.get_type()=='SERIAL_CONTROL' and context.get('identity_fresh') is True
            and (msg.device,msg.flags,msg.timeout,msg.baudrate)==(10,2,0,0)
            and any(msg.count==len(extra) and bytes(msg.data)==extra.ljust(70,b'\0') for extra in extras)):
        return True
    return base.allowed(msg,**context)


class StartupHome:
    """One current short press; any later press aborts, never resumes."""
    def __init__(self):
        self.press=self.release=self.ack_at=None
        self.ack_boot_us=None
        self.short_barrier_ns=None
        self.verified=False;self.aborted=False;self.events=[]

    @property
    def needs_stop(self):return self.press is not None and self.ack_at is None

    def event(self,value,now):
        self.events.append(dict(value=value,at=now))
        if value==1:
            if self.press is None:self.press=now
            else:self.aborted=True
        elif value==0 and self.press is not None and self.release is None:
            self.release=now
            if not 0<=now-self.press<=1.:self.aborted=True
        elif value==2 and self.press is not None and now-self.press>1.:self.aborted=True

    def ack(self,now,*,boot_us=None):
        if self.press is not None and now>=self.press:
            self.ack_at=now;self.ack_boot_us=boot_us

    def query_due(self,now,last_query_at):
        """No NSH traffic while waiting for human input or lock settling.

        ACK is not proof: the first permitted query must still provide a
        complete, fresh, post-ACK/post-release actuator snapshot.
        """
        return bool(not self.aborted and not self.verified and self.release is not None
            and self.ack_at is not None and now>=max(self.release,self.ack_at)+1.5
            and now-last_query_at>.3)

    def observe(self,state,*,requested_at,now,boot_ms,boot_us=None):
        current_boot_us=boot_ms*1000 if boot_ms is not None and boot_us is None else boot_us
        if (not self.aborted and self.ack_at is not None and self.release is not None
                and type(self.ack_boot_us) is int and state['timestamp_us']>self.ack_boot_us
                and requested_at>=max(self.ack_at,self.release)+1.5
                and requested_at<=now and state_valid(state,boot_ms)
                and type(current_boot_us) is int and current_boot_us//1000==boot_ms
                and state['timestamp_us']<=current_boot_us and state['lockdown']):
            self.verified=True
        return self.verified and not self.aborted

    def short_bootstrap_ready(self):
        # Permission to configure telemetry ONLY, never proof of output lock.
        return bool(not self.aborted and self.press is not None
                    and self.release is not None and self.ack_at is not None)

    def observe_short(self,lock,*,now_ns,post_home_ns,identity_verified):
        if not self.short_bootstrap_ready() or not identity_verified:return False
        ref=lock.clock.reference
        if (self.short_barrier_ns is None and ref is not None
                and ref['received_ns']-ref['rtt_ns']>=post_home_ns):
            # Remote time of a matched request sent AFTER ACK and release.
            # An old locked snapshot, even freshly received, cannot pass this.
            self.short_barrier_ns=ref['remote_ns']
        detail=lock.clock.assess(lock.status.sample,now_ns,
            identity_verified=identity_verified,state_fault=lock.status.fault)
        if (self.short_barrier_ns is not None and detail['valid']
                and lock.status.sample['stamp']*1_000_000>self.short_barrier_ns
                and int(lock.status.sample['received']*1e9)>=post_home_ns):
            self.verified=True
        return self.verified and not self.aborted

    def status(self):
        return dict(press_at=self.press,release_at=self.release,ack_at=self.ack_at,ack_boot_us=self.ack_boot_us,
            short_post_home_remote_barrier_ns=self.short_barrier_ns,
            verified=self.verified,aborted=self.aborted,events=deepcopy(self.events),
            flight_authorized=False)


def paths_for(heading,board,proof,backup,usb_proof,wireless_proof=None):
    paths=evidence_paths(heading,board,proof,backup)
    tools=Path(__file__).resolve().parent
    for name in ('one_home_preflight_policy.py','drone_preflight.py','trs_powered_fusion_bench.py',
                 'trs_powered_fusion_policy.py','trs_static_fusion_bench.py','trs_only_estop_bench.py',
                 'vicon_fc_attitude_check.py','usb_fusion_policy.py','usb_vision_policy.py',
                 'usb_yaw_fusion_policy.py','usb_height_fusion_policy.py','control_readonly_policy.py',
                 'short_status_transport.py','static_lock_gate.py','lock_clock_probe.py','output_lock_status.py',
                 'fusion_binary_record.py','fusion_binary_protocol.py','fusion_binary_receiver.py','parameter_snapshot.py',
                 'low_bandwidth_vision.py','frozen_target_stream.py','disarmed_offboard_policy.py',
                 'vision_reference_xyz_policy.py','trs_static_fusion_policy.py','trs_transport_config.py'):
        paths['one_home/'+name]=tools/name
    paths['usb_ground_pass']=Path(usb_proof)
    if wireless_proof is not None:
        from frozen_target_stream import prior_paths
        paths.update(prior_paths(wireless_proof))
    return paths


def draft(run_dir,heading,board,proof,backup,usb_proof,*,baud=115200,receive_rate_only=False,compact_ground_fusion=False,short_status=False,binary_diagnostics=False,low_bandwidth_vision=False,frozen_target_stream=False,wireless_proof=None,fixed_ekf_profile=None):
    if baud!=115200:raise ValueError('Pinned current TRS 115200 baud required')
    if receive_rate_only and compact_ground_fusion:raise ValueError('Distinct diagnostic scopes cannot overlap')
    if short_status and not compact_ground_fusion:raise ValueError('Short status requires compact static scope')
    if binary_diagnostics and not short_status:raise ValueError('Binary diagnostics requires short status')
    if low_bandwidth_vision and not binary_diagnostics:raise ValueError('Low-bandwidth vision requires binary ground scope')
    if frozen_target_stream:
        if not low_bandwidth_vision or wireless_proof is None:
            raise ValueError('Frozen target requires low-bandwidth profile and prior wireless proof')
        from frozen_target_stream import require_prior
        require_prior(wireless_proof,heading,board)
    elif wireless_proof is not None:raise ValueError('Wireless target proof belongs to distinct frozen-target scope')
    if fixed_ekf_profile is not None:
        if not (compact_ground_fusion and short_status and binary_diagnostics) or receive_rate_only or frozen_target_stream:
            raise ValueError('Fixed candidate is locked ground fusion only, no targets or legacy fallback')
        from ekf_static_profile import load_json, fingerprint
        profile=load_json(fixed_ekf_profile)
        return dict(schema=1,scope=FIXED_SCOPE,fixed_ekf_profile=str(Path(fixed_ekf_profile).resolve()),
            profile_sha256=fingerprint(profile),run_dir=str(Path(run_dir).resolve()),
            user_authorized=False,physical_confirmed=False,live_enabled=False,
            flight_authorized=False,control_ready=False,allowed_parameter_writes=[],
            note='Offline candidate only. No live permit consumer is implemented or enabled.')
    run_dir=Path(run_dir).resolve()
    return dict(schema=1,scope=FROZEN_TARGET_SCOPE if frozen_target_stream else LOW_BANDWIDTH_SCOPE if low_bandwidth_vision else BINARY_SCOPE if binary_diagnostics else SHORT_SCOPE if short_status else COMPACT_SCOPE if compact_ground_fusion else RECEIVE_SCOPE if receive_rate_only else SCOPE,
        frozen_target_stream=frozen_target_stream is True,
        wireless_proof=str(Path(wireless_proof).resolve()) if wireless_proof is not None else None,
        low_bandwidth_vision=low_bandwidth_vision is True,
        binary_diagnostics=binary_diagnostics is True,
        short_status=short_status is True,
        compact_ground_fusion=compact_ground_fusion is True,
        receive_rate_only=receive_rate_only is True,user_authorized=False,issued_unix=None,expires_unix=None,
        uid2=UID2,firmware='1.14.3',baud=baud,run_dir=str(run_dir),
        heading=str(Path(heading).resolve()),board=str(Path(board).resolve()),
        proof=str(Path(proof).resolve()),backup=str(Path(backup).resolve()),
        usb_proof=str(Path(usb_proof).resolve()),physical_confirmed=False,
        hashes={k:digest(p) for k,p in paths_for(heading,board,proof,backup,usb_proof,wireless_proof).items()},
        limits=dict(stream_seconds=20,max_packets=640,fusion_seconds=0 if receive_rate_only else 12),
        flight_authorized=False,global_holds_modified=False)


def consume(path,*,backup,output,heading,board,baud):
    from trs_runtime_monitor import inspect_evidence
    record=json.loads(Path(path).read_text());now=time.time()
    if record.get('scope')==FIXED_SCOPE or record.get('fixed_ekf_profile') is not None:
        raise PermissionError('Fixed EKF plan B is offline-only pending separate props-off bench review; no legacy fallback')
    expected=draft(Path(output).parent,heading,board,record['proof'],backup,record['usb_proof'],baud=baud,
        receive_rate_only=record.get('receive_rate_only') is True,
        compact_ground_fusion=record.get('compact_ground_fusion') is True,short_status=record.get('short_status') is True,
        binary_diagnostics=record.get('binary_diagnostics') is True,
        low_bandwidth_vision=record.get('low_bandwidth_vision') is True,
        frozen_target_stream=record.get('frozen_target_stream') is True,wireless_proof=record.get('wireless_proof'))
    for key in ('schema','scope','frozen_target_stream','wireless_proof','short_status','binary_diagnostics','low_bandwidth_vision','receive_rate_only','compact_ground_fusion','uid2','firmware','baud','run_dir','heading','board','proof','backup',
                'usb_proof','hashes','limits','flight_authorized','global_holds_modified'):
        if record.get(key)!=expected[key]:raise ValueError('One-HOME scope/code mismatch: '+key)
    if (record.get('user_authorized') is not True or record.get('physical_confirmed') is not True
            or type(record.get('issued_unix')) not in (int,float)
            or type(record.get('expires_unix')) not in (int,float)
            or not record['issued_unix']<=now<record['expires_unix']
            or not 0<record['expires_unix']-record['issued_unix']<=600
            or Path(output).resolve()!=Path(record['run_dir'])/'report.json'):
        raise ValueError('Explicit current battery-only static scope required')
    require_no_pending_recovery(Path(output).parent.parent)
    inspect_evidence(heading,board,record['proof']);load_backup(backup,now)
    passed=json.loads(Path(record['usb_proof']).read_text())
    if (passed.get('ground_fusion_verified') is not True
            or passed.get('runtime',{}).get('failure') is not None
            or passed.get('parameter_restoration_verified') is not True
            or passed.get('telemetry_restoration_verified') is not True
            or passed.get('runtime',{}).get('mission_inputs',{}).get('rigid_revision')
                !=json.loads(Path(record['proof']).read_text())['rigid_body_revision_id']):
        raise ValueError('Same-revision completed USB static acceptance required')
    paths=paths_for(heading,board,record['proof'],backup,record['usb_proof'],record.get('wireless_proof'))
    if (json.loads(paths['incident'].read_text()).get('block_active_benches') is not True
            or json.loads(paths['physical_hold'].read_text()).get('block_props_off_benches') is not True):
        raise ValueError('Existing global holds must remain intact')
    seal=EvidenceSeal(paths,record['hashes'])
    if not seal.refresh():raise ValueError(seal.fault)
    exclusive_json(str(path)+'.consumed.json',dict(scope=record['scope'],consumed_unix=now,
        permit_sha256=digest(path),flight_authorized=False))
    return {**record,'_seal':seal}


class PoweredRecoveryJournal(RecoveryJournal):
    def __init__(self,path,*,scope=SCOPE):
        if scope not in (SCOPE,RECEIVE_SCOPE,COMPACT_SCOPE,SHORT_SCOPE,BINARY_SCOPE,LOW_BANDWIDTH_SCOPE,FROZEN_TARGET_SCOPE,FIXED_SCOPE,FIXED_LIVE_SCOPE):raise ValueError('Unknown ground recovery scope')
        super().__init__(path);self.record['scope']=scope;self._save()

    def finish(self,changed,restored,final_arm):
        safe=not changed or (all(restored.get(name) is True for name in changed)
            and final_arm and final_arm['armed'] is False and final_arm['lockdown'] is True)
        self.record.update(verified_closed=bool(safe),needs_restore=not bool(safe))
        self._save();return bool(safe)

    def finish_fixed(self, writes, final_arm):
        """No fabricated rollback receipt; unknown close remains review-blocked."""
        safe=bool(not writes and final_arm and final_arm.get('armed') is False
            and final_arm.get('lockdown') is True and final_arm.get('manual_lockdown') is False
            and final_arm.get('force_failsafe') is False)
        self.record.update(verified_closed=safe,needs_restore=not safe,
            parameter_restore_required=bool(writes),needs_operator_review=not safe,
            fixed_ekf_parameter_writes=len(writes),
            closure_scope='Zero-write ground close; not parameter restoration or flight authorization')
        self._save();return safe
