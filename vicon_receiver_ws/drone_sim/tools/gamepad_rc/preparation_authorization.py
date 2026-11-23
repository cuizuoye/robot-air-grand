"""One-use USB-ground preparation scope and durable recovery; never flight.

Drafts are deliberately NOT authorized. Consuming a separately reviewed
permit preserves all existing incident/physical holds and binds current code,
candidate hashes, prior static evidence and a fresh full parameter backup.
No device, firmware, network, arm, unlock or permit-approval operation here.
"""
from copy import deepcopy
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import struct
import threading
import time

from mission_fusion_service import context_from_inputs
from rigid_body_revision import digest, REGISTRY
from trs_uart_readonly_check import UID2
import trs_vision_reference_policy as vision
from trs_message_interval_trial import CANDIDATES

TOOLS=Path(__file__).resolve().parent
ROOT=TOOLS.parents[1]
SCOPE='usb-only-locked-ground-continuous-preparation-no-flight-v1'
GROUND_FUSION_SCOPE='usb-only-locked-ground-fusion-acceptance-no-handoff-v2'
CODE_NAMES=('preparation_authorization.py','prepare_drone.py','preparation_host_bindings.py',
    'mission_preparation_runtime.py','mission_preparation_io.py','mission_preparation_coordinator.py',
    'mission_audit_collector.py','mission_live_inputs.py','mission_fusion_service.py',
    'compact_vision.py','trs_runtime_monitor.py','rigid_body_revision.py','mavlink_crc_stream.py')
CODE_NAMES+=('flight_mission_core.py','gamepad_estop_monitor.py','mavlink_readonly_probe.py',
    'trs_vision_reference_policy.py','usb_combined_fusion_policy.py','diagnostic_transaction.py',
    'persistent_shell.py','actuator_query.py','estop_only_policy.py','ground_estimator_settlement.py',
    'mission_telemetry_rates.py','trs_message_interval_trial.py')


def sync_directory(path):
    fd=os.open(path,os.O_RDONLY|os.O_DIRECTORY)
    try:os.fsync(fd)
    finally:os.close(fd)


def exclusive_json(path,record):
    with Path(path).open('x') as stream:
        json.dump(record,stream,indent=2,ensure_ascii=False,allow_nan=False)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    sync_directory(Path(path).parent)


def evidence_paths(heading,board,proof,backup,registry=REGISTRY):
    return {**{'code/'+name:TOOLS/name for name in CODE_NAMES},
        'heading':Path(heading),'board':Path(board),'static_proof':Path(proof),
        'registry':Path(registry),'backup':Path(backup)/'parameters_verified.json',
        'ros_frames':ROOT/'ros2_ws/src/uav_coord_bridge/uav_coord_bridge/frames.py',
        'ros_pose_gate':ROOT/'ros2_ws/src/uav_coord_bridge/uav_coord_bridge/mavlink_vision.py',
        'backup_manifest':Path(backup)/'parameters_manifest.json',
        'incident':TOOLS/'known_incidents/micoair743_v1143_shell_crash.json',
        'physical_hold':TOOLS/'known_incidents/physical_safety_hold.json'}


def draft(run_dir,heading,board,proof,backup,*,ground_fusion_only=False,usb_observer=False):
    if usb_observer and not ground_fusion_only:raise ValueError('USB observer needs distinct ground fusion scope')
    paths=evidence_paths(heading,board,proof,backup)
    return dict(schema=1,scope=GROUND_FUSION_SCOPE if ground_fusion_only else SCOPE,uid2=UID2,firmware='1.14.3',
        ground_fusion_only=ground_fusion_only is True,
        ground_status_max_age_s=1. if ground_fusion_only else .5,
        diagnostic_route='usb-ground-observer' if usb_observer else 'trs',
        user_authorized=False,issued_unix=None,expires_unix=None,
        run_dir=str(Path(run_dir).resolve()),backup=str(Path(backup).resolve()),
        heading=str(Path(heading).resolve()),board=str(Path(board).resolve()),
        static_proof=str(Path(proof).resolve()),
        physical_confirmations=dict(battery_disconnected=False,usb_only=False,stationary=False,props_treated_as_installed=False),
        allowed_parameters={'EKF2_EV_CTRL':[2,11],'EKF2_EV_DELAY':[0,65]},
        temporary_telemetry_intervals_us={str(k):v for k,v in CANDIDATES.items()},
        hashes={key:digest(path) for key,path in paths.items()},
        note='UNAUTHORIZED draft. Requires separate current physical review, fresh full backup, and one-use consumption. No flight permission.')


def load_backup(directory,now_unix):
    path=Path(directory)/'parameters_verified.json'
    record=json.loads(path.read_text());manifest=json.loads((path.parent/'parameters_manifest.json').read_text())
    rows=record.get('records',[])
    if (record.get('complete') is not True or record.get('expected_count')!=926
            or record.get('received_count')!=926 or len(rows)!=926
            or record.get('firmware',{}).get('version')!='1.14.3'
            or record.get('system_id')!=1 or record.get('component_id')!=1
            or not 0<=now_unix-path.stat().st_mtime<=3600
            or manifest.get('sha256',{}).get(path.name)!=digest(path)
            or any(type(row.get('index')) is not int for row in rows)
            or {row.get('index') for row in rows}!=set(range(926))
            or len({row.get('name') for row in rows})!=926):
        raise ValueError('Fresh complete 926-parameter backup and matching manifest required')
    by_name={row['name']:row for row in rows};values={name:row['value'] for name,row in by_name.items()}
    if set(vision.READ_PARAMS)-set(values) or not vision.scope_ok(values,values) or (
            values.get('EKF2_EV_CTRL')!=2 or values.get('EKF2_EV_DELAY')!=0):
        raise ValueError('Expected Vision-reference maintenance baseline required')
    for name in vision.READ_PARAMS:
        row=by_name[name];raw=bytes.fromhex(row['wire_value_hex'])
        if len(raw)!=4 or type(row['type']) is not int or row['type'] not in (6,9):raise ValueError('Typed wire backup required')
        value=struct.unpack('<i' if row['type']==6 else '<f',raw)[0]
        if not math.isfinite(value) or value!=row['value']:raise ValueError('Backup wire/value mismatch')
    if by_name['EKF2_EV_CTRL']['type']!=6 or by_name['EKF2_EV_DELAY']['type']!=9:
        raise ValueError('EV control/delay types mismatch')
    return {key:values[key] for key in vision.READ_PARAMS}


def load_fixed_backup(directory, profile, now_unix):
    """Independent plan B baseline; NEVER reinterpret an old dynamic permit.

    This checks an archived backup, not current FC freshness, not Flash-save
    completion and not a physical grant. Archive age is not current state:
    FixedMissionFusionService separately requests the task parameters from
    the identified current boot. Existing maintenance load_backup stays unchanged.
    now_unix is retained for call compatibility; file mtime is not evidence.
    """
    from ekf_static_profile import load_json, records_by_name, compare, READ_PARAMS
    path=Path(directory)/'parameters_verified.json'
    record=load_json(path);manifest=load_json(path.parent/'parameters_manifest.json')
    rows=record.get('records',[])
    if not isinstance(rows,list) or any(not isinstance(row,dict) for row in rows):
        raise ValueError('Typed parameter record list required')
    count=record.get('expected_count')
    if (type(count) is not int or not len(READ_PARAMS)<=count<=65535
            or record.get('complete') is not True
            or type(record.get('received_count')) is not int
            or record['received_count']!=count or len(rows)!=count
            or record.get('missing_indices')
            or record.get('firmware',{}).get('version')!='1.14.3'
            or record.get('system_id')!=1 or record.get('component_id')!=1
            or manifest.get('sha256',{}).get(path.name)!=digest(path)
            or any(type(row.get('index')) is not int for row in rows)
            or {row.get('index') for row in rows}!=set(range(count))
            or any(type(row.get('param_count')) is not int or row['param_count']!=count for row in rows)
            or any(not isinstance(row.get('name'),str) or not row['name'] for row in rows)
            or len({row['name'] for row in rows})!=count):
        raise ValueError('Complete typed fixed-profile archive with consistent FC counts and manifest required')
    for row in rows:
        try:
            raw=bytes.fromhex(row['wire_value_hex'])
            kind=row['type'];value=row['value']
            if (len(raw)!=4 or type(kind) is not int or kind not in (6,9)
                    or type(value) not in (int,float) or not math.isfinite(value)
                    or struct.unpack('<i' if kind==6 else '<f',raw)[0]!=value):
                raise ValueError('Invalid typed wire value')
        except (KeyError,TypeError,ValueError,OverflowError) as exc:
            raise ValueError('Backup typed wire/value mismatch: '+row['name']) from exc
    records=records_by_name(record);result=compare(profile,records)
    if not result['configuration_usable']:
        raise ValueError('Fixed profile lacks an approved/scoped-trial contract or backup mismatches: '+json.dumps(result))
    return {name:records[name]['value'] for name in READ_PARAMS}


class EvidenceSeal:
    """File checks run off the serial loop; callbacks only inspect a fresh seal."""
    def __init__(self,paths,hashes,*,clock=time.monotonic):
        self.paths=paths;self.hashes=hashes;self.clock=clock
        self.checked_at=None;self.fault=None;self.stop=threading.Event();self.worker=None

    def refresh(self):
        if self.fault:return False
        try:
            if set(self.paths)!=set(self.hashes) or any(digest(p)!=self.hashes[k] for k,p in self.paths.items()):
                raise ValueError('Code/evidence changed during ground preparation')
            self.checked_at=self.clock()
            return True
        except (ValueError,OSError) as exc:
            self.fault=str(exc);return False

    def fresh(self):
        return bool(not self.fault and self.checked_at is not None and 0<=self.clock()-self.checked_at<=1.)

    def start(self):
        if self.worker is not None:raise RuntimeError('One evidence watcher only')
        def work():
            while not self.stop.wait(.25):self.refresh()
        self.worker=threading.Thread(target=work,daemon=True);self.worker.start()

    def close(self):
        self.stop.set()
        if self.worker:self.worker.join(timeout=.2)


class GroundPreparationGrant:
    """Installed only after validated, durably consumed, explicit one-use permit."""
    def __init__(self,runtime,record,baseline,seal,*,wall_clock=time.time,clock=time.monotonic,origin=0.):
        self.runtime=runtime;self.record=record;self.baseline=baseline;self.seal=seal
        self.wall_clock,self.clock,self.origin=wall_clock,clock,origin
        self.boot=runtime.inputs.boot_session;self.revision=runtime.inputs.rigid_revision

    def __call__(self,runtime,frame,msg):
        if runtime is not self.runtime or runtime.now is None:return False
        c=runtime.context()
        if (c.ground_status_max_age!=self.record['ground_status_max_age_s']
                or runtime.ground_fusion_only is not self.record['ground_fusion_only']
                or ('usb-ground-observer' if runtime.observer_required else 'trs')!=self.record['diagnostic_route']
                or not runtime.observer_ok()):return False
        lag=self.clock()-self.origin-runtime.now
        if not 0<=lag<=.05:return False  # Also catches slow journal flush before write.
        c=replace(c,actuator_age=c.actuator_age+lag,source_age=c.source_age+lag,
                  heartbeat_age=c.heartbeat_age+lag,attitude_age=c.attitude_age+lag)
        s=runtime.coordinator.fusion
        rate_command=msg.get_type()=='COMMAND_LONG' and msg.command in (510,511)
        if rate_command:
            rates=runtime.rates
            restoring=bool(rates and rates.restoring)
            deadline=self.record['expires_unix']+(60 if restoring else 0)
            return bool(rates and self.record['issued_unix']<=self.wall_clock()<deadline
                and self.seal.fresh() and (c.boot_session,c.rigid_revision)==(self.boot,self.revision)
                and frame['at']==runtime.now and c.uid2==UID2 and c.firmware=='1.14.3'
                and runtime.inputs.home_proven and c.epoch_valid and runtime._identity_ok()
                and (msg.command==510 or c.armed is False and c.actuator_age<=.5
                     and c.lockdown is True)
                and (runtime.closing if restoring else not runtime.failure and not runtime.stop_latched)
                and rates.permits(msg,fresh=runtime._identity_ok(),
                    locked=c.armed is False and c.actuator_age<=.5 and c.lockdown is True))
        restoring=msg.get_type()=='PARAM_SET' and s.phase in ('WAIT_RESTORE_CTRL','WAIT_RESTORE_DELAY')
        deadline=self.record['expires_unix']+(60 if restoring else 0)
        if (not self.record['issued_unix']<=self.wall_clock()<deadline or not self.seal.fresh()
                or (c.boot_session,c.rigid_revision)!=(self.boot,self.revision)
                or frame['at']!=runtime.now or c.uid2!=UID2 or c.firmware!='1.14.3'
                or not runtime.inputs.land or runtime.now-runtime.inputs.land['at']+lag>c.ground_status_max_age
                or not runtime.inputs.home_proven or not runtime.prepare_requested
                or s.baseline is None or any(s.baseline.get(k)!=v for k,v in self.baseline.items())):
            return False
        if restoring:return runtime.closing and c.restore_ground()
        return bool(msg.get_type() in ('VISION_POSITION_ESTIMATE','PARAM_SET')
                    and not runtime.failure and not runtime.stop_latched and c.locked_ground())


def consume(path,runtime,run_dir,heading,board,proof,backup,*,now_unix=None,clock=time.monotonic,origin=0.):
    from trs_runtime_monitor import inspect_evidence
    now=time.time() if now_unix is None else now_unix
    path=Path(path);record=json.loads(path.read_text())
    expected=draft(run_dir,heading,board,proof,backup,ground_fusion_only=runtime.ground_fusion_only,
        usb_observer=runtime.observer_required)
    for name in ('schema','scope','ground_fusion_only','ground_status_max_age_s','diagnostic_route','uid2','firmware','run_dir','backup','heading','board','static_proof','allowed_parameters','temporary_telemetry_intervals_us','hashes'):
        if record.get(name)!=expected[name]:raise ValueError('Wrong preparation scope/evidence: '+name)
    issued,expires=record.get('issued_unix'),record.get('expires_unix')
    physical=record.get('physical_confirmations',{})
    if (record.get('user_authorized') is not True
            or type(issued) not in (int,float) or type(expires) not in (int,float)
            or not issued<=now<expires or not 0<expires-issued<=600
            or physical!=dict(battery_disconnected=True,usb_only=True,stationary=True,props_treated_as_installed=True)
            or any(type(value) is not bool for value in physical.values())):
        raise ValueError('Unapproved, expired or physically unconfirmed ground permit')
    registry=inspect_evidence(heading,board,proof)
    if registry['active_revision_id']!=runtime.inputs.rigid_revision:raise ValueError('Current rigid revision changed')
    paths=evidence_paths(heading,board,proof,backup)
    if (json.loads(paths['incident'].read_text()).get('block_active_benches') is not True
            or json.loads(paths['physical_hold'].read_text()).get('block_props_off_benches') is not True):
        raise ValueError('General incident and physical holds must remain intact')
    baseline=load_backup(backup,now)
    seal=EvidenceSeal(paths,record['hashes'],clock=clock)
    if not seal.refresh():raise ValueError(seal.fault)
    exclusive_json(str(path)+'.consumed.json',dict(scope=SCOPE,consumed_unix=now,
        permit_sha256=digest(path),boot_session=runtime.inputs.boot_session,
        rigid_revision=runtime.inputs.rigid_revision,general_holds_modified=False))
    return GroundPreparationGrant(runtime,record,baseline,seal,clock=clock,origin=origin)


class RecoveryJournal:
    """Atomic state and fsync BEFORE any possible parameter bytes; no auto-clear."""
    def __init__(self,path):
        self.path=Path(path)
        self.record=dict(schema=1,scope=SCOPE,needs_restore=False,verified_closed=False,events=[])
        exclusive_json(self.path,self.record)

    def _save(self):
        pending=Path(str(self.path)+'.pending')
        exclusive_json(pending,self.record)
        os.replace(pending,self.path);sync_directory(self.path.parent)

    def __call__(self,entry):
        self.record['events'].append(deepcopy(entry));self.record['needs_restore']=True
        self._save();return True

    def finalize(self,runtime,io):
        safe=(not self.record['events'] or runtime.closed
              and runtime.coordinator.fusion.phase=='CLOSED' and io.failure is None)
        self.record.update(verified_closed=bool(safe),needs_restore=not safe)
        self._save();return bool(safe)


class RateRecoveryJournal(RecoveryJournal):
    """Persist the existing exact-rate lifecycle before any SET can leave it."""
    def __init__(self,path,*,scope=SCOPE):
        self.path=Path(path)
        self.record=dict(schema=1,scope=scope,needs_restore=False,verified_closed=False,rates=None)
        exclusive_json(self.path,self.record)

    def __call__(self,summary):
        self.record.update(rates=deepcopy(summary),needs_restore=bool(summary['possibly_changed'])
            and not summary['restoration_verified'],verified_closed=summary['restoration_verified'])
        self._save()

    def covers(self,rates,msg):
        summary=self.record['rates']
        return bool(summary and self.record['needs_restore'] is True
            and int(msg.param1) in summary['possibly_changed']
            and summary['original_intervals_us']==rates.original)

    def finalize(self,runtime,io):
        rates=runtime.rates
        safe=not rates.changed or rates.summary()['restoration_verified'] and io.failure is None
        self.record.update(verified_closed=bool(safe),needs_restore=not safe)
        self._save();return bool(safe)


def require_no_pending_recovery(root):
    paths=[*Path(root).glob('*/recovery.json*'),*Path(root).glob('*/rates_recovery.json*'),
           *Path(root).glob('*/flight_recovery.json*')]
    for path in paths:
        if path.name.endswith('.pending'):raise ValueError('Incomplete durable recovery transaction: '+str(path))
        if json.loads(path.read_text()).get('needs_restore') is not False:
            raise ValueError('Previous ground preparation needs recovery: '+str(path))
