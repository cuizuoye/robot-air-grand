"""Plan B: typed, READ-ONLY fixed EKF profile comparison. No MAVLink writer.

Approval, persistence, current parameter reception and flight readiness are
different facts. A JSON file/old backup never proves current hardware readiness.
This module has no serial, ROS, subprocess, PARAM_SET or permit-generation API.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import stat
import struct

SCHEMA = 1
SCOPE = 'px4-v1143-fixed-ekf-ground-v1'
VERSION = '1.14.3'
TARGET = 'micoair_h743-v2'
PARAMETER_MAX_AGE = 4.0  # Existing parameter gate, NOT the 500ms state gate.

# Audited from installed source: EKF2.hpp parameter bindings + ekf2_params.c.
# Include velocity noise/gate even though this XYZ/yaw profile sends no velocity.
INT_PARAMETERS = (
    'EKF2_EV_CTRL', 'EKF2_HGT_REF', 'EKF2_RNG_CTRL', 'EKF2_BARO_CTRL',
    'EKF2_GPS_CTRL', 'EKF2_OF_CTRL', 'EKF2_AID_MASK', 'EKF2_EV_NOISE_MD',
    'EKF2_EV_QMIN', 'EKF2_MAG_TYPE', 'EKF2_MAG_CHECK', 'COM_RC_IN_MODE',
    'COM_OBL_RC_ACT', 'COM_RCL_EXCEPT', 'NAV_RCL_ACT', 'NAV_DLL_ACT',
    'COM_ARM_HFLT_CHK', 'COM_POSCTL_NAVL', 'COM_DL_LOSS_T',
)
FLOAT_PARAMETERS = (
    'EKF2_EV_DELAY', 'EKF2_EVP_NOISE', 'EKF2_EVV_NOISE', 'EKF2_EVA_NOISE',
    'EKF2_EVP_GATE', 'EKF2_EVV_GATE',
    'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z',
    'EKF2_IMU_POS_X', 'EKF2_IMU_POS_Y', 'EKF2_IMU_POS_Z',
    'COM_OF_LOSS_T', 'COM_RC_LOSS_T', 'COM_KILL_DISARM',
)
PARAMETER_TYPES = {**dict.fromkeys(INT_PARAMETERS, 6), **dict.fromkeys(FLOAT_PARAMETERS, 9)}
READ_PARAMS = tuple(PARAMETER_TYPES)
STATUSES = ('approved', 'pending_calibration', 'unreviewed')
SOURCE_FILES = ('src/modules/ekf2/EKF2.hpp', 'src/modules/ekf2/ekf2_params.c')
INITIAL_TRIAL_SCOPE = 'pocket-fixed-origin-0.3m-3s-initial-trial-v1'


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _text(value):
    return isinstance(value,str) and bool(value.strip())


def _sha(value):
    return isinstance(value,str) and re.fullmatch('[0-9a-f]{64}',value) is not None


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate JSON key: ' + key)
        result[key] = value
    return result


def load_json(path):
    """Only bounded regular local files, never a device/FIFO or /proc input."""
    path = Path(path).resolve(strict=True)
    if len(path.parts)>1 and path.parts[1] in ('dev', 'proc', 'sys'):
        raise ValueError('Device/pseudo-filesystem inputs are not offline snapshots')
    info = path.stat()
    if not stat.S_ISREG(info.st_mode) or info.st_size > 8 * 1024 * 1024:
        raise ValueError('Bounded regular JSON file required')
    value = json.loads(path.read_text(), object_pairs_hook=_object,
                       parse_constant=lambda text: (_ for _ in ()).throw(ValueError(text)))
    if not isinstance(value, dict):
        raise ValueError('JSON object required')
    return value


def typed_value(value, kind):
    if kind == 6 and type(value) is int and -(2**31) <= value < 2**31:
        return value
    if kind == 9 and type(value) in (int, float) and math.isfinite(value):
        try:
            result = struct.unpack('<f', struct.pack('<f', value))[0]
        except (OverflowError, struct.error):
            pass
        else:
            if math.isfinite(result):
                return result
    raise ValueError('Invalid INT32/finite REAL32 value')


def records_by_name(snapshot):
    rows = snapshot.get('records')
    if not isinstance(rows, list):
        raise ValueError('Typed snapshot records list required')
    result = {}
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get('name'), str):
            raise ValueError('Named parameter record required')
        name = row['name']
        if name in result:
            raise ValueError('Duplicate parameter record: ' + name)
        if name in PARAMETER_TYPES:
            result[name] = deepcopy(row)
    return result


def profile_errors(profile):
    errors = []
    if profile.get('schema') != SCHEMA or type(profile.get('schema')) is not int or profile.get('scope') != SCOPE:
        errors.append('profile_schema_or_scope')
    if not isinstance(profile.get('profile_id'), str) or not profile['profile_id'].strip():
        errors.append('profile_id_missing')
    fw = profile.get('firmware', {})
    if not isinstance(fw, dict) or fw.get('version') != VERSION or fw.get('target') != TARGET:
        errors.append('firmware_profile_mismatch')
    if profile.get('timestamp_mode') not in ('fc_receive_time','local_ros_timesync'):
        errors.append('unreviewed_timestamp_mode')
    if profile.get('timestamp_mode')=='local_ros_timesync' and not _text(profile.get('timestamp_evidence')):
        errors.append('timestamp_evidence_missing')
    parameters = profile.get('parameters', {})
    if not isinstance(parameters, dict):
        return errors + ['parameters_object_missing']
    if set(parameters) - set(READ_PARAMS):
        errors.append('unknown_profile_parameters')
    for name, row in parameters.items():
        if name not in PARAMETER_TYPES:
            continue
        if (not isinstance(row, dict) or type(row.get('type')) is not int
                or row['type'] != PARAMETER_TYPES[name] or row.get('status') not in STATUSES):
            errors.append('invalid_parameter_spec:' + name)
            continue
        if row.get('value') is not None:
            try:
                value = typed_value(row['value'], row['type'])
                if name == 'EKF2_EV_DELAY' and not 0 <= value <= 300:
                    raise ValueError('Delay outside actual 1.14.3 range')
                if name == 'EKF2_EV_CTRL' and not 0 <= value <= 15:
                    raise ValueError('Invalid EV bitmask')
                limits={'EKF2_HGT_REF':(0,3),'EKF2_RNG_CTRL':(0,2),'EKF2_BARO_CTRL':(0,1),
                        'EKF2_OF_CTRL':(0,1),'EKF2_EV_NOISE_MD':(0,1),'EKF2_EV_QMIN':(0,100)}
                if name in limits and not limits[name][0] <= value <= limits[name][1]:
                    raise ValueError('Parameter outside pinned source range')
                if name in ('EKF2_EVP_NOISE','EKF2_EVV_NOISE','EKF2_EVA_NOISE') and value <= 0:
                    raise ValueError('Positive finite measurement noise required')
                if name in ('EKF2_EVP_GATE','EKF2_EVV_GATE') and value < 1:
                    raise ValueError('Innovation gate must be at least one')
            except ValueError:
                errors.append('invalid_parameter_value:' + name)
        elif row.get('status') == 'approved':
            errors.append('approved_value_missing:' + name)
        if row.get('status') == 'approved' and not _text(row.get('evidence')):
            errors.append('approval_evidence_missing:' + name)
    return errors


def configuration_digest(profile):
    """FC persistence binds FC identity/image and typed values only.

    Host timestamps and evidence paths are not stored on the FC. They remain
    bound by fingerprint(profile) in current-session and execution reviews.
    This never creates a persistence proof or upgrades an unapproved profile.
    """
    firmware=profile.get('firmware') or {}
    return fingerprint(dict(firmware={k:firmware.get(k) for k in
        ('version','target','uid2','artifact_sha256')}, parameters={
            name: dict(type=row.get('type'), value=row.get('value'))
            for name, row in profile.get('parameters', {}).items() if isinstance(row, dict)}))


def compare(profile, records):
    """Offline comparison only; never issues current-session readiness."""
    errors = profile_errors(profile)
    specs = profile.get('parameters', {})
    if not isinstance(specs, dict):
        specs = {}
    missing, mismatch, type_mismatch, invalid, pending, unapproved = [], [], [], [], [], []
    for name, kind in PARAMETER_TYPES.items():
        spec, actual = specs.get(name), records.get(name)
        if not isinstance(spec, dict) or spec.get('value') is None:
            missing.append(dict(name=name, where='profile'))
        if not isinstance(actual, dict):
            missing.append(dict(name=name, where='snapshot'))
        if not isinstance(spec, dict):
            continue
        if spec.get('status') == 'pending_calibration':
            pending.append(name)
        if spec.get('status') != 'approved':
            unapproved.append(name)
        if not isinstance(actual, dict):
            continue
        if type(actual.get('type')) is not int or actual['type'] != kind:
            type_mismatch.append(name)
            continue
        try:
            got = typed_value(actual.get('value'), kind)
            if 'wire_value_hex' in actual:
                raw = bytes.fromhex(actual['wire_value_hex'])
                if len(raw) != 4 or struct.unpack('<i' if kind == 6 else '<f', raw)[0] != got:
                    raise ValueError('Typed wire backup mismatch')
            if spec.get('value') is not None and got != typed_value(spec['value'], kind):
                mismatch.append(dict(name=name, expected=spec['value'], actual=got))
        except (ValueError, TypeError, struct.error):
            invalid.append(name)
    delay = profile.get('delay_calibration', {})
    if not isinstance(delay, dict) or delay.get('status') != 'approved' or not _text(delay.get('evidence')):
        if 'EKF2_EV_DELAY' not in pending:
            pending.append('EKF2_EV_DELAY')
    persistence = profile.get('persistence', {})
    persisted = bool(not errors and isinstance(persistence, dict)
        and persistence.get('status') == 'verified_after_reboot'
        and persistence.get('configuration_sha256') == configuration_digest(profile)
        and _sha(persistence.get('evidence_sha256')))
    fw = profile.get('firmware', {})
    identity_reviewed = bool(isinstance(fw, dict) and isinstance(fw.get('uid2'), str)
        and re.fullmatch('[0-9a-f]{36}',fw['uid2']) and _sha(fw.get('artifact_sha256')))
    match = not (errors or missing or mismatch or type_mismatch or invalid)
    approved = bool(match and not pending and not unapproved and persisted and identity_reviewed)
    trial = bool(match and identity_reviewed and initial_trial_contract(profile))
    return dict(schema=1, scope=SCOPE, profile_sha256=fingerprint(profile),
        profile_errors=errors, missing=missing, mismatched=mismatch, type_mismatch=type_mismatch,
        invalid_records=invalid, pending_calibration=pending, unapproved=unapproved,
        approved_parameters=[n for n, s in specs.items() if isinstance(s, dict) and s.get('status') == 'approved'],
        configuration_match=match, configuration_approved=approved,
        initial_trial_eligible=trial, configuration_usable=approved or trial,
        persistence_evidence_present=persisted, firmware_binding_reviewed=identity_reviewed,
        current_configuration_verified=False, timed_out=[], physical_authorization=False,
        flight_authorized=False, scope_note='File comparison is not fresh FC reception or Flash completion proof')


def validate_current(profile, values, types, received_at, now, *, identity, requested_at=None, boot_session=None):
    """Validate owner-collected, same-session replies; never invent their time."""
    rows = {name: dict(name=name, value=value, type=types.get(name)) for name, value in values.items()}
    result = compare(profile, rows)
    time_ok = type(now) in (int, float) and math.isfinite(now) and now >= 0
    def fresh(name):
        at=received_at.get(name)
        if not time_ok or type(at) not in (int,float) or not math.isfinite(at) or not 0 <= now-at < PARAMETER_MAX_AGE:
            return False
        if requested_at is not None:
            requested=requested_at.get(name)
            if type(requested) not in (int,float) or not math.isfinite(requested) or not 0 <= requested <= at:
                return False
        return True
    result['timed_out'] = [name for name in READ_PARAMS if not fresh(name)]
    fw = profile.get('firmware', {})
    result['identity_match'] = bool(isinstance(identity, dict) and isinstance(fw, dict)
        and identity.get('uid2') == fw.get('uid2') and identity.get('firmware') == VERSION)
    result['identity_binding'] = {key:identity.get(key) for key in ('uid2','firmware')} if isinstance(identity,dict) else None
    result['boot_session'] = boot_session
    result['current_configuration_verified'] = bool(result['configuration_usable']
        and result['identity_match'] and not result['timed_out'])
    return result


def draft_from_snapshot(snapshot, *, profile_id='unapproved-fixed-ekf-draft'):
    """Preserve observed values; never silently substitute CTRL=11 or DELAY=65."""
    rows = records_by_name(snapshot)
    parameters = {}
    for name, kind in PARAMETER_TYPES.items():
        row = rows.get(name, {})
        parameters[name] = dict(type=kind, value=row.get('value'),
            status='pending_calibration' if name == 'EKF2_EV_DELAY' else 'unreviewed', evidence='')
    return dict(schema=SCHEMA, scope=SCOPE, profile_id=profile_id,
        firmware=dict(version=VERSION, target=TARGET, uid2=None, artifact_sha256=None),
        timestamp_mode='fc_receive_time', parameters=parameters,
        delay_calibration=dict(status='pending_calibration', evidence=''),
        persistence=dict(status='unverified', configuration_sha256=None, evidence_sha256=None),
        source_files=list(SOURCE_FILES), physical_authorization=False, flight_authorized=False)


def plan_changes(document, snapshot):
    """Pure offline change list shared by CLI reports; never call maintenance."""
    records = records_by_name(snapshot)
    result = compare(document, records)
    changes = []
    specifications=document.get('parameters', {})
    if not isinstance(specifications,dict):
        specifications={}
    for name, spec in specifications.items():
        if not isinstance(spec, dict) or name not in PARAMETER_TYPES:
            continue
        actual = records.get(name)
        try:
            target = typed_value(spec.get('value'), spec.get('type'))
            current = typed_value(actual.get('value'), actual.get('type')) if actual else None
        except (ValueError, AttributeError):
            continue
        if actual is None or actual.get('type') != spec['type'] or current != target:
            changes.append(dict(name=name, type=spec['type'], current=current, target=target,
                status=spec.get('status'), approved=spec.get('status') == 'approved',
                reboot_required=name in ('EKF2_EV_DELAY', 'EKF2_HGT_REF')))
    return dict(mode='offline-dry-run-only', validation=result, proposed_changes=changes,
        actual_writes=0, devices_opened=0, maintenance_authorized=False,
        flash_save_completion_verified=False, flight_authorized=False,
        live_maintenance_implemented=False,
        required_future_steps=['separate manual maintenance approval', 'typed backup and bounded writes',
            'actual save completion (not PARAM_VALUE/early storage ACK)', 'controlled reboot',
            'complete same-aircraft post-reboot readback', 'separate ground session proof'])


def require_approved_profile(profile):
    """Validate the archived contract only; never claim live FC agreement."""
    if not isinstance(profile,dict):
        raise PermissionError('Explicit fixed profile required')
    records={name:dict(type=row.get('type'),value=row.get('value'))
             for name,row in profile.get('parameters',{}).items() if isinstance(row,dict)}
    result=compare(profile,records)
    if not result['configuration_approved']:
        raise PermissionError('Fixed profile is unapproved, incomplete or lacks calibration/persistence evidence')
    if profile['parameters']['EKF2_EV_CTRL']['value']!=11:
        raise PermissionError('Fixed-point position/yaw sender requires EV_CTRL=11, without EV velocity')
    return result


def initial_trial_contract(profile):
    """Pure contract check, not calibration, evidence I/O or flight permission.

    A live typed readback replaces a persistence *label* for this unchanged
    configuration. Approximate extrinsics and untuned delay remain explicit.
    The caller must also bind the native pilot, envelope and measured layout.
    """
    try:
        trial=profile['initial_trial']
        refs=trial['evidence']
        return bool(trial['scope']==INITIAL_TRIAL_SCOPE
            and trial['configuration_sha256']==configuration_digest(profile)
            and trial['timestamp_mode']==profile['timestamp_mode']=='local_ros_timesync'
            and trial['calibration_approved'] is False and trial['flight_authorized'] is False
            and trial['precision_tuning_deferred'] is True
            and _text(trial['revision'])
            and len(trial['origin'])==3 and len(trial['orientation'])==4
            and all(type(v) in (int,float) and math.isfinite(v)
                    for v in trial['origin']+trial['orientation'])
            and set(refs)=={'layout','battery_report','battery_analysis','backup','manifest'}
            and all(isinstance(ref,dict) and _text(ref.get('path'))
                    and Path(ref['path']).is_absolute() and _sha(ref.get('sha256')) for ref in refs.values()))
    except (KeyError,TypeError,AttributeError,ValueError):
        return False


def require_mission_profile(profile, *, pilot_source, height, hover_seconds):
    """Existing mission contract, with one narrowly scoped initial-trial route."""
    if not isinstance(profile,dict):raise PermissionError('Explicit fixed profile required')
    if 'horizontal_motion' in profile:
        from follow_target_generator import HorizontalMotionSpec
        HorizontalMotionSpec.from_dict(profile['horizontal_motion'])
        if pilot_source != 'native_rc':
            raise PermissionError('Horizontal motion requires Pocket; no gamepad route extension')
    if 'initial_trial' not in profile:return require_approved_profile(profile)
    result=compare(profile,profile.get('parameters',{}))
    if (not result['initial_trial_eligible'] or pilot_source!='native_rc'
            or type(height) not in (int,float) or not .2<=height<=.5
            or type(hover_seconds) not in (int,float) or not 0<hover_seconds<=3
            or profile['parameters']['COM_RC_IN_MODE']['value']!=0
            or profile['parameters']['EKF2_EV_CTRL']['value']!=11):
        raise PermissionError('Initial trial requires bound baseline, Pocket, frozen origin, height 0.2..0.5 m and hover <=3 s')
    return result


def inspect_initial_trial_evidence(profile, *, revision, origin, orientation):
    """Reuse saved files once before opening devices; never called per pose.

    Historical static success establishes a usable baseline, not airborne
    performance. The existing live preparation and explicit TAKEOFF remain.
    """
    if 'initial_trial' not in profile:return
    if not initial_trial_contract(profile):raise PermissionError('Invalid initial-trial contract')
    from rigid_body_revision import digest, measured_capture_inputs, inspect_current_calibration
    trial=profile['initial_trial'];refs=trial['evidence']
    if 'rigid_frame_transfer' in trial:
        # Retain the original measured/fusion evidence as historical evidence;
        # never edit its coordinates to make a new same-name rigid body pass.
        from rigid_body_revision import inspect_frame_transfer
        transfer=trial['rigid_frame_transfer']
        base_ref=transfer['baseline_profile']
        if digest(base_ref['path'])!=base_ref['sha256']:
            raise ValueError('Rigid-frame baseline profile changed')
        baseline=load_json(base_ref['path']);old=baseline['initial_trial']
        if ('rigid_frame_transfer' in old or refs!=old['evidence']
                or configuration_digest(baseline)!=configuration_digest(profile)):
            raise ValueError('Rigid-frame transfer cannot replace parameter or baseline evidence')
        inspect_initial_trial_evidence(baseline,**{k:old[k] for k in ('revision','origin','orientation')})
        inspect_frame_transfer(transfer,old,trial)
        if (revision!=trial['revision'] or list(origin)!=trial['origin']
                or list(orientation)!=trial['orientation']):
            raise ValueError('Runtime extrinsics differ from transferred profile')
        return
    for name,ref in refs.items():
        if digest(ref['path'])!=ref['sha256']:raise ValueError('Initial-trial evidence changed: '+name)
    layout=measured_capture_inputs(refs['layout']['path'])
    inspect_current_calibration(layout['heading'],layout['board'])
    if (revision!=trial['revision'] or revision!=layout['revision']
            or list(origin)!=trial['origin'] or list(origin)!=layout['origin']
            or list(orientation)!=trial['orientation'] or list(orientation)!=layout['orientation']):
        raise ValueError('Initial-trial layout/revision differs from measured baseline')
    report=load_json(refs['battery_report']['path'])
    analysis=load_json(refs['battery_analysis']['path'])
    if (report.get('error') or report.get('scope')!='wifi-battery-static-sync-fusion'
            or report.get('layout',{}).get('revision')!=revision
            or report['layout']['origin']!=list(origin) or report['layout']['orientation']!=list(orientation)
            or analysis.get('sources',{}).get('report')!=refs['battery_report']
            or analysis.get('reset_counters_unchanged') is not True
            or analysis.get('rate_restored_readback_verified') is not True
            or len(analysis.get('rounds',[]))!=3
            or any(row.get('ev_position_height_yaw_active') is not True
                   or any(row.get('faults_or_rejections',{}).values()) for row in analysis['rounds'])):
        raise ValueError('Initial-trial retained stationary fusion evidence failed')
    backup=Path(refs['backup']['path'])
    if (backup.name!='parameters_verified.json'
            or Path(refs['manifest']['path'])!=backup.parent/'parameters_manifest.json'):
        raise ValueError('Initial-trial archive/manifest binding mismatch')
    from preparation_authorization import load_fixed_backup
    load_fixed_backup(backup.parent,profile,0.)
