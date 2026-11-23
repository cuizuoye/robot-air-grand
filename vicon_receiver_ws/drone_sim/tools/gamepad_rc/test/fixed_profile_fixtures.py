"""SYNTHETIC fixture only. Never copy this approval/persistence into a profile."""
from copy import deepcopy

import ekf_static_profile as p
from trs_uart_readonly_check import UID2
from test_trs_vision_reference import baseline


def values():
    result = baseline()
    for name, kind in p.PARAMETER_TYPES.items():
        result.setdefault(name, 0)
        result[name] = int(result[name]) if kind == 6 else float(result[name])
    result.update(EKF2_EV_CTRL=11, EKF2_EV_DELAY=37.0, COM_OF_LOSS_T=1.0,
                  EKF2_EVP_NOISE=.1, EKF2_EVV_NOISE=.1, EKF2_EVA_NOISE=.1,
                  EKF2_EVP_GATE=5.0, EKF2_EVV_GATE=3.0, COM_POSCTL_NAVL=1)
    return {name:p.typed_value(value,p.PARAMETER_TYPES[name]) for name,value in result.items()}


def approved_profile():
    v = values()
    rows = [dict(name=n, type=p.PARAMETER_TYPES[n], value=v[n]) for n in p.READ_PARAMS]
    result = p.draft_from_snapshot({'records':rows}, profile_id='SYNTHETIC-ONLY-no-hardware')
    result['firmware'].update(uid2=UID2, artifact_sha256='a'*64)
    for spec in result['parameters'].values():
        spec.update(status='approved', evidence='SYNTHETIC UNIT TEST ONLY')
    result['delay_calibration'].update(status='approved', evidence='SYNTHETIC 37ms; not a lab calibration')
    result['persistence'].update(status='verified_after_reboot', evidence_sha256='b'*64)
    result['persistence']['configuration_sha256'] = p.configuration_digest(result)
    return result


def snapshot(profile=None):
    profile = approved_profile() if profile is None else profile
    return dict(records=[dict(name=n, type=spec['type'], value=spec['value'])
                         for n,spec in profile['parameters'].items()])


def initial_trial_profile(profile=None):
    """Synthetic scoped contract; intentionally nonexistent physical evidence."""
    result=deepcopy(profile) if profile is not None else approved_profile()
    result['parameters']['COM_RC_IN_MODE']['value']=0
    result['timestamp_mode']='local_ros_timesync'
    result['timestamp_evidence']='SYNTHETIC ONLY'
    for row in result['parameters'].values():row['status']='unreviewed'
    result['parameters']['EKF2_EV_DELAY']['status']='pending_calibration'
    result['delay_calibration']['status']='pending_calibration'
    result['persistence']['status']='unverified'
    result['initial_trial']=dict(scope=p.INITIAL_TRIAL_SCOPE,
        configuration_sha256=p.configuration_digest(result),timestamp_mode='local_ros_timesync',
        revision='SYNTHETIC-ONLY-rigid-revision',origin=[0,0,0],orientation=[0,0,0,1],
        calibration_approved=False,flight_authorized=False,precision_tuning_deferred=True,
        evidence={name:dict(path='/SYNTHETIC-NONEXISTENT/'+name,sha256='c'*64)
                  for name in ('layout','battery_report','battery_analysis','backup','manifest')})
    return result


def current(profile=None, **changes):
    profile = approved_profile() if profile is None else profile
    v = {n:spec['value'] for n,spec in profile['parameters'].items()}
    v.update(changes)
    return p.validate_current(profile,v,dict(p.PARAMETER_TYPES),dict.fromkeys(v,10.),10.1,
        identity={'uid2':UID2,'firmware':'1.14.3'},boot_session='SYNTHETIC-boot-1')
