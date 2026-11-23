from pathlib import Path
import sys
from pymavlink.dialects.v20 import common as m
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from fc_health_readonly import allowed,COMMANDS,PARAMS,USB_IMU_KEYS,validate_usb_imu_profile,load_expected_usb_params,parameters_match_readonly_scope
import json
from types import SimpleNamespace
import pytest


def test_health_fixed_reads_only_no_fault_reset_or_param_writes():
    enc=m.MAVLink(None)
    for p in COMMANDS.values():
        msg=enc.serial_control_encode(10,2,0,0,len(p),p.ljust(70,b'\0'))
        assert allowed(msg,True) and not allowed(msg,False)
    for p in (b'reboot\n',b'hardfault_log fault\n',b'hardfault_log check\n',b'commander arm\n',b'param set EKF2_EV_CTRL 11\n'):
        assert not allowed(enc.serial_control_encode(10,2,0,0,len(p),p.ljust(70,b'\0')),True)
    assert not allowed(enc.param_set_encode(1,1,b'EKF2_EV_CTRL',0,6),True)
    assert not allowed(enc.serial_control_encode(10,0,0,0,0,bytes(70)),True)


def test_height_reference_is_readonly_not_implicitly_reset_to_baro():
    enc=m.MAVLink(None)
    assert 'EKF2_HGT_REF' in PARAMS
    for name in PARAMS:
        msg=enc.param_request_read_encode(1,1,name.encode(),-1)
        assert allowed(msg,True) and not allowed(msg,False)
    assert not allowed(enc.param_set_encode(1,1,b'EKF2_HGT_REF',0,6),True)


def test_sensor_profile_contains_only_fixed_single_sample_listeners():
    sensors=[payload for key,payload in COMMANDS.items() if key.startswith('sensor_')]
    assert len(sensors)==6
    assert all(p.startswith(b'listener ') and p.endswith(b' -n 1\n') for p in sensors)


def test_health_report_is_fixed_readonly_snapshot():
    assert COMMANDS['health_report'] == b'listener health_report -n 1\n'


def usb_args(base_port, **changes):
    values=dict(port=str(base_port),usb_imu_status=True,usb_battery_off_readonly_confirmed=True,
                physical_confirmed=False,sensor_status=False,board_config=False,regression_seconds=0,
                expected_readonly_baseline=Path(base_port).parent/'baseline.json')
    values.update(changes)
    return SimpleNamespace(**values)


def test_usb_imu_profile_is_minimal_and_does_not_claim_props_removed(tmp_path):
    port=tmp_path/'fake_usb';port.touch()
    validate_usb_imu_profile(usb_args(port),fc_usb=port)
    assert USB_IMU_KEYS==('actuator','version','sensor_selection','sensor_estimator_selector')
    assert all(key in COMMANDS for key in USB_IMU_KEYS)


@pytest.mark.parametrize('changes',[
    {'usb_battery_off_readonly_confirmed':False}, {'physical_confirmed':True},
    {'sensor_status':True}, {'board_config':True}, {'regression_seconds':20},
    {'usb_imu_status':False}, {'port':'/not/the/fc/usb'},
    {'expected_readonly_baseline':None},
])
def test_usb_imu_profile_rejects_other_modes_or_missing_confirmation(tmp_path,changes):
    port=tmp_path/'fake_usb';port.touch()
    with pytest.raises(ValueError):
        validate_usb_imu_profile(usb_args(port,**changes),fc_usb=port)


def test_usb_imu_profile_requires_present_usb_and_leaves_legacy_scope_unchanged(tmp_path):
    port=tmp_path/'missing_usb'
    with pytest.raises(ValueError): validate_usb_imu_profile(usb_args(port),fc_usb=port)
    validate_usb_imu_profile(usb_args(port,usb_imu_status=False,usb_battery_off_readonly_confirmed=False,expected_readonly_baseline=None),fc_usb=port)


def test_explicit_usb_read_baseline_does_not_relax_legacy_zero_requirement():
    values={'EKF2_EV_CTRL':2,'EKF2_EV_DELAY':0.,'EKF2_HGT_REF':3}
    assert not parameters_match_readonly_scope(values)
    assert parameters_match_readonly_scope(values,dict(values))
    assert not parameters_match_readonly_scope({**values,'EKF2_EV_DELAY':20},values)
    assert not parameters_match_readonly_scope({**values,'EKF2_HGT_REF':0},values)
    assert not parameters_match_readonly_scope({**values,'OTHER':0},values)


def test_expected_baseline_requires_complete_unique_same_version_table(tmp_path):
    path=tmp_path/'params.json'
    record={'complete':True,'firmware':{'version':'1.14.3'},'system_id':1,'component_id':1,
            'expected_count':3,'received_count':3,
            'records':[{'name':name,'value':value} for name,value in
                       [('EKF2_EV_CTRL',2),('EKF2_EV_DELAY',0.),('EKF2_HGT_REF',3)]]}
    path.write_text(json.dumps(record))
    assert load_expected_usb_params(path)=={'EKF2_EV_CTRL':2,'EKF2_EV_DELAY':0.,'EKF2_HGT_REF':3}
    for changes in ({'complete':False},{'expected_count':4},{'received_count':2},
                    {'firmware':{'version':'1.17.0'}},{'system_id':2}):
        path.write_text(json.dumps({**record,**changes}))
        with pytest.raises(ValueError): load_expected_usb_params(path)
