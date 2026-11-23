"""No hardware: exact frozen targets, source-time bounds and prior evidence."""
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from frozen_target_stream import FrozenTargetStream, source_age, require_prior
from disarmed_offboard_policy import GroundObservation, DisarmedHoldGate
from trs_uart_readonly_check import UID2
from rigid_body_revision import digest


def observation(t=0):
    return GroundObservation(t,UID2,'1.14.3',(.34,-.27,-.08),.3,(0,0,0),0,
        estimator_ready=True,gamepad_available=True)


def test_only_actual_fc_xyz_yaw_are_frozen_and_no_mode_entry_is_exposed():
    e=m.MAVLink(None);o=observation();g=FrozenTargetStream(o,50593792,authorized=True)
    for i in range(48):
        current=replace(o,t=i/16)
        msg=g.message(e,100000+i*62,current,50593792)
        assert g.permits(msg,current,50593792)
        g.record_send(current.t,msg,len(msg.pack(e)))
    g.finish();r=g.summary()
    assert r['actual_send_count']==48 and r['actual_span_s']>2
    assert r['frozen_position']==list(o.position) and r['frozen_yaw']==o.yaw
    assert not r['mode_commands_sent'] and not r['flight_authorized']
    with pytest.raises(RuntimeError):g.message(e,103000,replace(o,t=3),50593792)


@pytest.mark.parametrize('command',[176,400,21,22,185,246,511])
def test_even_offboard_mode_command_is_rejected(command):
    e=m.MAVLink(None);o=observation();g=FrozenTargetStream(o,50593792,authorized=True)
    assert not g.permits(e.command_long_encode(1,1,command,0,1,6,0,0,0,0,0),o,50593792)


@pytest.mark.parametrize('field,value',[('x',0),('z',-.5),('yaw',1),('vx',.1),
    ('afx',.1),('yaw_rate',.1),('coordinate_frame',8),('type_mask',0),('target_system',2)])
def test_moving_or_ignored_nonzero_fields_rejected(field,value):
    e=m.MAVLink(None);o=observation();g=FrozenTargetStream(o,50593792,authorized=True)
    msg=g.message(e,100000,o,50593792);setattr(msg,field,value)
    assert not g.permits(msg,o,50593792)


@pytest.mark.parametrize('change',[dict(position=(.4,-.27,-.08)),dict(pose_age_s=.151),
    dict(status_age_s=2.),dict(reset_counters=(1,0,0)),dict(primary_estimator=1),
    dict(home_latched=True),dict(gamepad_available=False),dict(armed=True),dict(lockdown=False)])
def test_bad_state_latches_no_restart(change):
    o=observation();g=FrozenTargetStream(o,50593792,authorized=True)
    with pytest.raises(RuntimeError):g.check(replace(o,t=.1,**change),50593792)
    with pytest.raises(RuntimeError):g.check(replace(o,t=.2),50593792)


def test_mode_change_or_actual_write_gap_never_resumes():
    o=observation();g=FrozenTargetStream(o,50593792,authorized=True)
    with pytest.raises(RuntimeError):g.check(o,3<<16)
    g=FrozenTargetStream(o,50593792,authorized=True)
    msg=g.message(m.MAVLink(None),100000,o,50593792);g.record_send(0,msg,65)
    with pytest.raises(RuntimeError):g.check(replace(o,t=.201),50593792)


def test_static_source_age_handles_real_2hz_publication_without_widening_mode_gate():
    o=replace(observation(),status_age_s=.65)
    with pytest.raises(ValueError):DisarmedHoldGate(o,operator_authorized=True)
    g=FrozenTargetStream(o,50593792,authorized=True)
    assert g.hold.origin.status_age_s==.65  # Preserve actual age; never clamp it.
    e=m.MAVLink(None)
    for i in range(48):
        current=replace(o,t=i/16,status_age_s=.05+(i%8)/16)
        msg=g.message(e,100000+i*62,current,50593792)
        assert g.permits(msg,current,50593792)
        g.record_send(current.t,msg,len(msg.pack(e)))
    with pytest.raises(ValueError):g.hold.mode_message(e,current)
    g.finish()
    with pytest.raises(ValueError):DisarmedHoldGate(o,operator_authorized=True)


def test_duration_and_packet_budget_are_enforced_without_runner():
    o=observation();g=FrozenTargetStream(o,50593792,authorized=True)
    with pytest.raises(RuntimeError):g.message(m.MAVLink(None),104000,replace(o,t=3.101),50593792)


@pytest.mark.parametrize('mode',[6<<16,4<<16|2<<24,-1,True,2**32])
def test_offboard_takeoff_or_unknown_initial_mode_is_forbidden(mode):
    with pytest.raises(ValueError):FrozenTargetStream(observation(),mode,authorized=True)


def test_fc_source_freshness_is_not_refreshed_by_late_arrival():
    clock=NS(fault=None,MAX_AGE_NS=500000000,DRIFT_PPM=1000,
        reference=dict(received_ns=1_000_000_000,rtt_ns=10000000,
                       lower_ns=99_000_000_000,upper_ns=99_010_000_000))
    assert .03<source_age(clock,100000,1_015_000_000,1_020_000_000,maximum=.15)<.04
    for boot,received,now in ((99000,1_020_000_000,1_020_000_000),
                              (100040,1_020_000_000,1_020_000_000),
                              (100000,900_000_000,1_100_000_000),
                              (100000,1_600_000_000,1_600_000_000)):
        with pytest.raises(RuntimeError):source_age(clock,boot,received,now,maximum=.15)


def test_prior_uses_actual_report_and_consumed_hash_not_summary(tmp_path):
    heading=tmp_path/'heading';board=tmp_path/'board'
    heading.write_text('heading');board.write_text('board')
    permit=tmp_path/'permit.json';permit.write_text(json.dumps(dict(scope='prior',
        hashes=dict(heading=digest(heading),board=digest(board)))))
    (tmp_path/'permit.json.consumed.json').write_text(json.dumps(dict(scope='prior',permit_sha256=digest(permit))))
    for name in ('recovery.json','rates_recovery.json'):
        (tmp_path/name).write_text(json.dumps(dict(verified_closed=True,needs_restore=False)))
    record=dict(compact_ground_fusion_verified=True,low_bandwidth_vision=True,binary_diagnostics=True,
        fc_usb_present_at_end=False,identity={'uid2':UID2},error=None,restore_errors=[],fusion_audits=[{}, {}, {}],
        parameter_restoration_verified=True,all_rate_transactions={'restoration_verified':True},
        actuator_after=dict(armed=False,lockdown=True),flight_authorized=False)
    report=tmp_path/'report.json';report.write_text(json.dumps(record))
    assert require_prior(report,heading,board)['wireless_static_report']==report
    record['compact_ground_fusion_verified']=False;report.write_text(json.dumps(record))
    with pytest.raises(ValueError):require_prior(report,heading,board)
