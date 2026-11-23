"""Offline Pocket arrival workflow; existing synthetic byte peer, no devices."""
import os
import socket
from copy import deepcopy

import pytest
import native_rc_ground as ground
import vicon_fixed_point_entry as entry
from rc_watch import NativeRCMonitor, review_parameters, summarize_observation
from test_pocket_wifi_delta import Peer, ready_monitor


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    monkeypatch.setattr(socket,'socket',lambda *a,**kw:pytest.fail('No physical network'))


def fake_clock(monkeypatch):
    now=[100.];peer=Peer()
    monkeypatch.setattr(ground.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(ground.time,'sleep',lambda dt:now.__setitem__(0,now[0]+dt))
    monkeypatch.setattr(ground,'FixedPeerUDP',lambda *a,**kw:peer)
    return now,peer


def test_go_wait_preserves_full_window_and_reads_parameters_only_once(tmp_path,monkeypatch):
    now,peer=fake_clock(monkeypatch);waiting=[]
    def go():
        if not waiting:waiting.append(now[0])
        return now[0]-waiting[0]>=5
    report=ground.run(output=tmp_path/'run',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        motor_power_isolated=True,observe_seconds=3,start_ready=go,notice=lambda _:None)
    assert report['error'] is None and peer.closed
    assert report['window_started_at']>=waiting[0]+5
    assert report['window_completed_at']-report['window_started_at']>=3
    assert report['observation_window_complete'] and report['observation_summary']['samples']>0
    assert all(report['window_started_at']<=s['at']<=report['window_deadline_at'] for s in report['rc_observations'])
    names=[msg.param_id for msg in peer.calls if msg.get_type()=='PARAM_REQUEST_READ']
    assert len(names)==len(set(names))==len(ground.PARAMS)
    assert all(msg.get_type() in ('HEARTBEAT','COMMAND_LONG','TIMESYNC','PARAM_REQUEST_READ','SERIAL_CONTROL') for msg in peer.calls)
    assert all(msg.command==512 for msg in peer.calls if msg.get_type()=='COMMAND_LONG')
    assert report['parameter_writes']==report['control_messages']==report['localization_messages']==0
    assert report['parameter_readbacks']['COM_RC_IN_MODE']['type']==9
    assert 'COM_RC_IN_MODE:type' in report['parameter_review']['manual_attention']


@pytest.mark.parametrize('line',['STOP\n','','GONE\n'])
def test_go_cancel_saves_report_without_motion_window(tmp_path,monkeypatch,line):
    now,peer=fake_clock(monkeypatch)
    report=ground.run(output=tmp_path/'run',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        motor_power_isolated=True,observe_seconds=3,start_ready=lambda:entry.ground_go_ready(lambda:line),notice=lambda _:None)
    assert report['error'] and peer.closed
    assert 'window_started_at' not in report and not report['observation_window_complete']
    assert not report['rc_observations'] and (tmp_path/'run/report.json').is_file()


def readbacks():
    values=dict(COM_RC_IN_MODE=0,COM_RC_OVERRIDE=3,COM_RC_STICK_OV=30.,
        RC_MAP_ROLL=1,RC_MAP_PITCH=2,RC_MAP_THROTTLE=3,RC_MAP_YAW=4,RC_MAP_FLTMODE=5,RC_MAP_KILL_SW=7)
    return {k:dict(value=v,type=9 if k=='COM_RC_STICK_OV' else 6,wire_value_hex='00000000') for k,v in values.items()}


def test_review_changes_only_actual_route_override_differences_keeps_original():
    rows=readbacks();before=deepcopy(rows)
    clean=review_parameters(rows)
    assert clean['runtime_takeover_configuration_valid'] and not clean['proposed_differences']
    rows['COM_RC_IN_MODE']['value']=1;rows['COM_RC_OVERRIDE']['value']=4
    result=review_parameters(rows)
    assert [(r['name'],r['proposed_value']) for r in result['proposed_differences']]==[('COM_RC_IN_MODE',0),('COM_RC_OVERRIDE',7)]
    assert result['proposed_differences'][1]['original']==rows['COM_RC_OVERRIDE']
    assert before['RC_MAP_KILL_SW']==rows['RC_MAP_KILL_SW']
    assert not result['flight_authorized'] and result['parameters_written']==0


def test_missing_wrong_type_and_duplicate_mapping_never_get_guessed_values():
    rows=readbacks();del rows['RC_MAP_YAW'];rows['RC_MAP_KILL_SW']['value']=1
    rows['COM_RC_IN_MODE']['type']=9
    result=review_parameters(rows)
    assert result['missing_parameters']==['RC_MAP_YAW']
    assert 'channel_mapping_overlap' in result['manual_attention']
    assert 'COM_RC_IN_MODE:type' in result['manual_attention']
    assert not result['proposed_differences']


def test_channels_need_fc_source_freshness_and_summary_cannot_approve():
    rc=ready_monitor()
    rc.channels=dict(received_at=5.,time_boot_ms=1,channel_count=4,values=[1000,1500,1500,1500],rssi=None)
    old=rc.status(5.1);assert not old['channels_fresh']
    rc.channels['time_boot_ms']=100000
    current=rc.status(5.1);assert current['channels_fresh']
    current['selected_manual_input'].update(roll=.4,pitch=0.,yaw=0.,throttle=-1.)
    current['kill_requested']=True;current['output_lock_confirmed']=False
    summary=summarize_observation([dict(at=5.,status=old),dict(at=5.1,status=current)])
    assert summary['channel_ranges']['1']==dict(min=1000,max=1000,samples=1)
    assert summary['kill_intents_observed']==[False,True]
    assert summary['output_lock_states_observed']==[False]
    assert summary['selected_axis_ranges']['roll']['max']==.4
    assert not any(summary[k] for k in ('airborne_takeover_verified','receiver_loss_response_verified','flight_authorized'))


def test_console_reader_handles_fragmented_go_and_discards_partial_eof():
    read,write=os.pipe()
    try:
        reader=entry.terminal_reader(read)
        os.write(write,b'G');assert reader() is None
        os.write(write,b'O\nTAIL');assert reader() is None
        assert entry.ground_go_ready(reader)
        os.close(write);write=None
        assert reader() is None
        with pytest.raises(RuntimeError):entry.ground_go_ready(reader)
    finally:
        os.close(read)
        if write is not None:os.close(write)


@pytest.mark.parametrize('flags',[
    ['--wait-for-go'],['--ground-observe','--wait-for-go','--observe-seconds','0'],
    ['--ground-observe','--fusion-observe','--wait-for-go']])
def test_bad_go_cli_scope_rejected_before_devices(flags):
    with pytest.raises(SystemExit):entry.main(flags)
