"""Display-only regressions: historical evidence must never become live authority."""
import io
import json
from pathlib import Path
import sys
import time
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pymavlink.dialects.v20 import common as mav
from drone_console import ConsoleModel, LogTail, Session, stamp, rviz_environment, active_joint_directory
from readiness_console import LatestStatusPrinter


def drain_seek(session):
    for _ in range(1000):
        session.pump(dt=10)
        if not session.seeking:return
    pytest.fail('seek did not complete')


def test_seek_rebuilds_history_without_future_state_and_resumes(tmp_path):
    path=replay_fixture(tmp_path,'seek')
    (path/'analysis.json').write_text('{"result":"completed report"}')
    s=Session(path);s.bounds=s.read_bounds()
    assert s.bounds==(1.,4.) and s.cursor is None and s.model.rows==0
    s.seek(99);drain_seek(s)
    assert s.eof and s.cursor==4 and s.model.report['result']=='completed report'
    assert s.model.positions['coop_car'][0]==pytest.approx(.3)
    # Backward rebuild must discard future trajectory, final report and events.
    s.seek(1);drain_seek(s)
    assert not s.eof and s.cursor==2 and not s.model.report
    assert s.model.positions['coop_car'][0]==pytest.approx(.1)
    assert s.model.rows==2
    assert not any(e['source']=='历史结论' for e in s.model.events)
    s.pump(dt=1)
    assert s.model.rows==3 and s.cursor==3
    s.seek(-10);drain_seek(s)
    assert s.cursor==1 and s.model.rows==1
    s.seek(2);s.seek(.5);drain_seek(s)
    assert s.cursor==1.5 and s.model.rows==1


def test_seek_bounds_ignore_malformed_and_partial_rows_and_reject_live(tmp_path):
    p=tmp_path/'console_events.jsonl'
    p.write_text('garbage\n'+json.dumps({'at':10,'text':'a'})+'\n'+
                 json.dumps({'at':20,'text':'b'})+'\n{"at":999')
    s=Session(tmp_path)
    assert s.read_bounds()==(10,20)
    assert all(t.offset==0 for t in s.tails)
    with pytest.raises(ValueError,match='时间轴'):s.seek(1)
    s.bounds=(10,20)
    with pytest.raises(ValueError):s.seek(float('nan'))
    live=Session(tmp_path,follow=True);live.bounds=(10,20)
    with pytest.raises(ValueError,match='实时'):live.seek(1)


def replay_fixture(base,name,kind='flight',*,complete=True,closed=True,synthetic=False):
    run=base/name;session=run/'session';session.mkdir(parents=True)
    rows=[]
    for i in range(4):
        phase=('FOLLOW' if kind=='flight' else 'TO_TAKEOFF' if i<3 else 'WAIT_TAKEOFF') if kind!='static' else 'WAIT_START'
        moving=kind!='static'
        rows.append(dict(at_ns=(i+1)*10**9,synthetic=synthetic,frame_id='map',frame_revision='test',
            car_position_map_m=[i*.1 if moving else 0,0,0],uav_position_map_m=[0,0,.5 if kind=='flight' else 0],
            observed_armed=kind=='flight' and i<3,observed_landed=kind!='flight' or i==3,observed_car_stopped=i==3,
            status=dict(phase=phase,car_task_phase=phase,fault=None)))
    (session/'cooperative.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (run/'plan.json').write_text(json.dumps(dict(route_local_forward_left_m=[[3,0],[3,2],[0,2],[0,0]])))
    if closed:(session/'joint_stop.json').write_text(json.dumps(dict(cooperative_complete=complete,car_stop_confirmed=True,errors=[])))
    (run/'analysis.json').write_text('{"result":"fixture"}')
    return session


def test_replay_library_only_closed_real_motion_and_links_originals(tmp_path):
    from drone_console import refresh_replay_library
    base=tmp_path/'raw';library=tmp_path/'回放'
    flight=replay_fixture(base,'test_20261120_155542')
    car=replay_fixture(base,'test_20261120_162135','car',complete=False)
    replay_fixture(base,'static_20261120_162136','static')
    replay_fixture(base,'mock_20261120_162137',synthetic=True)
    replay_fixture(base,'active_20261120_162138',closed=False)
    before={str(p):(p.read_bytes(),p.stat().st_mtime_ns) for p in base.rglob('*') if p.is_file()}
    entries=refresh_replay_library(base,library)
    assert len(entries)==2
    assert {e['result'] for e in entries}=={'流程完成','已就位'}
    assert {Path(e['directory']).parent.name for e in entries}=={'飞行跟随','小车就位'}
    for entry in entries:
        folder=Path(entry['directory'])
        assert (folder/'cooperative.jsonl').is_symlink()
        assert (folder/'cooperative.jsonl').resolve() in (flight/'cooperative.jsonl',car/'cooperative.jsonl')
        assert (folder/'analysis.json').is_file()
        replay=Session(folder);replay.pump(all_available=True)
        assert replay.model.rows==4 and 'coop_car' in replay.model.positions
    assert refresh_replay_library(base,library)==entries  # no duplicate folders
    assert before=={str(p):(p.read_bytes(),p.stat().st_mtime_ns) for p in base.rglob('*') if p.is_file()}


def test_replay_library_preserves_failed_flights_and_recreates_missing_link(tmp_path):
    from drone_console import refresh_replay_library
    base=tmp_path/'raw';library=tmp_path/'回放'
    replay_fixture(base,'failed_20261120_150245',complete=False)
    entries=refresh_replay_library(base,library)
    assert len(entries)==1 and entries[0]['result']=='未完整完成'
    folder=Path(entries[0]['directory']);(folder/'cooperative.jsonl').unlink()
    refresh_replay_library(base,library)
    assert (folder/'cooperative.jsonl').is_file()
    assert (folder/'回放说明.json').is_file()


def comparison_row(at,alignment,position,**meta):
    return dict(at_ns=round(at*1e9),synthetic=False,frame_id='map',frame_revision='test-frame',
        car_position_map_m=[0,0,0],uav_position_map_m=position,uav_received_ns=round(at*1e9),
        observed_armed=True,observed_landed=False,observed_car_stopped=True,
        status=dict(phase='FOLLOW',fault=None),position_comparison=dict(
            reference='uav_body',alignment=alignment,boot_session='test-boot',valid=True,reason=None,**meta))


def compared_model():
    from uav_coord_bridge.target_mapping import make_alignment,map_target_to_controller
    # Nonzero origins, nontrivial heading, and NED vertical sign all matter.
    alignment=make_alignment([2,3,4],[9,-7,-2],.7,-.2)
    model=ConsoleModel()
    model.ingest('cooperative.jsonl',comparison_row(10.,alignment,[2,3,4]))
    xyz=map_target_to_controller(alignment,[2.05+.03,3-.04,4+.02])
    model.ingest('wifi_raw.jsonl',rx(mav.MAVLink_local_position_ned_message(10050,*xyz,1,0,0),10.05))
    model.ingest('cooperative.jsonl',comparison_row(10.1,alignment,[2.1,3,4]))
    return model,alignment


def test_comparison_uses_frozen_transform_body_point_and_receipt_interpolation():
    model,_=compared_model();result=model.position_comparison(10.1)
    assert result['valid']
    assert result['vicon_body_map_m']==pytest.approx([2.05,3,4])
    assert result['delta_m']==pytest.approx([.03,-.04,.02],abs=1e-6)
    assert result['horizontal_m']==pytest.approx(.05,abs=1e-6)
    assert result['total_m']==pytest.approx(.053851648,abs=1e-6)
    assert result['pair_gap_s']==pytest.approx(.1)
    # Showing a wildly different raw marker origin must not change body comparison.
    model.position('vicon',10.1,[99,88,77])
    assert model.position_comparison(10.1)==result


def test_comparison_old_logs_and_unfrozen_alignment_remain_unavailable():
    model=ConsoleModel()
    assert not model.position_comparison(10.)['valid']
    row=comparison_row(10.,None,[0,0,0]);row['position_comparison'].update(valid=False,reason='waiting_for_flight_alignment')
    model.ingest('cooperative.jsonl',row)
    assert '等待本轮' in model.position_comparison(10.)['reason']


def test_comparison_stale_and_unbracketed_data_do_not_show_live_error():
    model,_=compared_model()
    assert not model.position_comparison(12.)['valid']
    model.comparison_vicon.clear();model.comparison_vicon.extend([(10.,(0,0,0)),(10.3,(0,0,0))])
    assert not model.position_comparison(10.3)['valid']
    model.comparison_vicon.clear();model.comparison_vicon.extend([(9.8,(0,0,0)),(10.,(0,0,0))])
    assert not model.position_comparison(10.1)['valid']  # no extrapolation


def test_comparison_reboot_fault_or_new_epoch_cannot_reuse_old_pairs():
    model,alignment=compared_model()
    model.ingest('wifi_raw.jsonl',rx(mav.MAVLink_local_position_ned_message(1,0,0,0,0,0,0),10.2))
    assert '时间倒退' in model.position_comparison(10.2)['reason']
    model,alignment=compared_model()
    row=comparison_row(10.2,alignment,[2.2,3,4]);row['position_comparison'].update(valid=False,reason='ekf_reset')
    model.ingest('cooperative.jsonl',row)
    assert not model.position_comparison(10.2)['valid'] and not model.comparison_px4
    model,alignment=compared_model()
    row=comparison_row(10.2,alignment,[2.2,3,4]);row['position_comparison']['boot_session']='new-boot'
    model.ingest('cooperative.jsonl',row)
    assert not model.position_comparison(10.2)['valid'] and not model.comparison_px4


def test_comparison_rejects_bad_transform_and_does_not_call_mock_data_real():
    model,_=compared_model()
    model.comparison_meta['alignment']['rotation_controller_from_map']=[[1,0,0],[0,1,0],[0,0,1]]
    assert not model.position_comparison(10.1)['valid']
    model,_=compared_model();model.cooperative_synthetic=True
    assert '模拟' in model.position_comparison(10.1)['reason']


def test_vehicle_visuals_have_four_wheels_or_rotors_and_measured_heading():
    from drone_console import display_vehicle_parts,display_part_pose
    import math
    car=display_vehicle_parts(True);uav=display_vehicle_parts(False)
    wheels=[p for p in car if p['shape']=='CYLINDER' and p['size']==(.06,.06,.0306)]
    rotors=[p for p in uav if p['shape']=='CYLINDER' and p['size']==(.11,.11,.004)]
    assert len(wheels)==4 and len(rotors)==4
    part={'xyz':(.1,0.,.02),'rpy':(0.,0.,0.)}
    pos,q=display_part_pose((1.,2.,3.),math.pi/2,part)
    assert all(abs(a-b)<1e-9 for a,b in zip(pos,(1.,2.1,3.02)))
    assert abs(q[2]-math.sqrt(.5))<1e-9 and abs(sum(v*v for v in q)-1)<1e-9


def rx(msg, at=10., *, sender=(1, 1), **kwargs):
    packer=mav.MAVLink(None, srcSystem=sender[0], srcComponent=sender[1])
    return dict(at_ns=int(at*1e9), accepted=True, wire_hex=msg.pack(packer).hex(), **kwargs)


def heartbeat(mode=6, armed=True):
    return mav.MAVLink_heartbeat_message(2, 12, 128 if armed else 0, mode<<16, 4, 3)


def test_cooperative_landing_stage_is_visible_without_a_phase_change():
    model=ConsoleModel()
    row=dict(at_ns=10**9,synthetic=True,frame_id='map',frame_revision='SYNTHETIC',
        car_position_map_m=[0,0,0],uav_position_map_m=[0,0,.3],
        observed_armed=True,observed_landed=False,observed_car_stopped=True,
        status=dict(phase='LAND',fault=None,landing_waiting_for='touchdown'))
    model.ingest('cooperative.jsonl',row)
    assert '带动力降落' in model.events[-1]['text']
    row.update(at_ns=2*10**9,observed_landed=True)
    row['status']['landing_waiting_for']='px4_auto_disarm'
    model.ingest('cooperative.jsonl',row)
    assert '等待自动锁定停桨' in model.events[-1]['text']
    assert model.fields['armed'] is True and model.status['phase']=='LAND'


def test_car_motion_display_does_not_invent_a_drone_trajectory_or_flight_status():
    model=ConsoleModel()
    row=dict(at_ns=10**9,synthetic=True,frame_id='map',frame_revision='CAR_TEST',
        car_position_map_m=[.1,.2,.1],status=dict(phase='WAIT_TAKEOFF',fault=None,car_stopped=True))
    model.ingest('car_motion.jsonl',row)
    assert model.positions['coop_car']==(.1,.2,.1)
    assert 'coop_uav' not in model.positions and 'armed' not in model.fields
    assert model.fields['car_stopped'] and model.cooperative_synthetic
    assert '等待无人机离台' in model.events[-1]['text']


def pose(at=10., **kwargs):
    f=dict(subject='MircoAir',segment='MircoAir',frame_id='map',sdk_ok=True,
           occluded=False,pose_published=True,queue_dropped=0,world_pose=[1,2,3,0,0,0,1])
    f.update(kwargs)
    return dict(at_ns=int(at*1e9),frame=f)


def test_rx_source_crc_and_direction_are_not_relaxed():
    m=ConsoleModel()
    m.ingest('wifi_raw.jsonl',rx(heartbeat(),sender=(2,1)))
    m.ingest('traffic.jsonl',rx(heartbeat(),direction='tx_attempt'))
    broken=rx(heartbeat());broken['wire_hex']=broken['wire_hex'][:-2]+'aa'
    m.ingest('wifi_raw.jsonl',broken)
    assert 'armed' not in m.fields
    m.ingest('wifi_raw.jsonl',rx(heartbeat(),at=11))
    assert m.fields['armed'] is True and m.fields['mode']=='Offboard'


def test_positions_keep_frames_separate_and_ignore_occlusion():
    m=ConsoleModel();m.ingest('vicon_frames.jsonl',pose())
    m.ingest('wifi_raw.jsonl',rx(mav.MAVLink_local_position_ned_message(1000,10,20,-4,0,0,0)))
    assert m.positions['vicon']==(1,2,3)
    assert m.positions['px4']==(20,10,4)
    m.ingest('vicon_frames.jsonl',pose(11,occluded=True,world_pose=[9,9,9,0,0,0,1]))
    assert m.times['vicon']==10 and not m.vicon_ok
    assert m.age('vicon',13)==3


def test_old_vicon_at_field_is_nanoseconds_not_seconds():
    row=pose();row['at']=row.pop('at_ns')
    assert stamp(row)==10
    assert stamp(dict(at=10,direction='rx_datagram'))==10


def test_fc_reboot_starts_a_separate_local_trace():
    m=ConsoleModel()
    for at,boot,z in ((10,1000,-1),(11,10,-2)):
        m.ingest('wifi_raw.jsonl',rx(mav.MAVLink_local_position_ned_message(boot,0,0,z,0,0,0),at))
    assert len(m.traces['px4'])==1 and m.origins['px4'][2]==2
    assert any('时间倒退' in e['text'] for e in m.events)


def sys_status(percent):
    return mav.MAVLink_sys_status_message(0,0,0,0,22000,0,percent,0,0,0,0,0,0)


def battery(percent, ident=0):
    return mav.MAVLink_battery_status_message(ident,0,0,0,[22000]+[65535]*9,0,0,0,percent)


def test_battery_zero_priority_unknown_and_history_age():
    m=ConsoleModel()
    m.ingest('wifi_raw.jsonl',rx(sys_status(45),10))
    m.ingest('wifi_raw.jsonl',rx(battery(29),11))
    m.ingest('wifi_raw.jsonl',rx(battery(99,ident=1),12))
    assert m.battery(12)['percent']==29
    assert m.battery(100) is None  # Old evidence is never called current.
    m.ingest('wifi_raw.jsonl',rx(battery(-1),13))
    assert m.battery(14)['percent']==45
    m.ingest('wifi_raw.jsonl',rx(sys_status(-1),15))
    assert m.battery(16) is None


def test_partial_lines_and_replacement_do_not_mix_sessions(tmp_path):
    p=tmp_path/'log.jsonl';p.write_text('{"at":1')
    t=LogTail(p);assert t.peek() is None
    with p.open('a') as out:out.write('}\n')
    assert t.pop()==(1.,dict(at=1))
    p.write_text('')
    assert t.peek() is None and t.changed


def test_replay_is_timed_and_tx_records_do_not_create_telemetry(tmp_path):
    rows=[rx(heartbeat(3,False),10),rx(heartbeat(6,True),12)]
    (tmp_path/'wifi_raw.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    s=Session(tmp_path);s.pump()
    assert s.model.fields['armed'] is False and not s.eof
    s.pump(dt=1);assert s.model.fields['armed'] is False
    s.pump(dt=1);assert s.model.fields['armed'] is True and s.eof


def test_follow_reads_appends_and_reports_no_age_for_future_boot(tmp_path):
    p=tmp_path/'wifi_raw.jsonl';p.write_text(json.dumps(rx(heartbeat(3,False),10))+'\n')
    s=Session(tmp_path,follow=True);s.pump()
    with p.open('a') as out:out.write(json.dumps(rx(heartbeat(6,True),12))+'\n')
    s.pump();assert s.model.fields['armed'] is True
    assert s.model.age('mode',1) is None


def test_host_fault_details_preserved_but_takeoff_token_not_shown():
    m=ConsoleModel()
    m.ingest('console_events.jsonl',dict(at_ns=10**10,text=json.dumps(dict(confirmation_text='TAKEOFF abc'))))
    assert not m.events
    fault=dict(phase='LAND',failure='send_pose_stale',missing=['pose'],failure_snapshot={'age':.17})
    m.ingest('console_events.jsonl',dict(at_ns=11*10**9,text=json.dumps(fault)))
    assert m.status==fault and '定位数据' in m.events[-1]['text']


def test_statustext_chunk_assembly_and_rejected_ack():
    m=ConsoleModel()
    for seq,text in enumerate(('a'*50,'ending')):
        msg=mav.MAVLink_statustext_message(3,text.encode(),id=42,chunk_seq=seq)
        m.ingest('wifi_raw.jsonl',rx(msg,10+seq))
    assert m.events[-1]['text']=='a'*50+'ending'
    m.ingest('wifi_raw.jsonl',rx(mav.MAVLink_command_ack_message(400,2),12))
    assert '400' in m.events[-1]['text'] and m.events[-1]['level']=='ERROR'


def test_async_console_mirror_drains_and_does_not_own_terminal():
    stream=io.StringIO();writer=LatestStatusPrinter(stream,capacity=128)
    for i in range(50):writer.publish(str(i))
    writer.close(drain=True);writer.worker.join(1)
    assert stream.getvalue().splitlines()==[str(i) for i in range(50)]
    assert not stream.closed


def test_rviz_is_local_and_does_not_mutate_parent_network(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','94')
    import os
    env=rviz_environment()
    assert env['ROS_DOMAIN_ID']=='186' and env['ROS_AUTOMATIC_DISCOVERY_RANGE']=='LOCALHOST'
    assert os.environ['ROS_DOMAIN_ID']=='94'


def test_large_malformed_prefix_does_not_hide_later_valid_record(tmp_path):
    p=tmp_path/'wifi_raw.jsonl'
    p.write_text('invalid\nnull\n[]\n'*20000+json.dumps(rx(heartbeat(3,False),10))+'\n')
    s=Session(tmp_path)
    for _ in range(20):
        s.pump(all_available=True)
        if s.eof:break
    assert s.eof and s.model.fields['armed'] is False


def test_formal_entry_mirrors_existing_events_without_new_controller_actions(tmp_path,monkeypatch):
    import vicon_fixed_point_entry as entry
    from types import SimpleNamespace
    calls=[]
    controller=object()
    monkeypatch.setattr(entry,'open_pocket_session',lambda **kwargs:controller)
    monkeypatch.setattr(entry,'terminal_reader',lambda fd:lambda:None)
    monkeypatch.setattr(sys,'stdin',SimpleNamespace(fileno=lambda:0))
    def serve(c,read_line,*,emit):
        assert c is controller
        calls.append('serve')
        emit(json.dumps(dict(phase='LAND',failure='send_pose_stale')))
    monkeypatch.setattr(entry,'serve_console',serve)
    args=['--run','--flight-console','--profile','synthetic.json','--output',str(tmp_path)]
    assert entry.main(args)==0 and calls==['serve']
    rows=[json.loads(line) for line in (tmp_path/'console_events.jsonl').read_text().splitlines()]
    assert len(rows)==1 and json.loads(rows[0]['text'])['failure']=='send_pose_stale'
    # File already exists: display logging failure must not change control service.
    assert entry.main(args)==0 and calls==['serve','serve']


def test_vicon_measured_speed_not_command_and_repeated_samples_do_not_refresh():
    import pytest
    m=ConsoleModel()
    for i in range(21):
        at=10+i*.05
        m.measured_motion('coop_car',at,(.25*(at-10),0,0))
        m.measured_motion('coop_uav',at,(.3*(at-10),.4*(at-10),-.08*(at-10)))
    assert m.motion_value('coop_car',11)['horizontal']==pytest.approx(.25)
    assert m.motion_value('coop_uav',11)['horizontal']==pytest.approx(.5)
    assert m.motion_value('coop_uav',11)['vertical']==pytest.approx(-.08)
    m.measured_motion('coop_car',11,(99,99,99))
    assert m.positions['coop_car']==pytest.approx((.25,0,0))
    assert m.motion_value('coop_car',12.1) is None
    m.measured_motion('coop_car',13,(4,0,0))
    assert m.motion_value('coop_car',13) is None  # no spike across a gap


def test_joint_position_uses_source_receipt_time_not_repeated_writer_time():
    m=ConsoleModel()
    row=dict(at_ns=10**9,synthetic=True,frame_id='map',frame_revision='TEST',
        car_received_ns=10**9,uav_received_ns=10**9,
        car_position_map_m=[0,0,0],uav_position_map_m=[0,0,.3],
        observed_armed=False,observed_landed=True,observed_car_stopped=True,
        status=dict(phase='WAIT_START',fault=None))
    m.ingest('cooperative.jsonl',row)
    row['at_ns']=4*10**9
    m.ingest('cooperative.jsonl',row)
    assert m.age('coop_car',4)==3
    assert len(m.velocity_samples['coop_car'])==1


def test_live_discovery_ignores_history_synthetic_future_and_partial_line(tmp_path):
    import os
    def write(name,at,synthetic=False,suffix=''):
        p=tmp_path/name/'session';p.mkdir(parents=True)
        (p/'cooperative.jsonl').write_text(json.dumps(dict(at_ns=int(at*1e9),synthetic=synthetic))+'\n'+suffix)
        return p
    write('old',10)
    write('mock',100,True)
    write('future',200)
    good=write('current',99,suffix='{"at_ns":')
    assert active_joint_directory(tmp_path,now=100)==good
    assert active_joint_directory(tmp_path,now=105) is None
    os.utime(good/'cooperative.jsonl',(time.time()-20,time.time()-20))
    assert active_joint_directory(tmp_path,now=100) is None


def test_circle_replay_has_own_label(tmp_path):
    from drone_console import refresh_replay_library
    session=replay_fixture(tmp_path/'raw','circle_20261123_150000')
    plan=session.parent/'plan.json';plan.write_text(json.dumps({'circle_route':{'diameter_m':1.5}}))
    entries=refresh_replay_library(tmp_path/'raw',tmp_path/'library')
    assert len(entries)==1
    assert '圆周跟随' in str(entries[0])
