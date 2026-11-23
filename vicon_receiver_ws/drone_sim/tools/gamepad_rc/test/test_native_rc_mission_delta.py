"""Native RC mission integration against the existing synthetic FC, no hardware."""
import json
import socket
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pymavlink.dialects.v20 import common as m
from test_continuous_execution import TransportPeer, FakePort
from fixed_profile_fixtures import approved_profile
from flight_start_confirmation import build_guarded_session
from mission_audit_collector import COMMANDS
from mavlink_crc_stream import CrcResyncParser
import ekf_static_profile
import vicon_fixed_point_entry as entry


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    monkeypatch.setattr(socket,'socket',lambda *a,**k:pytest.fail('No network in offline test'))


class NativePeer(TransportPeer):
    def __init__(self,tmp_path,authorized=True,*,timestamp_mode='fc_receive_time',initial_trial=False,horizontal_motion=None):
        profile=approved_profile()
        profile['timestamp_mode']=timestamp_mode
        if timestamp_mode=='local_ros_timesync':profile['timestamp_evidence']='SYNTHETIC test only'
        profile['parameters']['COM_RC_IN_MODE']['value']=0
        profile['persistence']['configuration_sha256']=ekf_static_profile.configuration_digest(profile)
        super().__init__(fixed_configuration=True,fixed_profile=profile)
        if horizontal_motion is not None:profile['horizontal_motion']=horizontal_motion
        self.intervals.update({32:0,230:0,245:0})
        if initial_trial:
            from fixed_profile_fixtures import initial_trial_profile
            profile=initial_trial_profile(profile)
        self.params.update(COM_RC_OVERRIDE=3,COM_RC_STICK_OV=30.,RC_MAP_KILL_SW=7,RC_MAP_FLTMODE=5)
        self.port=FakePort(); self.rc_lost=False; self.kill=False; self.sticks=False; self.manual=False
        self.force_failsafe=False;self.throttle=-1.;self.rc_source=9
        self.mode_slot=2;self.pilot_land=False
        self.switch_publication=None
        self.controller=build_guarded_session(revision='SYNTHETIC-ONLY-rigid-revision',origin=[0,0,0],
            orientation=[0,0,0,1],port=self.port,poll_gamepad=None,
            poll_pose=lambda:self.input_args['pose'],now_ros_ns=lambda:self.input_args['now_ros_ns'],
            check_port_identity=lambda:None,usb_present=lambda:False,origin_monotonic=0,
            recovery_path=tmp_path/'run'/'flight_recovery.json',fixed_profile=profile,pilot_source='native_rc')
        self.runtime=self.controller.runtime; self.io=self.controller.io
        if authorized:
            self.io.authorizer=lambda *args:True  # In-memory fixture only.
            self.runtime.sync_rates_enabled=True

    def _reply_text(self,key):
        if key=='visual_snapshot':
            stamp=int((self.now+self.boot_offset)*1e6);v=self.last_pose
            return ('nsh> '+COMMANDS[key].decode()+'TOPIC: vehicle_visual_odometry\n'
                f' timestamp: {stamp}\n timestamp_sample: {stamp}\n pose_frame: 1\n'
                ' velocity_frame: 0\n reset_counter: 0\n quality: 0\n'
                f' position: [{v.x}, {v.y}, {v.z}]\n q: [1, 0, 0, 0]\nnsh> ')
        if not key.startswith('native_') and key!='actuator':return super()._reply_text(key)
        topic=key[7:] if key.startswith('native_') else 'actuator_armed'
        stamp=int((self.now+self.boot_offset)*1e6)
        if topic=='manual_control_switches':
            # rc_update publishes immediately on a change, otherwise at 1 Hz.
            # Never attach changed switch values to the previous timestamp.
            values=(self.mode_slot,self.kill)
            previous=self.switch_publication
            if previous is None or previous[1]!=values or stamp-previous[0]>=1_000_000:
                self.switch_publication=(stamp,values)
            stamp=self.switch_publication[0]
        if topic=='failsafe_flags' and not self.rc_lost:stamp=stamp//500_000*500_000
        fields=dict(timestamp=stamp)
        if topic=='input_rc':fields.update(timestamp_last_signal=stamp,input_source=self.rc_source,channel_count=8,
            rssi=100,rc_lost=self.rc_lost,rc_failsafe=self.rc_lost)
        elif topic=='manual_control_setpoint':fields.update(timestamp_sample=stamp,data_source=1,
            valid=not self.rc_lost,roll=0.,pitch=0.,yaw=0.,throttle=self.throttle,sticks_moving=self.sticks)
        elif topic=='manual_control_switches':fields.update(timestamp_sample=stamp,mode_slot=self.mode_slot,
            arm_switch=3,offboard_switch=3,kill_switch=1 if self.kill else 3)
        elif topic=='failsafe_flags':
            import control_readonly_policy
            fields.update(dict.fromkeys(control_readonly_policy.FAILSAFE_FLAGS,False))
            fields.update(manual_control_signal_lost=self.rc_lost)
        else:fields.update(armed=self.armed,lockdown=self.locked,manual_lockdown=self.kill,force_failsafe=self.force_failsafe)
        return ('nsh> '+COMMANDS[key].decode()+'TOPIC: '+topic+'\n'+
                ''.join(f' {k}: {v}\n' for k,v in fields.items())+'\nnsh> ')

    def run_owner_turn(self,now,**kwargs):
        for key in ('pad_generation','pad_present','pad_event','home_events'):kwargs.pop(key,None)
        mode=3 if self.manual or not self.offboard else 6
        kwargs['rx']+=self.pack(self.encoder.heartbeat_encode(2,m.MAV_AUTOPILOT_PX4,
            128 if self.armed else 0,((6<<24)|(4<<16)) if self.pilot_land else mode<<16,3))
        return super().run_owner_turn(now,**kwargs)

    def prepare(self):
        for _ in range(2500):
            self.turn()
            if self.runtime.failure or self.runtime.core.phase=='WAIT_EXPLICIT_START':break
        return self.runtime.status()


def prepared(tmp_path):
    p=NativePeer(tmp_path); result=p.prepare()
    assert not result['failure'],(result['failure'],result.get('failure_context'),result['coordinator'])
    assert p.runtime.core.phase=='WAIT_EXPLICIT_START',result
    return p


def start(p):
    preview=p.controller.start.create()
    p.controller.start.confirm(preview['confirmation_text'])


def recorded_cadence_peer(tmp_path):
    """Recorded nominal rates with synthetic content, NOT a hardware replay.

    Exercise the real review predicates, not the usual always-true writer.
    No approval files or hardware evidence are created by this fixture.
    """
    import re
    from flight_execution_review import ReviewedExecutionGrant
    from test_shared_pilot_entry import reviewed_fake_grant
    p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync',initial_trial=True)
    pack=p.pack;last={};reply=p._reply_text;published={}
    def paced(msg):
        mid=msg.get_msgId()
        period={0:1.,30:.067,31:.1}.get(mid)
        if mid in (32,245):period=p.intervals.get(mid,0)/1e6 or 1.
        if period is not None:
            if p.now-last.get(mid,-1e9)<period-1e-8:return b''
            last[mid]=p.now
            if mid==0:
                mode=((6<<24)|(4<<16)) if p.pilot_land else (3 if p.manual or not p.offboard else 6)<<16
                msg=p.encoder.heartbeat_encode(2,m.MAV_AUTOPILOT_PX4,128 if p.armed else 0,mode,3)
        return pack(msg)
    def periodic_text(key):
        text=reply(key)
        if key in ('commander','actuator','native_actuator_armed'):
            values=(p.armed,p.locked,p.kill,p.offboard)
            prev=published.get(key)
            if prev is None or prev[1]!=values or p.now-prev[0]>=.5-1e-8:
                published[key]=(p.now,values,int((p.now+p.boot_offset)*1e6))
            text=re.sub(r' timestamp: \d+',f' timestamp: {published[key][2]}',text)
        return text
    p.pack=paced;p._reply_text=periodic_text
    g=reviewed_fake_grant(ReviewedExecutionGrant,p.runtime)
    g.record.update(issued_unix=100.,expires_unix=280.)
    g.original_record=json.dumps(g.record,sort_keys=True,allow_nan=False)
    g.wall=lambda:100.+p.now;g.start_window=(0.,180.)
    p.io.authorizer=g
    p.controller.review_slots.execution=g
    p.controller.start.native_start_check=p.controller.require_native_start
    return p,g


@pytest.mark.parametrize('delay',[0.,.051,.105,.17])
def test_recorded_rates_and_real_authorizer_account_for_journal_delay(tmp_path,delay):
    p,g=recorded_cadence_peer(tmp_path);r=p.runtime
    assert not p.prepare()['failure'],p.controller.status()
    before=p.io.journal.before;injected=None
    def slow_journal(runtime,record,msg):
        nonlocal injected
        result=before(runtime,record,msg)
        if injected is None and r.core.phase=='ASCEND' and msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED':
            injected=r.now
        return result
    p.io.journal.before=slow_journal
    g.clock=lambda:r.now+(delay if injected==r.now else 0.)
    start(p);time_advanced=False
    for _ in range(5000):
        p.turn()
        if injected is not None and not time_advanced:
            p.now=round(p.now+delay,9);time_advanced=True
        if p.controller.done:break
    assert injected is not None and p.controller.done,p.controller.status()
    assert p.controller.journal_finalized
    if delay<=.105:
        assert r.core.fault is None and any(row['phase']=='HOVER' for row in p.state_rows),p.controller.status()
    else:
        assert r.core.fault and p.controller.failure_snapshot
        rejected=p.controller.failure_snapshot['authorization_rejection']
        assert rejected['elapsed_since_turn_s']==pytest.approx(delay)
        assert rejected['ages_s']['source_age']>.15
    assert sum(msg.get_type()=='COMMAND_LONG' and msg.command==400 for _,msg in p.calls)==1


def test_long_preparation_renews_unused_token_without_spending_flight_time(tmp_path):
    p,g=recorded_cadence_peer(tmp_path);r=p.runtime
    assert not p.prepare()['failure']
    old=p.controller.start.create();origin=r.core.origin
    until=p.now+125.
    while p.now<until:
        p.turn()
        assert not r.failure and not p.controller.done,p.controller.status()
    assert p.controller.started_at is None and not r.core.arm_attempted
    with pytest.raises(PermissionError):p.controller.start.confirm(old['confirmation_text'])
    renewed=p.controller.start.create()
    assert renewed['confirmation_text']!=old['confirmation_text'] and r.core.origin is origin
    assert p.controller.start.create()==renewed
    with pytest.raises(PermissionError):p.controller.start.confirm(old['confirmation_text'])
    p.controller.start.confirm(renewed['confirmation_text'])
    for _ in range(5000):
        p.turn()
        if p.controller.done:break
    assert p.controller.done and r.core.fault is None,p.controller.status()
    assert p.controller.started_at==r.start_requested_at and p.controller.journal_finalized
    with pytest.raises(PermissionError):p.controller.start.create()
    with pytest.raises(PermissionError):p.controller.start.confirm(renewed['confirmation_text'])


def test_review_start_expiry_does_not_interrupt_an_already_started_flight(tmp_path):
    p,g=recorded_cadence_peer(tmp_path);r=p.runtime
    assert not p.prepare()['failure']
    g.record['expires_unix']=100.+p.now+.5
    g.original_record=json.dumps(g.record,sort_keys=True,allow_nan=False)
    g.start_window=(0.,p.now+.5)
    start(p)
    for _ in range(5000):
        p.turn()
        if p.controller.done:break
    assert not g.start_available()
    assert p.controller.done and r.core.fault is None and p.controller.journal_finalized,p.controller.status()


def test_native_takeover_confirmation_matches_one_hz_heartbeat(tmp_path):
    p=prepared(tmp_path);airborne(p);r=p.runtime
    p.manual=True;p.sticks=True;p.turn()
    assert r.core.pilot_control
    for heartbeat_age,confirmed in ((.49,True),(.51,True),(.99,True),(1.99,True),(2.,False)):
        r.inputs.hb['at']=r.now-heartbeat_age
        assert r.native_takeover_confirmed(r.now) is confirmed
        assert p.controller.status()['intervention_required'] is (not confirmed)
        assert r.core.pilot_control  # Expiry never resumes autonomous targets.


def test_retained_offboard_is_explained_before_new_start_without_mode_commands(tmp_path):
    p,g=recorded_cadence_peer(tmp_path)
    assert not p.prepare()['failure']
    r=p.runtime;r.inputs.hb.update(custom_mode=6<<16,offboard=True,at=r.now)
    count=len(p.port.calls)
    assert 'Offboard' in p.controller.status()['operator_action']
    with pytest.raises(PermissionError,match='Position'):p.controller.start.create()
    assert len(p.port.calls)==count and not r.core.arm_attempted


def test_expired_start_review_rejects_token_without_arming(tmp_path):
    p,g=recorded_cadence_peer(tmp_path)
    assert not p.prepare()['failure']
    token=p.controller.start.create();count=len(p.port.calls)
    g.wall=lambda:281.
    with pytest.raises(PermissionError,match='execution review'):
        p.controller.start.confirm(token['confirmation_text'])
    assert not p.runtime.core.arm_attempted and not p.controller.start.used
    assert len(p.port.calls)==count


def test_native_input_is_rechecked_at_actual_send_time(tmp_path):
    p,g=recorded_cadence_peer(tmp_path);r=p.runtime
    assert not p.prepare()['failure']
    r.inputs.native_rc.topics['input_rc']['received_at']=r.now-1.99
    assert r.context().pilot_current()
    g.clock=lambda:r.now+.051
    msg=r.encoder.command_long_encode(1,1,400,0,1,0,0,0,0,0,0)
    assert not g(r,{'at':r.now},msg)
    assert g.last_rejection['reason']=='send_pose_attitude_or_pilot_invalid'


def test_actual_flight_duration_timeout_keeps_landing_owner(tmp_path,monkeypatch):
    from flight_start_confirmation import FlightRunController
    p=prepared(tmp_path);airborne(p);r=p.runtime
    limit=r.now-r.start_requested_at+.05
    monkeypatch.setattr(FlightRunController,'task_seconds',classmethod(lambda cls,runtime:limit))
    for _ in range(1000):
        p.turn()
        if p.controller.done:break
    assert p.controller.done and p.controller.journal_finalized,p.controller.status()
    assert r.core.fault=='flight_duration_timeout'
    assert p.controller.failure_snapshot['reason']=='flight_duration_timeout'
    assert sum(msg.get_type()=='COMMAND_LONG' and msg.command==400 for _,msg in p.calls)==1


def airborne(p):
    start(p)
    for _ in range(1500):
        p.turn()
        if p.runtime.core.phase=='ASCEND':return
        assert not p.runtime.failure,p.runtime.status()
    pytest.fail(p.runtime.status())


def test_native_prepare_flight_landing_has_no_home_or_override(tmp_path):
    p=prepared(tmp_path);r=p.runtime
    assert not r.inputs.pad_present and not r.inputs.home_proven and r.inputs.home_press is None
    assert r.home_attempts==0 and not p.locked and p.last_pose is not None
    assert not r.core.arm_attempted
    start(p)
    for _ in range(5000):
        p.turn()
        if p.controller.done:break
    assert p.controller.done and r.core.phase=='COMPLETE',r.status()
    assert p.controller.journal_finalized
    assert not any(msg.get_type() in ('RC_CHANNELS_OVERRIDE','MANUAL_CONTROL','PARAM_SET')
        or msg.get_type()=='COMMAND_LONG' and msg.command==185 for _,msg in p.calls)
    assert {msg.command for _,msg in p.calls if msg.get_type()=='COMMAND_LONG'}=={512,176,400,21}


@pytest.mark.parametrize('reply_age,source_age,accepted',[
    (1.015558596,1.5,True),(1.5,2.,True),(1.99,2.49,True),
    (2.01,2.49,False),(.1,2.51,False)])
def test_commander_reply_tolerance_preserves_source_expiry(tmp_path,reply_age,source_age,accepted):
    # The recorded 11:20 reply gap is the first case. Other telemetry stays
    # current; neither receiving a heartbeat nor widening reply age restamps it.
    p=prepared(tmp_path);inputs=p.runtime.inputs
    held=inputs.critical_status['commander']
    held['at']=p.now-reply_age
    stamp=int(inputs._boot_us(p.now)-source_age*1e6)
    held['value']['timestamp']=stamp
    observation,missing=inputs.build(p.now)
    assert (observation is not None) is accepted,missing
    if accepted:assert observation.link_ok()
    else:assert 'current_commander_status_missing_or_stale' in missing
    assert held['value']['timestamp']==stamp


@pytest.mark.parametrize('reply_gap_s',[1.015558596,2.05])
def test_offboard_wait_tolerates_delayed_commander_but_not_missing_state(tmp_path,reply_gap_s):
    # Inject one Commander delivery gap, using the measured duration but
    # synthetic FC messages. This is not a hardware replay or flight result.
    p=prepared(tmp_path);r=p.runtime;observe=r.inputs.observe_status
    held_at=None;held_beyond_old_limit=False;released=False
    def delayed_status(values,*,requested_at,now):
        nonlocal held_at,held_beyond_old_limit,released
        if r.core.phase=='WAIT_OFFBOARD' and 'commander' in values and not released:
            if held_at is None:held_at=r.inputs.critical_status['commander']['at']
            if now-held_at<reply_gap_s:
                values={k:v for k,v in values.items() if k!='commander'}
            else:released=True
        return observe(values,requested_at=requested_at,now=now)
    r.inputs.observe_status=delayed_status
    start(p)
    for _ in range(5000):
        p.turn()
        if held_at is not None and not released and p.now-held_at>1.:
            held_beyond_old_limit=True
            assert not r.core.arm_attempted
        if p.controller.done or r.failure:break
    assert held_beyond_old_limit,r.status()
    if reply_gap_s<2.:
        assert released and p.controller.done and r.core.phase=='COMPLETE',r.status()
        assert r.core.fault is None and p.controller.journal_finalized
    else:
        assert r.core.phase=='BLOCKED' and not r.core.arm_attempted,r.status()
        assert r.core.fault=='fresh_navigation_input_or_output_enable_lost'


@pytest.mark.parametrize('commander_delay_s',[.81,1.1])
def test_arm_transition_waits_for_commander_then_completes(tmp_path,commander_delay_s):
    # 2026-11-09 first flight: ARM at 80.1567, old Commander at 80.1353,
    # armed actuator at 80.4644, new Commander at 80.9647; abort at +.5045.
    # Reuse the observed ~.81s command-to-Commander lag with synthetic state,
    # plus a second bounded lag. This is not hardware replay or flight proof.
    import re
    p=prepared(tmp_path);r=p.runtime;reply=p._reply_text;held_stamp=None
    def delayed_commander(key):
        nonlocal held_stamp
        text=reply(key)
        if key=='commander' and r.core.phase=='WAIT_ARM' and r.core.pending:
            sent=r.core.pending['sent_at']
            if p.now-sent<commander_delay_s:
                if held_stamp is None:
                    held_stamp=r.inputs.critical_status['commander']['value']['timestamp']
                text=text.replace('arming_state: 2','arming_state: 1')
                text=re.sub(r' timestamp: \d+',f' timestamp: {held_stamp}',text)
        return text
    p._reply_text=delayed_commander
    start(p);held_past_half_second=False;ascending=False
    for _ in range(5000):
        p.turn()
        if r.core.phase=='WAIT_ARM' and r.core.pending:
            elapsed=p.now-r.core.pending['sent_at']
            if .5<elapsed<commander_delay_s:
                held_past_half_second=True
                assert r.core.ascend_at is None and r.core.fault is None
                assert p.target.z==pytest.approx(r.core.origin.position_ned[2])
        if r.core.phase=='ASCEND':
            ascending=True
            assert r.inputs.actuator['armed']
            assert r.inputs.critical_status['commander']['value']['arming_state']==2
        if p.controller.done:break
    assert held_past_half_second and ascending,r.status()
    assert r.core.phase=='COMPLETE' and r.core.fault is None,r.status()
    assert p.controller.journal_finalized
    assert sum(msg.get_type()=='COMMAND_LONG' and msg.command==400 for _,msg in p.calls)==1


@pytest.mark.parametrize('status_age_s',[.525,1.016,1.99])
def test_ascending_preview_uses_same_native_status_budget(tmp_path,status_age_s):
    # The 11:38 hardware attempt passed the outer gate, then the preview's
    # hidden .5s status cutoff aborted ascent. Model that accepted diagnostic
    # age while retaining current pose/local/actuator evidence in the fixture.
    from dataclasses import replace
    p=prepared(tmp_path);r=p.runtime;build=r.inputs.build;injected=False
    def aged_status(now):
        nonlocal injected
        o,missing=build(now)
        if o is not None and r.core.phase in ('ASCEND','HOVER'):
            o=replace(o,status_age_s=status_age_s);injected=True
        return o,missing
    r.inputs.build=aged_status
    start(p)
    for _ in range(5000):
        p.turn()
        if p.controller.done:break
    assert injected
    assert p.controller.done and r.core.phase=='COMPLETE' and r.core.fault is None,r.status()
    assert p.controller.journal_finalized


@pytest.mark.parametrize('interrupt',['timeout','kill','pose_loss'])
def test_arm_transition_wait_does_not_hide_timeout_or_live_fault(tmp_path,interrupt):
    p=prepared(tmp_path);r=p.runtime;reply=p._reply_text
    def disagree(key):
        text=reply(key)
        if key=='commander' and r.core.phase=='WAIT_ARM':
            # Fresh but persistently inconsistent reports must still time out.
            text=text.replace('arming_state: 2','arming_state: 1')
        return text
    p._reply_text=disagree;start(p);sent=None;injected=False
    for _ in range(1500):
        p.turn(with_pose=not(injected and interrupt=='pose_loss'))
        if r.core.phase=='WAIT_ARM':
            sent=r.core.pending['sent_at']
            if p.now-sent>.65:
                injected=True
                if interrupt=='kill':p.kill=True
            assert r.core.ascend_at is None
        elif sent is not None:break
    assert injected and r.core.ascend_at is None
    if interrupt=='timeout':
        assert r.core.phase=='LAND' and r.core.fault=='command_state_confirmation_timeout'
        assert p.now-sent>=r.core.command_timeout
    elif interrupt=='kill':
        assert r.core.phase=='STOP_LATCHED' and r.core.stop_latched
    else:
        assert r.core.phase=='LAND' and r.core.fault=='fresh_navigation_input_or_output_enable_lost'
        assert p.now-sent<r.core.command_timeout
    assert sum(msg.get_type()=='COMMAND_LONG' and msg.command==400 for _,msg in p.calls)==1


@pytest.mark.parametrize('original',[0,100000,1000000])
def test_wifi_sync_rate_native_flight_and_exact_restore(tmp_path,original):
    p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync')
    p.intervals[111]=original
    assert not p.prepare()['failure']
    assert p.runtime.core.phase=='WAIT_EXPLICIT_START'
    assert p.intervals[111]==100000 and p.runtime.fixed_sync_reply_count>0
    start(p)
    for _ in range(5000):
        p.turn()
        if p.controller.done:break
    assert p.controller.done and p.runtime.core.phase=='COMPLETE',p.controller.status()
    assert p.intervals[111]==original
    assert p.runtime.sync_rates.summary()['restoration_verified']
    assert p.controller.journal_finalized
    journal=json.loads((tmp_path/'run'/'rates_recovery.json').read_text())
    assert not journal['needs_restore'] and journal['verified_closed']
    assert not any(msg.get_type() in ('PARAM_SET','RC_CHANNELS_OVERRIDE','MANUAL_CONTROL')
        or msg.get_type()=='COMMAND_LONG' and msg.command==185 for _,msg in p.calls)
    if original==100000:
        assert not any(msg.get_type()=='COMMAND_LONG' and msg.command==511 and msg.param1==111 for _,msg in p.calls)


def test_wifi_sync_waits_for_bootstrap_and_cancel_has_no_rate_writes(tmp_path):
    p=NativePeer(tmp_path,authorized=False,timestamp_mode='local_ros_timesync')
    for _ in range(50):p.turn()
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command==511 for _,msg in p.calls)
    p.controller.cancel()
    for _ in range(10):
        p.turn()
        if p.controller.done:break
    assert p.controller.done and p.intervals[111]==0
    assert not p.runtime.core.arm_attempted


def test_wifi_sync_setup_partial_write_is_journaled_then_restored_without_arm(tmp_path):
    p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync')
    write=p.port.write;failed=False
    def uncertain_setup(wire):
        nonlocal failed
        msg=CrcResyncParser().parse_buffer(wire)[0]
        if msg.get_type()=='COMMAND_LONG' and msg.command==511 and msg.param2==100000 and not failed:
            record=json.loads((tmp_path/'run'/'rates_recovery.json').read_text())
            assert record['needs_restore'] and record['rates']['original_intervals_us']=={'32':0,'230':0,'245':0,'111':0}
            failed=True
            return 1
        return write(wire)
    p.port.write=uncertain_setup
    for _ in range(700):
        p.turn()
        if p.controller.done:break
    assert failed and p.controller.done,p.controller.status()
    assert not p.runtime.core.arm_attempted and p.intervals[111]==0
    assert p.runtime.sync_rates.summary()['restoration_verified']
    assert not p.controller.journal_finalized  # Uncertain delivery stays explicit.


@pytest.mark.parametrize('restore_failure',[False,True])
def test_wifi_sync_restore_survives_pose_and_review_loss_without_claiming_failed_restore(tmp_path,restore_failure):
    p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync')
    assert not p.prepare()['failure']
    p.io.authorizer=lambda *args:False  # Ground review no longer usable.
    with pytest.raises(PermissionError,match='restoration pending'):p.controller.close()
    if restore_failure:
        original_write=p.port.write
        def write(wire):
            msg=CrcResyncParser().parse_buffer(wire)[0]
            if msg.get_type()=='COMMAND_LONG' and msg.command==511 and msg.param2==-1:
                # Bytes possibly escaped, no confirmed delivery or auto retry.
                return 1
            return original_write(wire)
        p.port.write=write
    p.controller.cancel()
    for _ in range(700):
        p.turn(with_pose=False)
        if p.controller.done:break
    assert p.controller.done,p.controller.status()
    journal=json.loads((tmp_path/'run'/'rates_recovery.json').read_text())
    assert journal['needs_restore']==restore_failure
    assert p.runtime.sync_rates.summary()['restoration_verified'] is not restore_failure
    assert p.controller.journal_finalized is not restore_failure
    assert not p.runtime.core.arm_attempted
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command in (176,400,21) for _,msg in p.calls)


def test_native_without_review_remains_readonly(tmp_path):
    p=NativePeer(tmp_path,authorized=False);p.prepare()
    assert not p.runtime.core.arm_attempted
    assert all(msg.get_type() not in ('VISION_POSITION_ESTIMATE','SET_POSITION_TARGET_LOCAL_NED')
        and not(msg.get_type()=='COMMAND_LONG' and msg.command!=512) for msg in p.port.calls)


@pytest.mark.parametrize('action',['manual','sticks','kill','rc_lost','force_failsafe'])
def test_native_takeover_stop_and_loss_do_not_compete_with_receiver(tmp_path,action):
    p=prepared(tmp_path);airborne(p)
    setattr(p,action,True)
    changed_at=p.now
    for _ in range(100):
        p.turn()
        if p.runtime.core.pilot_control or p.runtime.core.stop_latched or p.runtime.failure:break
    assert p.now-changed_at<=1.+1e-9
    r=p.runtime
    if action in ('manual','sticks'):
        assert r.core.pilot_control and r.core.phase=='PILOT_CONTROL',r.status()
        before=len(p.calls)
        for _ in range(30):p.turn()
        assert any(msg.get_type()=='VISION_POSITION_ESTIMATE' for _,msg in p.calls[before:])
        assert not any(msg.get_type() in ('COMMAND_LONG','SET_POSITION_TARGET_LOCAL_NED') for _,msg in p.calls[before:])
        if action=='sticks':assert p.controller.status()['intervention_required']
        setattr(p,action,False)
        for _ in range(20):p.turn()
        assert r.core.pilot_control
    elif action in ('kill','force_failsafe'):
        assert r.core.stop_latched and r.core.phase=='STOP_LATCHED',r.status()
        assert not any(msg.get_type()=='COMMAND_LONG' and msg.command==185 for _,msg in p.calls)
    else:
        assert r.failure=='native_rc_loss_reported',r.status()
        assert r.core.phase in ('LAND','COMPLETE'),r.status()


def test_native_inputs_cannot_inject_old_gamepad_events(tmp_path):
    p=NativePeer(tmp_path)
    with pytest.raises(PermissionError):p.runtime.turn(0,home_events=(True,))
    with pytest.raises(PermissionError):p.runtime.enable_pilot({})
    assert not p.port.calls


@pytest.mark.parametrize('accepted',[False,True])
def test_native_mode_switch_yields_without_competing_with_pilot_land(tmp_path,accepted):
    p=prepared(tmp_path);airborne(p)
    p.mode_slot=6;p.pilot_land=accepted
    changed_at=p.now
    for _ in range(100):
        p.turn()
        if p.runtime.core.pilot_control:break
    assert p.now-changed_at<=1.+1e-9
    r=p.runtime
    assert r.core.pilot_control and r.core.takeover_reason=='native_rc_mode_switch_intent'
    assert p.controller.status()['intervention_required'] is (not accepted)
    before=len(p.calls)
    for _ in range(30):p.turn()
    assert any(msg.get_type()=='VISION_POSITION_ESTIMATE' for _,msg in p.calls[before:])
    assert not any(msg.get_type() in ('COMMAND_LONG','SET_POSITION_TARGET_LOCAL_NED') for _,msg in p.calls[before:])
    # Returning the switch never restarts the autonomous task.
    p.mode_slot=2;p.pilot_land=False
    for _ in range(30):p.turn()
    assert r.core.pilot_control


def test_native_mode_change_during_warmup_prevents_arm(tmp_path):
    p=prepared(tmp_path);start(p);p.mode_slot=6
    for _ in range(150):p.turn()
    assert p.runtime.core.pilot_control
    assert not p.runtime.core.arm_attempted
    assert p.runtime.core.phase=='BLOCKED'
    assert p.runtime.closed  # Fresh landed/disarmed proof closes without another SC action.
    assert any(e['event']=='prearm_takeover_ground_close' for e in p.runtime.events)
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command in (176,400) for _,msg in p.calls)


def test_native_factory_rejects_unapproved_profile_before_resources(tmp_path):
    profile=approved_profile();profile['delay_calibration']['status']='pending_calibration'
    path=tmp_path/'profile.json';path.write_text(json.dumps(profile))
    with pytest.raises(PermissionError,match='unapproved'):
        entry.open_pocket_session(profile_path=path,output=tmp_path/'run',
            peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),open_devices=True)
    assert not (tmp_path/'run').exists()


def test_native_start_rechecks_throttle_and_sticks(tmp_path):
    p=prepared(tmp_path); ticket=p.controller.start.create()
    p.throttle=0.
    # Deadline scheduling need not poll at the old batch's fixed 350ms.
    # Require the changed sample within the shared 1s RC query budget, then
    # verify confirmation uses it rather than the ticket's older low throttle.
    before=p.now
    while p.now-before<1.:
        p.turn()
        if p.runtime.inputs.pilot_status(p.now)['selected_manual_input']['throttle']==0.:break
    assert p.now-before<=1.+1e-9
    assert p.runtime.inputs.pilot_status(p.now)['selected_manual_input']['throttle']==0.
    with pytest.raises(PermissionError):p.controller.start.confirm(ticket['confirmation_text'])
    assert not p.runtime.core.arm_attempted


def test_mavlink_override_cannot_satisfy_native_rc_preparation(tmp_path):
    p=NativePeer(tmp_path);p.rc_source=6
    for _ in range(250):p.turn()
    assert not p.runtime.prepare_requested and not p.runtime.inputs.home_proven
    assert not any(msg.get_type()=='VISION_POSITION_ESTIMATE' for msg in p.port.calls)


def test_pocket_route_rejects_legacy_review_even_with_same_boot(tmp_path):
    from flight_execution_review import ReviewedExecutionGrant
    p=prepared(tmp_path)
    # A truncated legacy record is enough to prove the route mismatch is denied
    # before any historical evidence or device access can turn it into a grant.
    path=tmp_path/'legacy.json'
    path.write_text(json.dumps(dict(schema=1,scope='single-vertical-physical-flight-review-v1',authorized=True)))
    grant=ReviewedExecutionGrant(path,p.runtime,usb_present=lambda:False,check_port_identity=lambda:None)
    with pytest.raises(PermissionError):grant.consume()
    assert not Path(str(path)+'.consumed.json').exists()


def test_large_wifi_datagram_is_drained_with_existing_turn_budget(tmp_path):
    p=NativePeer(tmp_path)
    p.port.bounded_udp=True
    p.input_args=dict(pose=None,now_ros_ns=100_000_000_000)
    hb=p.pack(p.encoder.heartbeat_encode(2,m.MAV_AUTOPILOT_PX4,0,3<<16,3))
    p.port.rx=hb*200
    before=len(p.port.rx)
    p.io.step(.01)
    assert p.io.rx_bytes==p.runtime.max_rx_bytes
    assert len(p.port.rx)==before-p.runtime.max_rx_bytes
    assert p.io.failure is None


@pytest.mark.parametrize('name,value',[('COM_RC_OVERRIDE',1),('RC_MAP_KILL_SW',0),('RC_MAP_FLTMODE',7)])
def test_bad_native_takeover_configuration_is_reported_without_writes(tmp_path,name,value):
    p=NativePeer(tmp_path);p.params[name]=value
    for _ in range(350):p.turn()
    assert p.runtime.native_configuration is None and not p.runtime.prepare_requested
    assert p.runtime.coordinator.waiting_reason=='native_rc_override_kill_or_mode_configuration_missing'
    assert not any(msg.get_type() in ('PARAM_SET','VISION_POSITION_ESTIMATE') for msg in p.port.calls)


def test_native_parameter_drift_aborts_without_overwriting_it(tmp_path):
    import struct
    p=prepared(tmp_path);airborne(p)
    msg=p.encoder.param_value_encode(b'COM_RC_OVERRIDE',struct.unpack('<f',struct.pack('<i',1))[0],6,100,0)
    p.turn(extra_rx=p.pack(msg))
    assert p.runtime.failure=='native_rc_configuration_changed:COM_RC_OVERRIDE'
    assert not any(mg.get_type()=='PARAM_SET' for _,mg in p.calls)


def test_pocket_factory_uses_one_wifi_writer_and_raw_vicon_without_gamepad(tmp_path,monkeypatch):
    import continuous_device_host,rigid_body_revision,mavlink_readonly_probe,mavlink_link
    import ev_delay_vicon_source,vicon_receiver_provenance
    profile=approved_profile();profile['parameters']['COM_RC_IN_MODE']['value']=0
    profile['persistence']['configuration_sha256']=ekf_static_profile.configuration_digest(profile)
    path=tmp_path/'profile.json';path.write_text(json.dumps(profile))
    monkeypatch.setattr(continuous_device_host,'configuration',lambda:dict(
        revision='SYNTHETIC',origin=[0,0,0],orientation=[0,0,0,1]))
    monkeypatch.setattr(rigid_body_revision,'require_current_calibration',lambda *a:None)
    monkeypatch.setattr(mavlink_readonly_probe,'FC_USB',tmp_path/'no-physical-usb')
    monkeypatch.setattr(vicon_receiver_provenance,'verify_current',lambda:{})
    class Source:
        def __init__(self,*args):
            self.closed=False
            self.latest=dict(sdk_ok=True,occluded=False,pose_published=True,queue_dropped=0,
                subject='MircoAir',segment='MircoAir',frame_id='map',ros_stamp_ns=100_000_000_000,
                world_pose=[1,2,3,0,0,0,1])
        def poll(self):self.on_frame(self.latest,1)
        def now_ros_ns(self):return 100_000_000_000
        def close(self):self.closed=True
    source=Source(); port=FakePort();port.closed=False
    port.check=lambda:None
    def close():port.closed=True
    port.close=close
    def make_source(*a,**kw):
        assert kw=={'latest_only':True}
        return source
    monkeypatch.setattr(ev_delay_vicon_source,'ViconSource',make_source)
    monkeypatch.setattr(mavlink_link,'FixedPeerUDP',lambda *a,**k:port)
    c=entry.open_pocket_session(profile_path=path,output=tmp_path/'run',open_devices=True,
        peer=('192.168.4.1',14550),bind=('192.168.4.2',14550))
    assert not port.calls and c.io.poll_gamepad is None
    c.io.step(.01)
    assert c.runtime.coordinator.source_pose[1]==[1.,-2.,-3.]
    assert not c.runtime.inputs.home_proven and not c.runtime.inputs.pad_present
    assert set(c.io.types)<={'HEARTBEAT','COMMAND_LONG'}
    c.close()
    assert source.closed and port.closed


def test_wifi_one_hz_landed_feedback_is_configured_before_fusion_and_restored_last(tmp_path):
    p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync')
    run=p.run_owner_turn;last_landed=-1.;decoder=CrcResyncParser()
    def realistic_feedback(now,**kwargs):
        nonlocal last_landed
        rx=[]
        for msg in decoder.parse_buffer(kwargs['rx']):
            if msg.get_msgId()==245:
                period=(p.intervals[245] or 1_000_000)/1e6
                if now-last_landed<period-1e-8:continue
                last_landed=now
            rx.append(bytes(msg.get_msgbuf()))
        kwargs['rx']=b''.join(rx)
        return run(now,**kwargs)
    p.run_owner_turn=realistic_feedback
    assert not p.prepare()['failure'],p.runtime.status()
    assert p.runtime.core.phase=='WAIT_EXPLICIT_START'
    assert p.runtime.context().ground_status_max_age==.5
    assert p.intervals=={245:200000,32:100000,230:200000,111:100000}
    p.controller.cancel()
    for _ in range(800):
        p.turn(with_pose=False)
        if p.controller.done:break
    assert p.controller.done and p.controller.journal_finalized,p.controller.status()
    assert p.intervals=={245:0,32:0,230:0,111:0}
    sets=[msg for _,msg in p.calls if msg.get_type()=='COMMAND_LONG' and msg.command==511]
    assert sets[0].param1==sets[-1].param1==245 and sets[-1].param2==-1
    assert not p.runtime.core.arm_attempted


def test_wifi_rate_setup_waits_for_current_native_input_even_with_early_review(tmp_path):
    p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync')
    p.rc_lost=True
    for _ in range(100):p.turn()
    assert not p.runtime.failure,p.runtime.status()
    assert p.runtime.sync_rates.phase=='SET_NEXT'
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command==511 for _,msg in p.calls)
    p.rc_lost=False
    assert not p.prepare()['failure'],p.runtime.status()
    assert p.runtime.core.phase=='WAIT_EXPLICIT_START'


def test_wifi_interrupted_visual_series_drains_once_and_finishes_rate_cleanup(tmp_path):
    p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync')
    reply=p._reply_text;interrupted=False
    def partial_snapshot(key):
        nonlocal interrupted
        if key=='visual_snapshot' and not interrupted:
            interrupted=True
            return 'nsh> '+COMMANDS[key].decode()+'TOPIC: vehicle_visual_odometry\n'
        return reply(key)
    p._reply_text=partial_snapshot
    for _ in range(1500):
        p.turn()
        if interrupted:break
    assert interrupted and not p.runtime.failure,p.runtime.status()
    # The real listener waits for publications after localization stops.
    # Preserve a partial reply, then deliver its genuine terminal boundary.
    p.runtime._fail('fusion_preparation_context_lost')
    boundary_at=p.now+2.1;boundary_sent=False
    for _ in range(1000):
        if p.now>=boundary_at and not boundary_sent:
            p._shell_text('timeout waiting for update\nnsh> ');boundary_sent=True
        p.turn(with_pose=False)
        if p.controller.done:break
    assert boundary_sent and p.controller.done,p.controller.status()
    assert p.controller.journal_finalized,p.controller.status()
    assert p.runtime.sync_rates.summary()['restoration_verified']
    assert p.intervals=={245:0,32:0,230:0,111:0}
    visual=[msg for _,msg in p.calls if msg.get_type()=='SERIAL_CONTROL'
        and bytes(msg.data[:msg.count])==COMMANDS['visual_snapshot']]
    assert len(visual)==1 and not p.runtime.core.arm_attempted
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command in (176,400,21) for _,msg in p.calls)


def test_rate_restore_completion_waits_for_fresh_ground_before_journal_close(tmp_path):
    p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync')
    assert not p.prepare()['failure']
    rates=p.runtime.sync_rates;action=rates.action;expired=False
    def completed_between_actor_samples(now,**kwargs):
        nonlocal expired
        msg=action(now,**kwargs)
        if rates.phase=='CLOSED' and not expired:
            p.runtime.inputs.actuator['at']=now-1.1
            p.runtime.inputs.actuator['timestamp_us']=int((now+p.boot_offset-1.1)*1e6)
            expired=True
        return msg
    rates.action=completed_between_actor_samples
    p.controller.cancel()
    for _ in range(700):
        p.turn(with_pose=False)
        if expired:break
    assert expired and not p.controller.done
    assert p.io.journal.record['needs_restore']
    for _ in range(100):
        p.turn(with_pose=False)
        if p.controller.done:break
    assert p.controller.done and p.controller.journal_finalized,p.controller.status()


@pytest.mark.parametrize('stationary_manual',[False,True])
def test_two_hz_status_and_delayed_shell_use_short_visual_queries(tmp_path,stationary_manual):
    import re
    from collections import deque
    p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync')
    reply=p._reply_text
    publication={};snapshot_states=[]
    def slow_publication(key):
        text=reply(key)
        if key=='visual_snapshot':
            snapshot_states.append((p.runtime.coordinator.audit.live_lockdown(p.now),p.runtime.inputs.pilot_status(p.now)['native_input_observed']))
        if key in ('actuator','commander'):
            stamp=int((p.now+p.boot_offset)*1e6)//500000*500000
            # Unchanged state is 2Hz; a real arm/mode change publishes now.
            signature=re.sub(r' timestamp: \d+',' timestamp: SOURCE',text)
            previous=publication.get(key)
            if previous and previous[0]!=signature:
                stamp=int((p.now+p.boot_offset)*1e6)
            if previous:stamp=max(stamp,previous[1])
            publication[key]=(signature,stamp)
            text=re.sub(r' timestamp: \d+',f' timestamp: {stamp}',text)
        elif stationary_manual and key=='native_manual_control_setpoint':
            # ManualControl.cpp schedules unchanged valid output at 200ms;
            # rc_update can retain the input sample for up to 300ms.
            stamp=int((p.now+p.boot_offset)*1e6)//200000*200000
            sample=stamp//300000*300000
            text=re.sub(r' timestamp: \d+',f' timestamp: {stamp}',text)
            text=re.sub(r' timestamp_sample: \d+',f' timestamp_sample: {sample}',text)
        return text
    p._reply_text=slow_publication
    run=p.run_owner_turn;parser=CrcResyncParser();waiting=deque()
    def delayed_shell(now,**kwargs):
        rx=[]
        for msg in parser.parse_buffer(kwargs['rx']):
            wire=bytes(msg.get_msgbuf())
            if msg.get_type()=='SERIAL_CONTROL':waiting.append((now+.03,wire))
            else:rx.append(wire)
        while waiting and waiting[0][0]<=now+1e-9:rx.append(waiting.popleft()[1])
        kwargs['rx']=b''.join(rx)
        return run(now,**kwargs)
    p.run_owner_turn=delayed_shell
    assert not p.prepare()['failure'],p.runtime.status()
    assert p.runtime.core.phase=='WAIT_EXPLICIT_START',p.runtime.status()
    queries=[(at,bytes(m.data[:m.count])) for at,m in p.calls if m.get_type()=='SERIAL_CONTROL' and m.count]
    visual=[i for i,(_,q) in enumerate(queries) if q==COMMANDS['visual_snapshot']]
    assert len(visual)==3
    assert all(q!=COMMANDS['visual_series'] for _,q in queries)
    assert len(snapshot_states)==3 and all(actor is False and rc for actor,rc in snapshot_states)
    checks=p.runtime.coordinator.fusion.receipt_checks
    assert checks['recent_matching_receipts_verified']
    assert not checks['all_packet_reception_rate_verified']
    assert not checks['maximum_fc_publication_gap_verified']
    # Previously expired only 310ms after reaching WAIT_EXPLICIT_START.
    # Exercise sustained waiting across many full-audit refresh cycles.
    for _ in range(2000):
        p.turn()
        assert not p.runtime.failure,(p.now,p.runtime.inputs.build(p.now)[1],p.runtime.coordinator.audit.key,p.runtime.inputs.audit_at)
        assert p.runtime.core.phase=='WAIT_EXPLICIT_START'
    if stationary_manual:
        start(p)
        for _ in range(5000):
            p.turn()
            if p.controller.done:break
        assert p.runtime.core.phase=='COMPLETE',p.runtime.status()
        assert not p.runtime.failure,p.runtime.status()
    else:
        p.controller.cancel()
    for _ in range(1000):
        p.turn(with_pose=False)
        if p.controller.done:break
    assert p.controller.done and p.controller.journal_finalized,p.controller.status()


@pytest.mark.parametrize('host_age,source_age,accepted',[
    (.04,.461622,True),(.23,.704719,True),(.501,.6,False),(.1,1.001,False),(.1,-.001,False)])
def test_native_status_publication_period_is_not_network_freshness(host_age,source_age,accepted):
    from rc_watch import state_feedback_age
    assert (state_feedback_age(2,2-host_age,source_age,native=True)<=.5) is accepted
    if source_age>.5:assert state_feedback_age(2,2-host_age,source_age,native=False)>.5


@pytest.mark.parametrize('load',['current_run','mixed_archived_maxima'])
def test_recorded_reply_load_waits_flies_and_restores(tmp_path,load):
    import re
    from collections import deque
    evidence=json.loads((Path(__file__).parent/'fixtures/native_shell_timing_20261109.json').read_text())
    shapes={}
    for q in evidence['queries']:
        topic=q['command'].split()[1]
        shapes[topic]=dict(size=sum(len(c['text'].encode()) for c in q['chunks']),
            delays=[c['delay_s'] for c in q['chunks']])
    for q in evidence['recent_shapes']:
        topic=q['command'].split()[1];chunks=q['chunks'];origin=chunks[0]['at_ns']
        shape=dict(size=sum(len(c['text'].encode()) for c in chunks),
            # First-byte latency is not in echo-only recent records. Keep
            # 20ms explicitly modeled overhead; relative chunks are recorded.
            delays=[.02+(c['at_ns']-origin)/1e9 for c in chunks])
        if topic not in shapes or shape['delays'][-1]>shapes[topic]['delays'][-1]:shapes[topic]=shape
    # This full topic was not reached in the failing run: use the largest
    # observed single-topic shape as a disclosed synthetic load bound.
    shapes['estimator_status']=max(shapes.values(),key=lambda s:s['size'])
    a,b=shapes['vehicle_local_position'],shapes['vehicle_status']
    shapes['critical_status']=dict(size=a['size']+b['size'],
        delays=a['delays']+[a['delays'][-1]+d for d in b['delays']])
    # The current 10:25 run reached all topics, so use its actual sizes and
    # chunk spacing instead of mixing worst cases from different sessions.
    if load=='current_run':shapes=evidence['current_run_shapes']
    p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync');p.dt=.005
    waiting=deque();reply=p._reply_text;publication={};query_trace=[]
    def periodic(key):
        text=reply(key)
        if key in ('actuator','commander','native_manual_control_setpoint','selector') or key.startswith('flags_'):
            period=(1000000 if key=='selector' or key.startswith('flags_') else
                    200000 if key=='native_manual_control_setpoint' else 500000)
            stamp=int((p.now+p.boot_offset)*1e6)//period*period
            signature=re.sub(r' timestamp(?:_sample)?: \d+',' timestamp: TIME',text)
            previous=publication.get(key)
            if previous and previous[0]!=signature:stamp=int((p.now+p.boot_offset)*1e6)
            if previous:stamp=max(stamp,previous[1])
            publication[key]=(signature,stamp)
            text=re.sub(r' timestamp: \d+',f' timestamp: {stamp}',text)
            if key=='native_manual_control_setpoint':
                text=re.sub(r' timestamp_sample: \d+',f' timestamp_sample: {stamp}',text)
        return text
    p._reply_text=periodic
    def chunked_reply(text):
        assert not waiting,'No second command may overlap an unfinished reply'
        command=re.search(r'listener[^\n]*',text).group()
        topic='critical_status' if ';' in command else command.split()[1]
        query_trace.append(dict(at=p.now,topic=topic,timing=p.runtime.inputs.pilot_status(p.now)['topic_timing'],budget=dict(p.runtime.coordinator.audit.reply_durations)))
        shape=shapes[topic];data=text.encode()
        if len(data)<shape['size']:
            at=data.rfind(b'nsh> ');data=data[:at]+b' '*(shape['size']-len(data)-1)+b'\n'+data[at:]
        chunks=[data[i:i+70] for i in range(0,len(data),70)]
        delays=shape['delays']
        for i,chunk in enumerate(chunks):
            delay=delays[round(i*(len(delays)-1)/max(1,len(chunks)-1))]
            waiting.append((p.now+delay,p.pack(p.encoder.serial_control_encode(
                10,0,0,0,len(chunk),chunk.ljust(70,b'\0')))))
    p._shell_text=chunked_reply
    run=p.run_owner_turn
    def receive_chunks(now,**kwargs):
        while waiting and waiting[0][0]<=now+1e-9:kwargs['rx']+=waiting.popleft()[1]
        return run(now,**kwargs)
    p.run_owner_turn=receive_chunks
    result=p.prepare()
    (tmp_path/'query_trace.json').write_text(json.dumps(query_trace,indent=2))
    (tmp_path/'failure_detail.json').write_text(json.dumps(dict(pilot=p.runtime.inputs.pilot_status(p.now),failures=p.runtime.coordinator.audit.failures,result=result),indent=2,default=str))
    assert not result['failure'],(p.now,result['failure'],{k:v for k,v in p.runtime.inputs.pilot_status(p.now)['topic_timing'].items() if v['problems']},p.runtime.coordinator.audit.key,p.runtime.coordinator.audit.failures[-2:])
    assert p.runtime.core.phase=='WAIT_EXPLICIT_START',result
    assert p.runtime.inputs.pilot_status(p.now)['native_input_observed']
    assert not p.runtime.core.arm_attempted
    assert not any(m.get_type() in ('PARAM_SET','SET_POSITION_TARGET_LOCAL_NED') or
        m.get_type()=='COMMAND_LONG' and m.command in (176,185,400,21) for _,m in p.calls)
    for _ in range(4000):
        p.turn()
        assert not p.runtime.failure,(p.now,p.runtime.failure,p.runtime.inputs.pilot_status(p.now)['reasons'],{k:v for k,v in p.runtime.inputs.pilot_status(p.now)['topic_timing'].items() if v['problems']},p.runtime.coordinator.audit.key)
        assert p.runtime.core.phase=='WAIT_EXPLICIT_START'
    start(p)
    for _ in range(8000):
        p.turn()
        if p.runtime.core.phase=='COMPLETE':break
        assert not p.runtime.failure,(p.now,p.runtime.failure,p.runtime.inputs.pilot_status(p.now)['reasons'],{k:v for k,v in p.runtime.inputs.pilot_status(p.now)['topic_timing'].items() if v['problems']},p.runtime.coordinator.audit.key)
    assert p.runtime.core.phase=='COMPLETE',p.runtime.status()
    for _ in range(2000):
        p.turn(with_pose=False)
        if p.controller.done:break
    assert p.controller.done and p.controller.journal_finalized,p.controller.status()
    assert p.runtime.sync_rates.summary()['restoration_verified']


@pytest.mark.parametrize('gap',[.151,.25,.45])
def test_native_dispatch_jitter_with_current_data_and_real_send_review(tmp_path,gap):
    p,g=recorded_cadence_peer(tmp_path);r=p.runtime
    assert not p.prepare()['failure'];airborne(p)
    p.now=round(p.now+gap-.01,9);p.turn()
    if gap>.4:
        assert r.core.fault and r.core.phase in ('LAND','COMPLETE')
    else:
        assert r.failure is None and r.core.fault is None,p.controller.status()
        for _ in range(5000):
            p.turn()
            if p.controller.done:break
        assert p.controller.done and r.core.fault is None,p.controller.status()


@pytest.mark.parametrize('hold',[1.2,2.1])
def test_clock_holdover_keeps_reference_but_persistent_loss_ends_mission(tmp_path,hold):
    p,g=recorded_cadence_peer(tmp_path);r=p.runtime
    assert not p.prepare()['failure'];airborne(p)
    ref=dict(r.fixed_clock.reference);ingest=r.fixed_clock.ingest
    r.fixed_clock.ingest=lambda *a:False
    while p.now-ref['received_ns']/1e9<hold and not r.failure:
        p.turn()
    assert r.fixed_clock.reference==ref  # No reference restamping.
    if hold>2:
        assert r.failure and r.core.phase in ('LAND','COMPLETE')
    else:
        assert r.failure is None and r._fixed_clock_ok(),p.controller.status()
        r.fixed_clock.ingest=ingest
        for _ in range(5000):
            p.turn()
            if p.controller.done:break
        assert p.controller.done and r.core.fault is None,p.controller.status()


def test_auxiliary_reply_grace_reaches_context_and_real_send_review(tmp_path):
    from copy import deepcopy
    p,g=recorded_cadence_peer(tmp_path);r=p.runtime
    assert not p.prepare()['failure']
    i=r.inputs;now=r.now;boot=i._boot_us(now)
    i.actuator.update(at=now-.7,timestamp_us=int(boot-1.1e6))
    i.critical_status['local']['at']=now-.7
    i.critical_status['local']['value']['timestamp']=int(boot-.7e6)
    for key in ('input_rc','manual_control_setpoint','manual_control_switches','failsafe_flags'):
        i.native_rc.topics[key]['received_at']=now-1.5
    original=deepcopy(i.actuator)
    obs,missing=i.build(now)
    assert obs is not None and not missing,missing
    assert r.context().link_source_ok() and obs.navigation_ok()
    msg=r.encoder.command_long_encode(1,1,400,0,1,0,0,0,0,0,0)
    g.clock=lambda:now+.001
    assert g(r,{'at':now},msg),g.last_rejection
    assert i.actuator==original
    i.actuator['at']=now-1.01
    assert 'actuator_state_missing_or_stale' in i.build(now)[1]
    assert not r.context().link_source_ok() and not g(r,{'at':now},msg)
    assert g.last_rejection['reason']=='send_link_state_stale'


@pytest.mark.parametrize('prestart',[True,False])
def test_native_delayed_switch_uses_publication_time_not_arrival(tmp_path,monkeypatch,prestart):
    p=prepared(tmp_path);r=p.runtime
    # A changed slot is reported after start, but its FC publication can be old.
    boundary=int(r.inputs.attitude['boot_ms']*1000)
    previous=r.native_mode_sample
    r.native_mode_sample=(boundary-500_000,previous[1])
    start(p)
    status=dict(r.inputs.pilot_status(p.now))
    status['switches']=dict(status['switches'],timestamp=boundary-100_000 if prestart else boundary+100_000,
                            mode_slot=6)
    monkeypatch.setattr(r.inputs,'pilot_status',lambda now:status)
    r._service_native_rc(p.now)
    assert r.core.pilot_control is (not prestart)
    if prestart:assert r.core.phase=='WARMUP'
    else:assert r.core.takeover_reason=='native_rc_mode_switch_intent'
    assert not r.core.arm_attempted
