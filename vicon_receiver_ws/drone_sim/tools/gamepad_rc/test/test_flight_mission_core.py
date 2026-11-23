"""Real wire encoding + injected offline sink; never opens hardware or network."""
from dataclasses import replace
from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import flight_mission_core as p
from mavlink_crc_stream import CrcResyncParser


def observation(t, **changes):
    values=dict(t=t,boot_session='offline-boot',rigid_revision='offline-rigid',
        reset_counters=(0,0,0),position_ned=(1.,2.,2.3),velocity_ned=(0.,0.,0.),yaw_ned=.6,
        uid2=p.UID2,firmware='1.14.3',status_age_s=.01,local_age_s=.01,pose_age_s=.01,
        armed=False,landed=True,lockdown=True,fusion_ready=True,preflight_ready=True,
        gamepad_available=True,home_response_verified=True)
    return p.Observation(**{**values,**changes})


def selector(instance=0,count=1):
    return dict(primary_instance=instance,instance_changed_count=count,instances_available=2,
                healthy=[True,True]+[False]*7,gyro_fault_detected=False,accel_fault_detected=False)


class Sink:
    def __init__(self):
        self.messages=[]
        self.encoder=m.MAVLink(None,srcSystem=250,srcComponent=p.MissionCore.component)
    def __call__(self,message):
        wire=message.pack(self.encoder)
        self.encoder.seq=(self.encoder.seq+1)%256
        decoded=CrcResyncParser().parse_buffer(wire)
        assert len(decoded)==1 and decoded[0].get_type()!='BAD_DATA'
        self.messages.append(decoded[0])
        return True


def ready(*,wire_boundary=False,**config):
    sink=Sink();core=p.MissionCore(sink,**config)
    if wire_boundary:
        from mission_wire_transport import MissionWireTransport
        transport=MissionWireTransport();transport.attach(core)
        def dispatch(msg):
            assert transport(msg)
            return sink(msg)
        core.sink=dispatch
    assert not core.prepare(observation(0),selector())
    assert core.prepare(observation(2),selector())
    core.start(observation(2.1,lockdown=False),selector(),operator_confirmed=True)
    assert not sink.messages
    return core,sink


def tick(core,t,**changes):
    o=observation(t,lockdown=False,**changes)
    core.vision_message(o,int((100+t)*1e6),[1.,-2.,-3.],[1.,0.,0.,0.],0.)
    return core.tick(o,selector(),boot_ms=int(t*1000))


def ack(core,command,t,result=m.MAV_RESULT_ACCEPTED,*,target=(250,220),source=(1,1)):
    encoder=m.MAVLink(None,srcSystem=source[0],srcComponent=source[1])
    msg=encoder.command_ack_encode(command,result,target_system=target[0],target_component=target[1])
    msg=m.MAVLink(None).parse_char(msg.pack(encoder))
    return core.ack(msg,t)


def offboard_wait(*,wire_boundary=False):
    core,sink=ready(wire_boundary=wire_boundary)
    for i in range(22):tick(core,2.1+i*.1)
    assert core.phase=='WAIT_OFFBOARD'
    return core,sink


def airborne(*,wire_boundary=False):
    core,sink=offboard_wait(wire_boundary=wire_boundary)
    assert ack(core,176,4.3)
    tick(core,4.3,offboard=True)
    assert core.phase=='WAIT_ARM'
    assert ack(core,400,4.4)
    tick(core,4.4,offboard=True,armed=True)
    assert core.phase=='ASCEND'
    return core,sink


@pytest.mark.parametrize('change',[
    {'uid2':'wrong'}, {'firmware':'1.17.0'}, {'armed':True}, {'landed':False},
    {'fusion_ready':False}, {'preflight_ready':False}, {'gamepad_available':False},
    {'home_response_verified':False}, {'pose_age_s':.151}, {'status_age_s':.501},
    {'local_age_s':.501}, {'position_ned':(float('nan'),0,0)}])
def test_prepare_never_invents_missing_evidence(change):
    sink=Sink();core=p.MissionCore(sink)
    with pytest.raises(ValueError):core.prepare(observation(0,**change),selector())
    assert not sink.messages


def test_reuses_healthy_initialization_settlement_and_freezes_change_counter():
    sink=Sink();core=p.MissionCore(sink)
    assert not core.prepare(observation(0),selector())
    assert not core.prepare(observation(.5),selector(1,2))
    assert core.prepare(observation(2.1),selector(1,2))
    assert core.frozen_selector==(1,2) and not sink.messages


def test_start_cannot_clear_home_lock_or_reuse_missing_confirmation():
    core=p.MissionCore(Sink());core.prepare(observation(0),selector());core.prepare(observation(2),selector())
    with pytest.raises(ValueError):core.start(observation(2.1),selector(),operator_confirmed=True)
    with pytest.raises(ValueError):core.start(observation(2.1,lockdown=False),selector())
    assert core.phase=='WAIT_EXPLICIT_START' and not core.sent


def test_actual_sink_acceptance_not_planned_messages_drives_warmup():
    core,sink=ready()
    core.sink=lambda message:False
    with pytest.raises(OSError):tick(core,2.1)
    assert core.phase=='BLOCKED' and core.fault=='dispatch_error_uncertain_delivery'
    assert not core.setpoints and not core.offboard_attempted


def test_warmup_sends_frozen_current_target_and_only_one_mode_request():
    core,sink=offboard_wait()
    sp=[msg for msg in sink.messages if msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED']
    assert len(sp)>=20
    assert all((msg.x,msg.y,msg.z)==pytest.approx((1,2,2.3)) for msg in sp)
    commands=[msg for msg in sink.messages if msg.get_type()=='COMMAND_LONG']
    assert len(commands)==1 and commands[0].command==176
    assert (commands[0].param1,commands[0].param2)==(1,6)


@pytest.mark.parametrize('source,target', [((1,88),(250,220)),((2,1),(250,220)),((1,1),(250,219))])
def test_wrong_ack_source_or_target_does_not_advance(source,target):
    core,_=offboard_wait()
    assert not ack(core,176,4.3,source=source,target=target)
    tick(core,4.3,offboard=True)
    assert core.phase=='WAIT_OFFBOARD' and not core.arm_attempted


def test_accepted_ack_alone_cannot_prove_mode_or_arm_state():
    core,_=offboard_wait();assert ack(core,176,4.3)
    tick(core,4.3)
    assert core.phase=='WAIT_OFFBOARD' and not core.arm_attempted
    tick(core,4.4,offboard=True)
    assert core.phase=='WAIT_ARM';assert ack(core,400,4.5)
    tick(core,4.5,offboard=True)
    assert core.phase=='WAIT_ARM'


@pytest.mark.parametrize('wire_boundary',[False,True])
def test_complete_vertical_mission_uses_relative_height_ramp_and_observed_landing(wire_boundary):
    core,sink=airborne(wire_boundary=wire_boundary);position=core.origin.position_ned;land_since=None
    for i in range(1,200):
        t=4.4+i*.1
        if core.phase=='LAND' and land_since is None:
            land_since=t
            assert ack(core,21,t)
        landed=land_since is not None and t-land_since>=1
        tick(core,t,armed=not landed,landed=landed,offboard=True,position_ned=position)
        targets=[msg for msg in sink.messages if msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED']
        if targets:position=(targets[-1].x,targets[-1].y,targets[-1].z)
        if core.phase=='COMPLETE':break
    assert core.phase=='COMPLETE' and core.fault is None
    assert min(msg.z for msg in targets)==pytest.approx(1.8)
    assert all((msg.x,msg.y)==pytest.approx((1,2)) for msg in targets)
    assert all(abs(a.z-b.z)<=.015001 for a,b in zip(targets,targets[1:]))
    commands=[msg for msg in sink.messages if msg.get_type()=='COMMAND_LONG']
    assert [msg.command for msg in commands]==[176,400,21]
    assert all(msg.param1==1 for msg in commands if msg.command==400)
    assert not core.status()['flight_authorized']
    assert core.status()['actual_sent_to_fc']==0
    assert core.status()['mission_sequence_passed_in_offline_sink']


@pytest.mark.parametrize('changes', [
    {'pose_age_s':.151}, {'local_age_s':.501}, {'gamepad_available':False},
    {'fusion_ready':False}, {'reset_counters':(1,0,0)}, {'rigid_revision':'changed'},
    {'boot_session':'changed'}, {'offboard':False}])
def test_inflight_fault_aborts_without_rearming_or_clearing_lock(changes):
    core,sink=airborne()
    values=dict(armed=True,landed=False,offboard=True)
    tick(core,4.5,**{**values,**changes})
    assert core.phase=='LAND' and core.fault
    commands=[msg.command for msg in sink.messages if msg.get_type()=='COMMAND_LONG']
    assert commands==[176,400,21]


def test_home_with_invalid_navigation_still_latches_and_sends_stop_when_armed():
    core,sink=airborne()
    o=observation(4.5,armed=True,landed=False,home_pressed=True,pose_age_s=100,
                  position_ned=(float('nan'),0,0),gamepad_available=False)
    core.tick(o,{},boot_ms=0)
    assert core.phase=='STOP_LATCHED'
    msg=sink.messages[-1];assert msg.command==185 and msg.param1==2
    core.tick(replace(o,t=4.6,home_pressed=False),{},boot_ms=0)
    assert core.stop_latched and not core.vision_message(o,999999999,[0,0,0],[1,0,0,0],0)
    with pytest.raises(ValueError):core.start(o,selector(),operator_confirmed=True)


def test_lost_link_sends_no_unconfirmed_control_and_never_fakes_landing():
    core,sink=airborne();before=len(sink.messages)
    tick(core,4.5,armed=True,landed=False,status_age_s=2)
    assert core.phase=='LAND' and len(sink.messages)==before
    core.tick(observation(40,armed=True,landed=False,status_age_s=2),selector(),boot_ms=40000)
    assert core.phase=='LAND_TIMEOUT'
    assert not core.status()['mission_sequence_passed_in_offline_sink']


def test_fresh_localization_continues_during_landing_no_parameter_rollback_in_air():
    core,sink=airborne();tick(core,4.5,armed=True,landed=False,gamepad_available=False)
    assert core.phase=='LAND'
    o=observation(4.6,armed=True,landed=False,lockdown=False)
    assert core.vision_message(o,200000000,[1,2,3],[1,0,0,0],0)
    assert not any(msg.get_type()=='PARAM_SET' for msg in sink.messages)


@pytest.mark.parametrize('command,param1',[(176,1),(400,1),(400,0),(21,0),(185,2),(185,0),(246,1)])
def test_real_device_default_boundary_rejects_every_mission_command(command,param1):
    msg=m.MAVLink(None).command_long_encode(1,1,command,0,param1,0,0,0,0,0,0)
    assert not p.readonly_dispatch_allowed(msg)


def test_single_owner_cycle_services_home_before_invalid_vision_math():
    core,sink=airborne()
    result=core.cycle(observation(4.5,armed=True,home_pressed=True),{},boot_ms=0,
                      vision=('invalid',None,None,None))
    assert result['stop_latched'] and sink.messages[-1].command==185


def test_single_owner_cycle_invalid_vision_cannot_keep_ascending():
    core,sink=airborne()
    core.cycle(observation(4.5,armed=True,landed=False,lockdown=False,offboard=True),selector(),boot_ms=4500,
               vision=(200000000,[1,2,3],[0,0,0,0],0))
    assert core.phase=='LAND' and core.fault=='invalid_vision_encoding'
    assert sink.messages[-1].command==21


def test_unexpected_arm_before_arm_request_aborts_and_never_rearms():
    core,sink=ready()
    tick(core,2.1,armed=True,landed=False)
    assert core.phase=='LAND'
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command==400 for msg in sink.messages)


def test_partial_arm_send_cannot_be_repeated_as_if_it_never_happened():
    core,sink=offboard_wait();ack(core,176,4.3)
    def fail_on_arm(msg):
        if msg.get_type()=='COMMAND_LONG' and msg.command==400:return False
        return sink(msg)
    core.sink=fail_on_arm
    with pytest.raises(OSError):tick(core,4.3,offboard=True)
    assert core.arm_attempted and core.phase=='LAND'
    assert core.fault=='dispatch_error_uncertain_delivery'


def test_no_actual_vision_sink_acceptance_cannot_be_substituted_by_recent_source_age():
    core,_=ready()
    core.tick(observation(2.1,lockdown=False),selector(),boot_ms=2100)
    assert core.phase=='BLOCKED' and core.fault=='fresh_vision_dispatch_lost'


def test_landing_timeout_never_turns_off_fresh_position_stream_or_restarts_mission():
    core,sink=airborne();tick(core,4.5,armed=True,landed=False,gamepad_available=False)
    core.tick(observation(40,armed=True,landed=False),selector(),boot_ms=40000)
    assert core.phase=='LAND_TIMEOUT'
    o=observation(40.1,armed=True,landed=False)
    core.cycle(o,selector(),boot_ms=40100,vision=(500000000,[1,2,3],[1,0,0,0],0))
    assert sink.messages[-1].get_type()=='VISION_POSITION_ESTIMATE'
    assert core.phase=='LAND_TIMEOUT'


def test_home_stop_delivery_failure_remains_latched_not_a_landing_restart():
    core,_=airborne();core.sink=lambda msg:False
    with pytest.raises(OSError):core.tick(observation(4.5,armed=True,home_pressed=True),{},boot_ms=0)
    assert core.phase=='STOP_LATCHED' and core.stop_latched


@pytest.mark.parametrize('field',['fusion_ready','preflight_ready','gamepad_available',
                                  'home_response_verified','armed','landed'])
def test_truthy_strings_cannot_substitute_for_verified_boolean_state(field):
    core=p.MissionCore(Sink())
    with pytest.raises(ValueError):core.prepare(observation(0,**{field:'true'}),selector())


def test_missing_mode_ack_never_causes_arm_and_has_bounded_timeout():
    core,sink=offboard_wait()
    for i in range(1,35):
        tick(core,4.2+i*.1,offboard=True)
        if core.phase=='BLOCKED':break
    assert core.phase=='BLOCKED' and not core.arm_attempted
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command==400 for msg in sink.messages)


def test_arm_confirmation_timeout_lands_without_retrying_arm():
    core,sink=offboard_wait();ack(core,176,4.3);tick(core,4.3,offboard=True)
    for i in range(1,35):
        tick(core,4.3+i*.1,offboard=True,landed=False)
        if core.phase=='LAND':break
    assert core.phase=='LAND'
    assert sum(msg.get_type()=='COMMAND_LONG' and msg.command==400 for msg in sink.messages)==1


def test_selector_change_after_freeze_does_not_rebind_any_target():
    core,sink=airborne();before=sum(msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED' for msg in sink.messages)
    o=observation(4.5,armed=True,landed=False,lockdown=False,offboard=True)
    core.cycle(o,selector(1,2),boot_ms=4500)
    assert core.phase=='LAND' and core.frozen_selector==(0,1)
    assert sum(msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED' for msg in sink.messages)==before


def test_backwards_clock_does_not_resume_the_previous_flight_phase():
    core,_=airborne()
    with pytest.raises(ValueError):core.tick(observation(1,armed=True),selector(),boot_ms=1000)
    assert core.phase=='LAND' and core.fault=='invalid_session_clock'


@pytest.mark.parametrize('stamp',[1,104400000])
def test_stored_or_duplicate_vision_timestamp_never_becomes_a_new_send(stamp):
    core,sink=airborne();before=len(sink.messages)
    assert not core.vision_message(observation(4.5),stamp,[0,0,0],[1,0,0,0],0)
    assert len(sink.messages)==before
