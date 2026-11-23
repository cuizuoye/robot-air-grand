from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from flight_mission_core import MissionCore
from mission_wire_transport import MissionWireTransport


def wire_core(**args):
    transport=MissionWireTransport(**args)
    core=MissionCore(transport);transport.attach(core)
    core.origin=NS(position_ned=(1,2,3),yaw_ned=.4)
    return core,transport


def arm(core):
    core.phase='WAIT_ARM';core.arm_attempted=True
    return core.encoder.command_long_encode(1,1,400,0,1,0,0,0,0,0,0)


def test_real_mission_writes_have_no_default_authorization_even_with_attached_port():
    class Port:
        def write(self,wire):pytest.fail('Default-denied mission must not touch real writer')
    core,t=wire_core(port=Port(),usb_present=lambda:False)
    with pytest.raises(PermissionError):t(arm(core))
    assert t.physical_tx_bytes==0 and not t.records


def test_usb_connected_always_blocks_flight_traffic_even_with_authorizer():
    class Port:
        def write(self,wire):pytest.fail('FC USB connected; no flight writes')
    core,t=wire_core(port=Port(),usb_present=lambda:True,authorizer=lambda *a:True)
    with pytest.raises(PermissionError):t(arm(core))


def test_offline_wire_capture_encodes_real_mavlink_without_any_port():
    core,t=wire_core();assert t(arm(core))
    packet=bytes.fromhex(t.records[0]['wire_hex'])
    msg=m.MAVLink(None).parse_char(packet)
    assert msg.command==400 and msg.param1==1 and msg.param2==0
    assert t.physical_tx_bytes==0 and not t.status()['flight_authorized']


@pytest.mark.parametrize('cmd,param1',[(400,0),(400,1),(185,0),(185,1),(246,1),(22,0)])
def test_no_unrequested_arm_disarm_unlock_motor_reboot_or_takeoff_command(cmd,param1):
    core,t=wire_core()
    message=core.encoder.command_long_encode(1,1,cmd,0,param1,0,0,0,0,0,0)
    with pytest.raises(ValueError):t(message)


def test_short_write_latches_uncertain_delivery_and_blocks_all_automatic_retries():
    port=NS(write=lambda wire:3)
    core,t=wire_core(port=port,usb_present=lambda:False,authorizer=lambda *a:True)
    with pytest.raises(OSError,match='Short write'):t(arm(core))
    assert t.physical_tx_bytes==3 and t.uncertain_delivery
    with pytest.raises(OSError,match='Previous'):t(arm(core))


def test_authorized_writer_test_uses_only_fake_port_and_full_wire_acceptance():
    packets=[]
    port=NS(write=lambda wire:(packets.append(wire) or len(wire)))
    core,t=wire_core(port=port,usb_present=lambda:False,authorizer=lambda *a:True)
    assert t(arm(core)) and len(packets)==1
    assert t.physical_tx_bytes==len(packets[0])


def test_transport_owner_cannot_be_replaced():
    core,t=wire_core()
    with pytest.raises(ValueError):t.attach(core)


def test_home_stop_can_still_be_attempted_after_uncertain_write_without_clearing_fault():
    port=NS(write=lambda wire:3)
    core,t=wire_core(port=port,usb_present=lambda:False,authorizer=lambda *a:True)
    with pytest.raises(OSError):t(arm(core))
    port.write=lambda wire:len(wire)
    core.phase='STOP_LATCHED';core.stop_latched=True
    msg=core.encoder.command_long_encode(1,1,185,0,2,0,0,0,0,0,0)
    assert t(msg) and t.uncertain_delivery
    assert t.records[-1]['physical_dispatch']
