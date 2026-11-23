"""Injected memory port only: no real USB, serial, network or ROS access."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mission_preparation_io import PreparationIO
from mavlink_crc_stream import CrcResyncParser
from test_mission_preparation_runtime import Peer,prepared_peer


class MemoryPort:
    timeout=write_timeout=0
    def __init__(self):
        self.rx=b'';self.writes=[];self.short=False;self.read_fails=False;self.order=[]
    def read(self,limit):
        assert limit==2048
        if self.read_fails:raise OSError('synthetic read failure')
        result,self.rx=self.rx[:limit],self.rx[limit:]
        return result
    def write(self,wire):
        self.order.append('write');self.writes.append(wire)
        return len(wire)-1 if self.short else len(wire)


def attach(peer,*,home=(),authorize=None,journal=None,usb=True):
    port=MemoryPort();now=peer.now+.032;ns=int((100+now)*1e9)
    io=PreparationIO(peer.runtime,port,
        poll_gamepad=lambda:dict(pad_generation='synthetic-pad',pad_present=True,home_events=home),
        poll_pose=lambda:(ns,'map',[1,-2,.1],[0,0,0,1]),usb_present=lambda:usb,
        authorize=authorize,journal=journal)
    return io,port,now,ns


def decode(wire):return CrcResyncParser().parse_buffer(wire)[0]


def test_adapter_never_replays_old_offline_capture_history():
    p=Peer();p.home_check();io,port,t,ns=attach(p)
    assert io.seen==sum(p.runtime.sent_counts.values())
    old=len(p.runtime.frames);io.step(t,ns)
    assert len(port.writes)==len(p.runtime.frames)-old
    assert not any(decode(wire).get_type()=='COMMAND_LONG' and decode(wire).command==185 for wire in port.writes)


def test_default_device_boundary_denies_localization_before_any_pose_bytes():
    p=prepared_peer();io,port,t,ns=attach(p)
    result=io.step(t,ns)
    assert 'authorization' in result['failure']
    assert not port.writes and result['physical_tx_bytes']==0
    assert p.runtime.failure and not result['flight_authorized']


def test_home_is_sent_first_even_when_rx_fails_and_future_reads_stay_blocked():
    p=prepared_peer();io,port,t,ns=attach(p,home=(True,))
    port.read_fails=True;result=io.step(t,ns)
    msg=decode(port.writes[0])
    assert msg.command==185 and msg.param1==2
    assert result['failure'] and p.runtime.stop_latched
    assert len(port.writes)==1


def test_usb_absence_is_never_treated_as_flight_authorization():
    p=Peer();p.home_check();io,port,t,ns=attach(p,home=(True,),usb=False)
    result=io.step(t,ns)
    assert result['failure'] and not port.writes
    assert p.runtime.home_check_requested


@pytest.mark.parametrize('timeout,write_timeout',[(1,0),(0,1),(None,0),(0,None)])
def test_blocking_serial_settings_are_rejected_before_any_reads(timeout,write_timeout):
    p=Peer();port=MemoryPort();port.timeout=timeout;port.write_timeout=write_timeout
    with pytest.raises(ValueError,match='nonblocking'):
        PreparationIO(p.runtime,port,poll_gamepad=lambda:{},poll_pose=lambda:None,usb_present=lambda:True)
    assert not port.writes


def pending_parameter():
    p=Peer(fusion=True);p.home_check();p.runtime.request_prepare(operator_confirmed=True)
    for _ in range(50):
        p.turn()
        if p.runtime.coordinator.fusion.phase=='WAIT_DELAY_ECHO':break
    assert p.runtime.coordinator.fusion.phase=='WAIT_DELAY_ECHO'
    record=next(row for row in reversed(p.runtime.frames) if row['type']=='PARAM_SET')
    return p,record


def test_even_authorized_parameter_requires_write_ahead_journal():
    p,record=pending_parameter();io,port,_,_=attach(p,authorize=lambda *args:True)
    with pytest.raises(PermissionError,match='journal'):io._dispatch(record)
    assert not port.writes and not io.possibly_changed


def test_journal_precedes_possible_partial_write_and_preserves_recovery_flag():
    p,record=pending_parameter();entries=[]
    io,port,_,_=attach(p,authorize=lambda *args:True)
    def journal(row):
        port.order.append('journal');entries.append(row)
        return True
    io.journal=journal;port.short=True
    with pytest.raises(OSError,match='Short'):io._dispatch(record)
    assert port.order==['journal','write']
    assert entries[0]['baseline']['EKF2_EV_DELAY']==0
    assert io.status()['parameter_recovery_required']
    assert io.physical_tx_bytes==len(bytes.fromhex(record['wire_hex']))-1


def test_reviewed_callback_cannot_enable_flight_commands():
    p=Peer();p.home_check();io,port,_,_=attach(p,authorize=lambda *args:True,journal=lambda row:True)
    enc=p.runtime.encoder
    for command in (176,400,21,22,246,511):
        msg=enc.command_long_encode(1,1,command,0,1,0,0,0,0,0,0)
        record=dict(type='COMMAND_LONG',wire_hex=msg.pack(enc).hex())
        with pytest.raises(PermissionError):io._dispatch(record)
    assert not port.writes


def test_reader_failure_latches_without_silently_resuming_configuration():
    p=prepared_peer();io,port,t,ns=attach(p,authorize=lambda *args:True,journal=lambda row:True)
    def broken():raise OSError('synthetic missing gamepad')
    io.poll_gamepad=broken
    status=io.step(t,ns)
    assert status['failure'] and p.runtime.failure
    assert not port.writes


def test_transport_fault_cannot_be_cleared_by_local_closed_state():
    p,record=pending_parameter();io,_,_,_=attach(p,authorize=lambda *args:True,journal=lambda row:True)
    io._dispatch(record)
    io.failure='uncertain_restore_delivery'
    p.runtime.closed=True  # A local phase alone never proves physical restoration.
    assert io.status()['parameter_recovery_required']
