from pathlib import Path
import json
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mavlink_crc_stream as p


def wire_pair():
    e=m.MAVLink(None,srcSystem=1,srcComponent=1)
    attitude=bytes(e.attitude_encode(1,1,2,3,4,5,6).pack(e))
    heartbeat=bytes(e.heartbeat_encode(m.MAV_TYPE_QUADROTOR,m.MAV_AUTOPILOT_PX4,0,0,0).pack(e))
    return attitude,heartbeat


@pytest.mark.parametrize('width',[1,2,3,7,40,4096])
def test_bytewise_resynchronization_recovers_only_the_next_intact_frame(width):
    first,second=wire_pair();raw=first[:15]+second+bytes(40);decoder=p.CrcResyncParser();msgs=[]
    for i in range(0,len(raw),width):msgs+=decoder.parse_buffer(raw[i:i+width])
    valid=[msg for msg in msgs if msg.get_type()!='BAD_DATA']
    assert [msg.get_type() for msg in valid]==['HEARTBEAT']
    assert bytes(valid[0].get_msgbuf())==second
    assert any(msg.get_type()=='BAD_DATA' and 'CRC' in str(msg.reason) for msg in msgs)
    assert decoder.summary()['repaired_bytes']==0


def test_intact_frames_remain_identical_to_upstream_decoding():
    first,second=wire_pair();decoder=p.CrcResyncParser()
    messages=decoder.parse_buffer(first+second)
    assert [bytes(msg.get_msgbuf()) for msg in messages]==[first,second]
    assert decoder.summary()['counts']=={'crc_valid_frames':2}


def test_valid_serial_payload_is_not_searched_for_nested_heartbeat():
    e=m.MAVLink(None,srcSystem=1,srcComponent=1);_,inner=wire_pair()
    wire=bytes(e.serial_control_encode(10,0,0,0,len(inner),inner.ljust(70,b'\0')).pack(e))
    assert [msg.get_type() for msg in p.CrcResyncParser().parse_buffer(wire)]==['SERIAL_CONTROL']


def test_ignore_crc_environment_cannot_be_used_to_accept_damage(monkeypatch):
    monkeypatch.setattr(p.m,'MAVLINK_IGNORE_CRC',True)
    with pytest.raises(ValueError,match='disabled'):p.CrcResyncParser()


def test_switching_crc_bypass_mid_session_is_rejected(monkeypatch):
    decoder=p.CrcResyncParser();monkeypatch.setattr(p.m,'MAVLINK_IGNORE_CRC',True)
    with pytest.raises(ValueError,match='during session'):decoder.parse_buffer(wire_pair()[1])


def test_signed_or_unknown_schema_never_becomes_trusted_message():
    raw=bytearray(wire_pair()[1]);raw[2]=1
    decoder=p.CrcResyncParser()
    assert all(msg.get_type()=='BAD_DATA' for msg in decoder.parse_buffer(raw+bytes(20)))
    raw=bytearray(wire_pair()[1]);raw[7:10]=b'\xff\xff\xff'
    assert all(msg.get_type()=='BAD_DATA' for msg in p.CrcResyncParser().parse_buffer(raw))


def test_pinned_table_crc_extra_disagreement_fails_closed(tmp_path):
    record=json.loads(p.DEFINITIONS.read_text());record['messages']['0'][0]^=1
    path=tmp_path/'defs.json';path.write_text(json.dumps(record))
    with pytest.raises(ValueError,match='CRC_EXTRA mismatch'):p.CrcResyncParser(path)


@pytest.mark.parametrize('field,value',[(0,-1),(0,300),(1,-1),(2,256),(2,'9')])
def test_invalid_definition_values_rejected(tmp_path,field,value):
    record=json.loads(p.DEFINITIONS.read_text());record['messages']['0'][field]=value
    path=tmp_path/'defs.json';path.write_text(json.dumps(record))
    with pytest.raises(ValueError):p.CrcResyncParser(path)
