"""No serial access: USB state proof remains mandatory in the TRS workaround."""
from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from trs_static_fusion_bench import check_observer_reply
from mavlink_crc_stream import CrcResyncParser


def test_wireless_damage_does_not_certify_state_or_block_independent_usb_reads():
    bad=m.MAVLink_bad_data(b'broken','invalid CRC')
    check_observer_reply('trs',bad,pending=True,restoring=False)
    check_observer_reply('usb',bad,pending=False,restoring=False)
    with pytest.raises(RuntimeError,match='USB observer reply damaged'):
        check_observer_reply('usb',bad,pending=True,restoring=False)


def test_observer_damage_does_not_block_best_effort_parameter_rollback():
    check_observer_reply('usb',m.MAVLink_bad_data(b'broken','invalid CRC'),pending=True,restoring=True)


def test_truncated_radio_frame_keeps_following_crc_valid_state_on_separate_usb_route():
    enc=m.MAVLink(None,srcSystem=1,srcComponent=1)
    first=bytes(enc.attitude_encode(1,1,2,3,4,5,6).pack(enc))
    second=bytes(enc.heartbeat_encode(m.MAV_TYPE_QUADROTOR,m.MAV_AUTOPILOT_PX4,0,0,0).pack(enc))
    radio,usb=CrcResyncParser(),CrcResyncParser()
    damaged=radio.parse_buffer(first[:15]+second+bytes(40))
    for msg in damaged:check_observer_reply('trs',msg,pending=True,restoring=False)
    assert any(msg.get_type()=='BAD_DATA' for msg in damaged)
    assert any(msg.get_type()=='HEARTBEAT' for msg in damaged)
    intact=usb.parse_buffer(second)
    assert len(intact)==1 and intact[0].get_type()=='HEARTBEAT'
    check_observer_reply('usb',intact[0],pending=True,restoring=False)
    assert radio.summary()['repaired_bytes']==usb.summary()['repaired_bytes']==0
