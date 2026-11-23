"""Offline regression for startup MAVLink1 AUTOPILOT_VERSION without uid2."""
from types import SimpleNamespace
import pytest
from pymavlink.dialects.v20 import common as m
from pymavlink.dialects.v10 import common as m1
from ev_delay_device_io import Endpoint

EXPECTED = dict(firmware='1.14.3', board_version=54, uid=123, uid2=bytes(range(18)).hex())

def endpoint():
    port = SimpleNamespace(is_open=True, in_waiting=0)
    port.read = lambda size: b''
    ep = Endpoint(port, 219, EXPECTED, clock=lambda: 1000000000)
    ep.evidence.version_requested_at = 0
    return ep

def feed(ep, dialect, uid=123, uid2=None):
    enc = dialect.MAVLink(None, srcSystem=1, srcComponent=1)
    args = [0, (1<<24)|(14<<16)|(3<<8), 0, 0, 54, [1]*8, [0]*8, [0]*8, 0, 0, uid]
    if dialect is m: args.append(list(range(18)) if uid2 is None else uid2)
    packet = enc.autopilot_version_encode(*args).pack(enc)
    ep.port.in_waiting = len(packet)
    ep.port.read = lambda size: packet
    ep.poll()

def test_v1_is_not_identity_proof_then_complete_v2_matches():
    ep=endpoint();feed(ep,m1)
    assert ep.identity_ns is None and ep.evidence.identity is None and not ep.safe()
    assert ep.incomplete_version['wire_hex'].startswith('fe')
    feed(ep,m)
    assert ep.evidence.identity == EXPECTED and ep.identity_ns == 1000000000

@pytest.mark.parametrize('dialect,uid,uid2', [(m1,999,None),(m,999,None),(m,123,[0]*18)])
def test_real_mismatch_still_fails(dialect,uid,uid2):
    ep=endpoint()
    with pytest.raises(RuntimeError,match='FC identity changed'):feed(ep,dialect,uid,uid2)
    assert not ep.safe()

def peer_packet():
    enc=m.MAVLink(None,srcSystem=250,srcComponent=219)
    return enc.heartbeat_encode(6,8,0,0,0).pack(enc)

def test_forwarded_owned_peer_is_not_fc_sync_or_identity():
    ep=endpoint();packet=peer_packet()
    ep.port.in_waiting=len(packet);ep.port.read=lambda size:packet
    ep.poll()
    assert ep.sync.first_valid_ns is None and ep.identity_ns is None and not ep.safe()
    assert ep.forwarded_peer_frames['219:HEARTBEAT']==1
    feed(ep,m)
    assert ep.evidence.identity==EXPECTED

def test_forwarded_peer_bad_crc_is_still_fatal():
    ep=endpoint();packet=bytearray(peer_packet());packet[-1]^=1
    ep.port.in_waiting=len(packet);ep.port.read=lambda size:bytes(packet)
    with pytest.raises(RuntimeError,match='CRC/framing'):ep.poll()

def test_forwarded_peer_cannot_validate_orphan_prefix():
    ep=endpoint();ep.sync.pending_orphan={'at_ns':1000000000}
    packet=peer_packet();ep.port.in_waiting=len(packet);ep.port.read=lambda size:packet
    with pytest.raises(RuntimeError,match='First valid frame'):ep.poll()

def test_crc_valid_unknown_wrapper_cannot_create_false_source_failure():
    ep=endpoint();packet=bytes.fromhex('fd0100000a01019b01003d53b7')
    ep.port.in_waiting=len(packet);ep.port.read=lambda size:packet
    ep.poll()
    assert ep.sync.first_valid_ns is None and not ep.safe()
    feed(ep,m)
    assert ep.evidence.identity==EXPECTED

def test_unknown_wrapper_bad_crc_is_still_fatal():
    ep=endpoint();packet=bytes.fromhex('fd0100000a01019b01003d53b6')
    ep.port.in_waiting=len(packet);ep.port.read=lambda size:packet
    with pytest.raises(RuntimeError,match='CRC/framing'):ep.poll()
