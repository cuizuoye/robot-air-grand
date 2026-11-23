from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mission_telemetry_rates import TemporaryTelemetryRates,CANDIDATES
from mavlink_crc_stream import CrcResyncParser


def received(msg):
    fc=m.MAVLink(None,srcSystem=1,srcComponent=1)
    return CrcResyncParser().parse_buffer(msg.pack(fc))[0]


def exchange(rates,now,values,*,locked=True,refuse=None):
    msg=rates.action(now,fresh=True,locked=locked)
    if msg is None:return None
    assert rates.permits(msg,fresh=True,locked=locked)
    fc=m.MAVLink(None)
    key=int(msg.param1)
    if msg.command==510:
        response=fc.message_interval_encode(interval_us=values[key],message_id=key)
    else:
        if key!=refuse:values[key]=int(msg.param2) if msg.param2>0 else 0
        response=fc.command_ack_encode(511,0,target_system=250,target_component=202)
    rates.ingest(received(response),now)
    return msg


def active(rates=None):
    rates=rates or TemporaryTelemetryRates()
    values={32:1_000_000,230:2_000_000,245:0}
    now=0.;sent=[]
    for _ in range(100):
        msg=exchange(rates,now,values)
        if msg:sent.append(msg)
        now=round(now+.05,5)
        if rates.phase=='ACTIVE':break
    assert rates.phase=='ACTIVE'
    return rates,values,now,sent


def test_originals_candidates_and_exact_restore_with_same_owner_and_journal():
    saved=[];r=TemporaryTelemetryRates(persist=lambda record:saved.append(record))
    r,values,now,sent=active(r)
    assert [msg.command for msg in sent].count(511)==3
    for key in CANDIDATES:
        assert any(key in record['possibly_changed'] for record in saved)
    assert r.original=={32:1_000_000,230:2_000_000,245:0}
    r.begin_restore(now)
    for _ in range(100):
        exchange(r,now,values);now=round(now+.05,5)
        if r.phase=='CLOSED':break
    assert values==r.original and r.summary()['restoration_verified']
    assert r.summary()['heartbeat_rate_writes']==0


@pytest.mark.parametrize('original',[0,100000,1000000])
def test_sync_profile_only_changes_needed_interval_and_never_sets_while_airborne(original):
    saved=[];r=TemporaryTelemetryRates(profile='timesync',persist=saved.append)
    values={111:original};now=0.;sent=[]
    for _ in range(50):
        msg=exchange(r,now,values)
        if msg:sent.append(msg)
        now+=.05
        if r.phase=='ACTIVE':break
    assert r.phase=='ACTIVE' and values=={111:100000}
    assert all(msg.param1==111 for msg in sent)
    assert sum(msg.command==511 for msg in sent)==(original!=100000)
    if original!=100000:
        assert any(v['possibly_changed']==[111] and v['original_intervals_us']=={111:original} for v in saved)
    r.begin_restore(now)
    assert r.action(now,fresh=True,locked=False) is None
    for _ in range(50):
        exchange(r,now,values);now+=.05
        if r.phase=='CLOSED':break
    assert values=={111:original} and r.summary()['restoration_verified']


@pytest.mark.parametrize('field,value',[('param1',0),('param2',50000),('param3',1),
    ('command',400),('target_system',2),('confirmation',1)])
def test_sync_rate_mutated_candidate_cannot_escape_pinned_shape(field,value):
    r=TemporaryTelemetryRates(profile='timesync')
    r.original={111:0};r.phase='SET_NEXT'
    msg=r.action(0,fresh=True,locked=True)
    setattr(msg,field,value)  # Even the producer's mutable object is checked.
    assert not r.permits(msg,fresh=True,locked=True)


def test_no_lock_still_allows_read_originals_but_never_set():
    r=TemporaryTelemetryRates();values={key:1_000_000 for key in CANDIDATES};sent=[]
    for i in range(10):
        msg=exchange(r,i*.1,values,locked=False)
        if msg:sent.append(msg)
    assert len(sent)==3 and all(msg.command==510 for msg in sent)
    assert not r.changed and r.phase=='SET_NEXT'


def test_ack_alone_does_not_prove_candidate_rate():
    r=TemporaryTelemetryRates();values={key:1_000_000 for key in CANDIDATES}
    for i in range(20):
        exchange(r,i*.1,values,refuse=32)
        if r.phase=='BLOCKED':break
    assert r.error=='interval_readback_mismatch_32'
    assert r.changed==[32]


def test_missing_ack_never_retries_set_but_can_restore_possible_change():
    r=TemporaryTelemetryRates();values={key:1_000_000 for key in CANDIDATES}
    for i in range(3):exchange(r,i*.1,values)
    msg=r.action(.3,fresh=True,locked=True);assert msg.command==511
    r.action(3.31,fresh=True,locked=True)
    assert r.phase=='BLOCKED' and r.changed==[32]
    r.begin_restore(3.4)
    assert r.action(3.4,fresh=True,locked=True).param2==1_000_000


def test_only_current_exact_action_has_send_permission():
    r,_,now,_=active()
    e=m.MAVLink(None)
    for command in (176,185,400,21,22,246,511):
        assert not r.permits(e.command_long_encode(1,1,command,0,0,1,0,0,0,0,0),fresh=True,locked=True)
    r.begin_restore(now);msg=r.action(now,fresh=True,locked=True)
    assert not r.permits(msg,fresh=False,locked=True)
    assert not r.permits(msg,fresh=True,locked=False)


def test_late_or_wrong_source_reply_never_proves_original_interval():
    r=TemporaryTelemetryRates();r.action(0,fresh=True,locked=True)
    e=m.MAVLink(None,srcSystem=2,srcComponent=1)
    msg=e.message_interval_encode(interval_us=1_000_000,message_id=32)
    wrong=m.MAVLink(None).parse_char(msg.pack(e))
    r.ingest(wrong,.1);assert not r.original
    r.ingest(received(msg),3.1);assert not r.original


def test_same_batch_and_boundary_silence_cannot_falsely_pass_rate_measurement():
    r,_,now,_=active();fc=m.MAVLink(None)
    msg=received(fc.local_position_ned_encode(1,0,0,0,0,0,0))
    r.ingest(msg,now);r.ingest(msg,now)
    assert not r.summary()['measured_profile_verified']  # No division-by-zero on a batch.
    r.begin_restore(now+6)
    assert not r.summary()['measured_profile_verified']
    assert r.summary()['observed']['32']['max_gap_including_window_boundaries_ms']>=6000


def test_each_restore_stream_attempted_even_after_one_timeout():
    r,values,now,_=active();r.begin_restore(now)
    first=r.action(now,fresh=True,locked=True);assert int(first.param1)==245
    r.action(now+3.1,fresh=True,locked=True)
    next_msg=r.action(now+3.2,fresh=True,locked=True)
    assert int(next_msg.param1)==230 and r.restored[245] is False
    r.restoration_incomplete('operator_interrupted')
    assert not r.summary()['restoration_verified']


def test_mission_profile_reuses_feedback_rates_skips_unchanged_and_restores_exactly():
    r=TemporaryTelemetryRates(profile='mission')
    originals={245:1_000_000,32:100000,230:200000,111:0}
    values=dict(originals);sent=[];now=0.
    for _ in range(100):
        msg=exchange(r,now,values)
        if msg:sent.append(msg)
        now+=.05
        if r.phase=='ACTIVE':break
    assert r.phase=='ACTIVE' and values=={**CANDIDATES,111:100000}
    assert r.changed==[245,111]  # Existing matching feedback is not rewritten.
    assert [int(msg.param1) for msg in sent if msg.command==511]==[245,111]
    r.begin_restore(now)
    for _ in range(100):
        msg=exchange(r,now,values)
        if msg:sent.append(msg)
        now+=.05
        if r.phase=='CLOSED':break
    assert values==originals and r.summary()['restoration_verified']
    assert [int(msg.param1) for msg in sent if msg.command==511]==[245,111,111,245]
