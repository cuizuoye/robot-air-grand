from copy import deepcopy
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import pytest
from pymavlink.dialects.v20 import common as m
from short_status_transport import ShortStatusTransport, RateTransactionGroup
from static_lock_gate import StaticLockGate


class Harness:
    def __init__(self, original=0, message_id=252, interval_us=200000):
        self.time=1_000_000_000
        self.value=original
        self.records=[]
        self.sent=[]
        self.restoring=False
        self.drop=False
        self.fail_write=False
        self.enc=m.MAVLink(None,srcSystem=250,srcComponent=213)
        self.fc=m.MAVLink(None,srcSystem=1,srcComponent=1)
        self.transport=ShortStatusTransport(StaticLockGate(),self.enc,
                                           lambda r:self.records.append(deepcopy(r)),
                                           message_id=message_id,interval_us=interval_us)

    def clock(self):return self.time

    def pump(self):self.time+=10_000_000

    def send(self, msg):
        assert self.transport.allowed(msg,identity=True,locked=not self.restoring,restoring=self.restoring)
        self.sent.append(msg)
        if msg.command==511:
            assert self.records[-1]['possibly_changed']==[self.transport.message_id]
            assert not self.records[-1]['restoration_verified']
            if self.fail_write:raise OSError('Partial write')
            self.value=msg.param2 if msg.param2>0 else 0
            reply=self.fc.command_ack_encode(511,m.MAV_RESULT_ACCEPTED,0,0,250,213)
        else:
            reply=self.fc.message_interval_encode(self.transport.message_id,int(self.value))
        if not self.drop:
            parsed=m.MAVLink(None).parse_buffer(reply.pack(self.fc))[0]
            self.transport.ingest(parsed,self.time)

    def configure(self):self.transport.configure(self.send,self.pump,self.clock)

    def restore(self):
        self.restoring=True
        self.transport.restore(self.send,self.pump,self.clock)


@pytest.mark.parametrize('original',[0,100000,200000,1000000])
def test_exact_original_restore_and_journal_before_write(original):
    h=Harness(original);h.configure()
    assert h.value==200000 and h.transport.enabled
    assert not h.records[-1]['restoration_verified']
    h.restore()
    assert h.value==original
    assert h.records[-1]['restoration_verified'] and not h.transport.enabled
    assert h.sent[-2].param2==(original if original>0 else -1)


@pytest.mark.parametrize('original',[0,100000,1000000])
def test_same_channel_timesync_restores_original_without_own_probe(original):
    h=Harness(original,message_id=111,interval_us=100000)
    h.configure()
    sent=[];h.transport.probe(h.time,sent.append,identity=True,restoring=False)
    assert not sent
    h.restore()
    assert h.value==original and h.records[-1]['restoration_verified']


def test_partial_set_failure_retains_restore_obligation():
    h=Harness();h.fail_write=True
    with pytest.raises(OSError):h.configure()
    assert h.transport.changed and not h.records[-1]['restoration_verified']
    h.fail_write=False;h.restore()
    assert h.records[-1]['restoration_verified']


def test_missing_restore_reply_does_not_claim_success():
    h=Harness();h.configure();h.drop=True
    with pytest.raises(RuntimeError):h.restore()
    assert not h.transport.restored
    assert not h.records[-1]['restoration_verified']


def test_only_pinned_rate_and_probe_are_allowed():
    h=Harness();t=h.transport;h.configure()
    def cmd(command,p1,p2):return h.enc.command_long_encode(1,1,command,0,p1,p2,0,0,0,0,0)
    for msg in [cmd(400,1,0),cmd(185,0,0),cmd(511,32,100000),cmd(511,252,0),cmd(511,252,10000)]:
        assert not t.allowed(msg,identity=True,locked=True,restoring=False)
    assert not t.allowed(cmd(511,252,200000),identity=False,locked=True,restoring=False)
    assert not t.allowed(cmd(511,252,200000),identity=True,locked=False,restoring=False)
    assert t.allowed(cmd(511,252,-1),identity=True,locked=False,restoring=True)
    token=t.lock.clock.request(2_000_000_000)
    assert t.allowed(h.enc.timesync_encode(0,token),identity=True,locked=False,restoring=False)
    assert not t.allowed(h.enc.timesync_encode(0,token+1),identity=True,locked=False,restoring=False)
    assert t.allowed(h.enc.timesync_encode(0,token),identity=True,locked=False,restoring=True)
    assert not t.allowed(cmd(511,252,200000),identity=True,locked=False,restoring=False)
    assert t.allowed(cmd(511,252,200000),identity=True,locked=False,restoring=False,bootstrap=True)


def test_no_configuration_retry_and_wrong_source_ignored():
    h=Harness();h.configure()
    with pytest.raises(RuntimeError):h.configure()
    enc=m.MAVLink(None,srcSystem=2,srcComponent=1)
    msg=m.MAVLink(None).parse_buffer(enc.message_interval_encode(252,123).pack(enc))[0]
    old=h.transport.interval
    h.transport.ingest(msg,h.clock()+1)
    assert h.transport.interval==old


@pytest.mark.parametrize('original',[0,200000,500000,1000000])
def test_binary_pinned_rate_exact_restore(original):
    h=Harness(original,message_id=248,interval_us=500000)
    h.configure();assert h.value==500000
    h.restore();assert h.value==original and h.records[-1]['restoration_verified']


def test_binary_cannot_bootstrap_or_create_another_clock_owner():
    h=Harness(message_id=248,interval_us=500000);h.configure()
    msg=h.enc.command_long_encode(1,1,511,0,248,500000,0,0,0,0,0)
    assert not h.transport.allowed(msg,identity=True,locked=False,bootstrap=True,restoring=False)
    sent=[]
    h.transport.probe(h.time,sent.append,identity=True,restoring=False)
    assert not sent
    with pytest.raises(ValueError):Harness(message_id=248,interval_us=100000)


@pytest.mark.parametrize('original',[0,100000,50000,1000000])
def test_frozen_fc_local_rate_restores_exact_original(original):
    h=Harness(original,message_id=32,interval_us=50000)
    h.configure();assert h.value==50000
    h.restore();assert h.value==original and h.records[-1]['restoration_verified']


def test_fc_local_rate_needs_real_lock_and_has_no_clock_owner():
    h=Harness(message_id=32,interval_us=50000);h.configure()
    msg=h.enc.command_long_encode(1,1,511,0,32,50000,0,0,0,0,0)
    assert not h.transport.allowed(msg,identity=True,locked=False,bootstrap=True,restoring=False)
    sent=[];h.transport.probe(h.time,sent.append,identity=True,restoring=False)
    assert not sent
    with pytest.raises(ValueError):Harness(message_id=32,interval_us=100000)


def test_group_restoring_one_stream_cannot_hide_other_obligation():
    states=[];group=RateTransactionGroup(lambda summary:states.append(deepcopy(summary)))
    lock=Harness();fusion=Harness(message_id=248,interval_us=500000)
    def attach(harness,name):
        record=group.member(name)
        def callback(summary):
            harness.records.append(deepcopy(summary));record(summary)
        harness.transport.journal=callback
    attach(lock,'lock');attach(fusion,'fusion')
    lock.configure();fusion.configure()
    assert set(states[-1]['original_intervals_us'])=={248,252}
    fusion.drop=True
    with pytest.raises(RuntimeError):fusion.restore()
    lock.restore()
    assert not states[-1]['restoration_verified']
    assert states[-1]['possibly_changed']==[248,252]


def test_probe_opportunities_do_not_extend_any_evidence_limit():
    h=Harness();h.configure();sent=[];t=h.transport
    start=h.time
    t.probe(start,sent.append,identity=True,restoring=False)
    assert len(sent)==1 and t.next_probe_ns==start+100_000_000
    # At the exact RTT boundary the prior token still owns the request.
    t.probe(start+100_000_000,sent.append,identity=True,restoring=False)
    assert len(sent)==1
    assert t.next_probe_ns<=start+101_000_000
    t.probe(start+101_000_000,sent.append,identity=True,restoring=False)
    assert len(sent)==2 and sent[0].ts1!=sent[1].ts1
    assert t.lock.clock.MAX_RTT_NS==100_000_000
    assert t.lock.clock.MAX_AGE_NS==500_000_000
