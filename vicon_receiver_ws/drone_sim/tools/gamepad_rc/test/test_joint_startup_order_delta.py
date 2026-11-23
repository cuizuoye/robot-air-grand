"""Only the new startup ordering; no serial devices or old test collection."""
from types import SimpleNamespace as NS
import pytest
from ev_delay_joint_capture import request_startup_versions


@pytest.mark.parametrize('ready',['both','no_v2','no_heartbeat','stale'])
def test_single_query_only_after_current_peer_evidence(ready):
    now=[0];events=[];endpoints=[]
    for name in ('usb','trs'):
        endpoints.append(NS(heartbeat_ns=None,v2_fc_received_ns=None,fault=None,
            heartbeat=lambda name=name:events.append(('hb',name)),
            version=lambda name=name:events.append(('version',name,now[0]))))
    def pump():
        now[0]+=100_000_000
        for e in endpoints:
            e.heartbeat_ns=None if ready=='no_heartbeat' else now[0]
            e.v2_fc_received_ns=(None if ready=='no_v2' or now[0]<300_000_000
                                 else now[0]-600_000_000 if ready=='stale' else now[0])
    if ready=='both':
        request_startup_versions(endpoints,pump,clock=lambda:now[0])
        assert events==[('hb','usb'),('hb','trs'),('version','usb',300_000_000),('version','trs',300_000_000)]
    else:
        with pytest.raises(RuntimeError,match='before identity query'):
            request_startup_versions(endpoints,pump,clock=lambda:now[0])
        assert events==[('hb','usb'),('hb','trs')]
