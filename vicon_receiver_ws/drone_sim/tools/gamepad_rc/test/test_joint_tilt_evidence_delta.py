"""Only retention of the failed pre-recording attitude pair."""
from types import SimpleNamespace as NS
import ev_delay_live_owner as mod


def test_first_failure_is_retained_after_later_good_observation(monkeypatch):
    o=object.__new__(mod.Owner)
    o.identity=lambda:True
    o.ep=NS(expected={'uid2':'synthetic'},identity_ns=100)
    o.usb=object();o.lock=NS(clock=None)
    o.mount=[0,0,0,1];o.offset=[0,0,0];o.boot_session='synthetic';o.generation='pad'
    o.physical_conditions={};o.logger=NS(report={});o.session=None;o.config={};o.report={}
    o.source_pose=lambda:({'callback_ns':150,'sequence':1},[0,0,0],[1,0,0,0],10)
    att=NS(q1=1,q2=0,q3=0,q4=0,to_dict=lambda:{'time_boot_ms':123})
    monkeypatch.setattr(mod,'proven_attitude',lambda *a:(att,140))
    values=iter([6.,1.])
    monkeypatch.setattr(mod,'attitude_comparison',lambda *a:{'gravity_axis_difference_deg':next(values)})
    assert o.observation(200).tilt_difference_deg==6
    assert o.report['first_failed_tilt_pair']['at_ns']==200
    assert o.report['first_failed_tilt_pair']['fc_received_ns']==140
    assert o.observation(300).tilt_difference_deg==1
    assert o.report['last_tilt_pair']['at_ns']==300
    assert o.report['first_failed_tilt_pair']['comparison']['gravity_axis_difference_deg']==6
