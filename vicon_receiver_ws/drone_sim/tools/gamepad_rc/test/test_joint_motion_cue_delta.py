import os
import pytest
from ev_delay_joint_capture import motion_cue_ready
from ev_delay_live_owner import Owner


def test_cue_is_nonblocking_and_requires_go():
    r,w=os.pipe()
    try:
        assert not motion_cue_ready(r)
        os.write(w,b'GO\n');assert motion_cue_ready(r)
        os.write(w,b'stop\n')
        with pytest.raises(RuntimeError,match='cancelled/invalid'):motion_cue_ready(r)
    finally:os.close(r);os.close(w)


@pytest.mark.parametrize('signal',[True,False])
def test_wait_keeps_servicing_owner_and_never_starts_on_timeout(signal):
    o=object.__new__(Owner);now=[0];calls=[];o.report={}
    o.clock=lambda:now[0];o.identity=lambda:True;o.notice=lambda *a,**k:None
    def pump():now[0]+=100_000_000;calls.append(now[0])
    o.pump_all=pump;o.motion_ready=lambda:signal and now[0]>=3_000_000_000
    o.prepare_visual_preroll=lambda gate:None  # This test isolates the existing cue wait.
    if signal:
        o.prepare_recording(None);assert now[0]==6_000_000_000  # GO plus 3s countdown before begin
    else:
        with pytest.raises(RuntimeError,match='Motion cue timeout'):o.prepare_recording(None)
    assert len(calls)>20
