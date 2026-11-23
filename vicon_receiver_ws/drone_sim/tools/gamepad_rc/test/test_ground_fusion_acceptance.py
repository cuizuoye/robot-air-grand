"""Distinct USB-ground-only proof; realistic 2Hz actor stamps, no flight path."""
from dataclasses import replace
import re
import json
import time

import pytest

from test_preparation_temporary_rates import RatePeer
from test_preparation_authorization import files,approved
from test_trs_vision_reference import baseline
from mission_fusion_service import context_from_inputs
import preparation_authorization as authorization
from prepare_drone import HostSession
from test_preparation_host import DrivenIO


class SlowActorPeer(RatePeer):
    def shell_text(self,text):
        if 'TOPIC: actuator_armed' in text:
            stamp=int((int((self.now+10)*2)/2)*1e6)
            text=re.sub(r'timestamp: \d+',f'timestamp: {stamp}',text)
        super().shell_text(text)


def test_ground_profile_is_separate_and_cannot_mark_ready_or_handoff():
    p=SlowActorPeer(ground=True);p.rates_ready();r=p.runtime
    c=replace(r.context(),actuator_age=.75)
    assert c.ground_status_max_age==1. and c.locked_ground()
    assert not replace(c,ground_status_max_age=.5).locked_ground()
    assert context_from_inputs(r.inputs,p.now).ground_status_max_age==.5
    for method,args in ((r.coordinator.fusion.mark_ready,(c,r.inputs)),
                        (r.coordinator.fusion.handoff,(c,r.coordinator.core))):
        with pytest.raises(PermissionError):method(*args)


def test_two_hz_actor_ground_fusion_acceptance_and_exact_restore_without_handoff():
    p=SlowActorPeer(ground=True);p.rates_ready();r=p.runtime
    r.request_prepare(operator_confirmed=True)
    session=HostSession(r,DrivenIO(p),prepare=True)
    for _ in range(800):
        status=session.turn(round(p.now+.032,9),0)
        if status['done']:break
    assert status['done'] and status['ground_fusion_verified'],status
    assert not status['prepared'] and r.closed
    assert r.coordinator.core.phase!='WAIT_EXPLICIT_START'
    assert r.coordinator.fusion.transferred_core is None
    assert p.values==p.original and p.params==baseline()
    assert len(r.coordinator.ground_audits)==3 and not r.inputs.audit_rows
    assert r.coordinator.diagnostic_history


def test_strict_permit_cannot_be_consumed_by_ground_exception(files):
    p=SlowActorPeer(ground=True);p.rates_ready();path=approved(files)
    with pytest.raises(ValueError,match='scope'):
        authorization.consume(path,p.runtime,files.run,files.heading,files.board,files.proof,files.backup)


def test_ground_profile_never_accepts_stale_pose_or_unlocked_state():
    p=SlowActorPeer(ground=True);p.rates_ready();c=p.runtime.context()
    assert not replace(c,source_age=.151).locked_ground()
    assert not replace(c,actuator_age=1.001).locked_ground()
    assert not replace(c,attitude_age=.501).locked_ground()
    assert not replace(c,lockdown=False).locked_ground()
    assert not replace(c,armed=True).locked_ground()


def test_ground_source_loss_during_visual_wait_can_restore_without_restarting_pose():
    p=SlowActorPeer(ground=True);p.rates_ready();r=p.runtime
    r.request_prepare(operator_confirmed=True)
    for _ in range(60):  # May first wait for the next real 2Hz actor publication.
        p.turn()
        if p.visual_wait is not None:break
    assert p.visual_wait is not None
    for _ in range(6):p.turn(with_pose=False)
    assert r.failure and not r.coordinator.fusion.changed
    count=r.sent_counts['VISION_POSITION_ESTIMATE'];r.request_close()
    for _ in range(160):
        p.turn(with_pose=False)
        if r.closed:break
    assert r.closed and p.values==p.original and p.params==baseline()
    assert r.sent_counts['VISION_POSITION_ESTIMATE']==count and r.failure


def test_ground_fusion_rows_are_not_replayed_or_combined_across_estimators():
    p=SlowActorPeer(ground=True);p.rates_ready();r=p.runtime
    r.request_prepare(operator_confirmed=True)
    for _ in range(500):
        p.turn()
        if r.coordinator.ground_verified(p.now):break
    assert r.coordinator.ground_verified(p.now)
    rows=r.coordinator.ground_audits
    assert not r.coordinator.ground_verified(p.now+2.01)
    assert len(rows)==3 and not r.inputs.audit_rows


def test_ground_dispatch_requires_distinct_profile_bound_permit(files):
    p=SlowActorPeer(ground=True);p.rates_ready();r=p.runtime;r.request_prepare(operator_confirmed=True)
    for _ in range(500):
        p.turn()
        if r.coordinator.ground_verified(p.now):break
    d=authorization.draft(files.run,files.heading,files.board,files.proof,files.backup,ground_fusion_only=True)
    now=time.time();d.update(user_authorized=True,issued_unix=now-1,expires_unix=now+120,
        physical_confirmations=dict(battery_disconnected=True,usb_only=True,stationary=True,props_treated_as_installed=True))
    token=files.root/'ground.json';token.write_text(json.dumps(d))
    grant=authorization.consume(token,r,files.run,files.heading,files.board,files.proof,files.backup,clock=lambda:p.now)
    from test_mission_preparation_io import decode
    frame=next(row for row in reversed(r.frames) if row['type']=='VISION_POSITION_ESTIMATE')
    assert frame['at']==r.now
    r.inputs.actuator['timestamp_us']=r.inputs._boot_us(p.now)-750_000
    assert grant(r,frame,decode(bytes.fromhex(frame['wire_hex'])))
    r.ground_fusion_only=False
    assert not grant(r,frame,decode(bytes.fromhex(frame['wire_hex'])))
