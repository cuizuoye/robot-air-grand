"""Synthetic tests for the NEW measured-layout branch only; no device access."""
import datetime as dt
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
import rigid_body_revision as revision
import ev_delay_joint_capture as live
from vicon_heading_calibration import solve_heading
from receive_only_permit import UID


def put(path, doc):
    path.write_text(json.dumps(doc));return path


@pytest.fixture
def case(tmp_path):
    stamp=1_700_000_000
    def endpoint(x,seconds):
        return dict(source_checks_passed=True,frame_ids=['map'],pose_topic='/vicon/MircoAir/MircoAir',
            source_timing=dict(samples=200,rate_hz=100,maximum_arrival_gap_ms=10),
            position_std_mm=[.1,.1,.1],last_pose=dict(position_m=[x,0,0],quaternion_xyzw=[0,0,0,1],stamp_ns=(stamp+seconds)*10**9),
            captured_at_utc=dt.datetime.fromtimestamp(stamp+seconds+.01,dt.timezone.utc).isoformat())
    docs={'start':endpoint(0,0),'end':endpoint(.25,10)}
    paths={n:tmp_path/(n+'.json') for n in ('heading','board','geometry','measurement','start','end','level','nose_up','right_up','level_rx','nose_up_rx','right_up_rx')}
    h=solve_heading(docs['start'],docs['end'],True)
    h.update(rigid_body_revision_id='new',source_files=[str(paths['start']),str(paths['end'])]);docs['heading']=h
    docs['board']=dict(rigid_body_revision_id='new',ready_to_send_to_fc=False,imu_offset_verified=False,marker_name='M1',board_center_in_rigid_m_candidate=[0,0,0])
    docs['geometry']=dict(markers=[dict(name='M'+str(i+1),rigid_xyz_mm=p,occluded=False,result=2) for i,p in enumerate([[70,2,2],[0,50,2],[-70,0,2],[0,-50,2],[-20,20,2]])])
    docs['measurement']=dict(board_still_centered_confirmed=True,forward_left_up_mm=[70,2,2],marker_model_name_candidate='M1')
    for i,(name,angles) in enumerate([('level',[0,0,0]),('nose_up',[0,10,0]),('right_up',[-10,0,0])]):
        docs[name]=dict(static_observation_passed=True,error=None,identity=dict(uid2=UID,firmware='1.14.3'),
            heading_candidate=str(paths['heading']),actual_sent_to_fc=0,checks={'stationary':True},
            heartbeats=[dict(armed=False)],tx_messages={'HEARTBEAT':8,'AUTOPILOT_VERSION_REQUEST':1},
            rx_evidence=dict(crc_bypass=False,path=str(paths[name+'_rx'])),
            captured_at_utc=dt.datetime.fromtimestamp(stamp+20+i*20,dt.timezone.utc).isoformat(),
            last_pair=dict(fc_boot_ms=10000+i*20000),pairs=[dict(vicon_roll_pitch_yaw_deg=angles,fc_roll_pitch_yaw_deg=angles)]*30)
        paths[name+'_rx'].write_text('synthetic RX evidence; not hardware data\n')
    for n,doc in docs.items():put(paths[n],doc)
    registry=tmp_path/'registry.json'
    manifest=tmp_path/'manifest.json'
    def refresh():
        put(registry,dict(active_revision_id='new',static_validation_completed=False,
            **{'pending_'+n+'_candidate_path':str(paths[n]) for n in ('heading','board')},
            **{'pending_'+n+'_candidate_sha256':revision.digest(paths[n]) for n in ('heading','board')}))
        put(manifest,dict(scope=revision.MEASURED_CAPTURE_SCOPE,rigid_body_revision_id='new',flight_authorized=False,calibration_approved=False,
            files={n:dict(path=str(p),sha256=revision.digest(p)) for n,p in paths.items()}))
    refresh()
    return SimpleNamespace(paths=paths,registry=registry,manifest=manifest,refresh=refresh)


def inspect(c):return revision.measured_capture_inputs(c.manifest,registry_path=c.registry)


def mutate(c,name,fn):
    d=json.loads(c.paths[name].read_text());fn(d);put(c.paths[name],d);c.refresh()


def test_new_evidence_is_capture_only_not_registry_adoption(case):
    before=case.registry.read_bytes();c=inspect(case)
    assert c['measured_layout'] and c['origin']==[0,0,0]
    assert not c['calibration_approved'] and not c['flight_authorized']
    assert case.registry.read_bytes()==before


@pytest.mark.parametrize('name,fn',[
 ('heading',lambda d:d.update(rigid_body_revision_id='old')),
 ('heading',lambda d:d.update(body_orientation_in_rigid_xyzw_candidate=[0,0,1,0])),
 ('board',lambda d:d.update(board_center_in_rigid_m_candidate=[.02,0,0])),
 ('board',lambda d:d.update(ready_to_send_to_fc=True)),
 ('measurement',lambda d:d.update(board_still_centered_confirmed=False)),
 ('geometry',lambda d:d['markers'][0].update(occluded=True)),
 ('nose_up',lambda d:d.update(static_observation_passed=False)),
 ('right_up',lambda d:[r.update(fc_roll_pitch_yaw_deg=[10,0,0]) for r in d['pairs']]),
 ('level',lambda d:d.update(heartbeats=[dict(armed=True)])),
 ('level',lambda d:d['rx_evidence'].update(crc_bypass=True)),
 ('level',lambda d:d.update(tx_messages={'ARM':1})),
])
def test_changed_evidence_rejected_even_with_refreshed_hashes(case,name,fn):
    mutate(case,name,fn)
    with pytest.raises(ValueError):inspect(case)


def test_raw_evidence_change_without_new_hash_rejected(case):
    case.paths['right_up_rx'].write_text('changed')
    with pytest.raises(ValueError,match='hash/path'):inspect(case)


def test_scope_cannot_be_promoted_to_flight(case):
    doc=json.loads(case.manifest.read_text());doc['flight_authorized']=True;put(case.manifest,doc)
    with pytest.raises(ValueError,match='scope/revision'):inspect(case)


def test_default_entry_still_uses_existing_configuration(monkeypatch):
    monkeypatch.setattr(live,'local_inputs',lambda: (_ for _ in ()).throw(ValueError('legacy gate')))
    with pytest.raises(ValueError,match='legacy gate'):live.main([])


def test_new_cli_dry_run_does_not_execute(case,monkeypatch,capsys):
    c=inspect(case)
    monkeypatch.setattr(live,'local_inputs',lambda path:(c,{}))
    monkeypatch.setattr(live,'execute',lambda *a:pytest.fail('dry run opened execution'))
    assert live.main(['--measured-layout-evidence',str(case.manifest)])==0
    assert json.loads(capsys.readouterr().out)['flight_authorized'] is False


def test_run_still_requires_physical_confirmation_before_inputs(case,monkeypatch):
    monkeypatch.setattr(live,'local_inputs',lambda *a:pytest.fail('read inputs before physical checks'))
    with pytest.raises(SystemExit):live.main(['--run','--measured-layout-evidence',str(case.manifest)])


def test_execute_rejects_manifest_change_before_devices(case,monkeypatch):
    monkeypatch.setattr(revision,'REGISTRY',case.registry)
    c=revision.measured_capture_inputs(case.manifest)
    doc=json.loads(case.manifest.read_text());doc['note']='changed after inspection';put(case.manifest,doc)
    args=SimpleNamespace(authorization_id='SYNTHETIC_ONLY',confirm_props_off_battery_disconnected_usb_only=False,
        confirm_props_installed_battery_disconnected_usb_only=True,confirm_motor_power_isolated=True,confirm_independent_usb_cutoff=True)
    with pytest.raises(ValueError,match='changed since inspection'):live.execute(args,c,{})
