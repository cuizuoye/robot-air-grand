"""Synthetic input, temporary permits and no hardware access."""
import json
import time
from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import one_home_preflight_policy as p
import preparation_authorization as auth
from test_preparation_authorization import files


def state(**changes):
    return dict(timestamp_us=10_000_000,armed=False,lockdown=True,
                manual_lockdown=False,force_failsafe=False)|changes


def test_early_single_press_is_queued_and_ack_alone_is_not_lock_proof():
    s=p.StartupHome();s.event(1,0);s.event(0,.2)
    assert s.needs_stop and not s.verified
    s.ack(.5,boot_us=9_500_000);assert not s.needs_stop and not s.verified
    assert not s.observe(state(),requested_at=1.99,now=2.,boot_ms=10100)
    assert s.observe(state(),requested_at=2.,now=2.1,boot_ms=10100)
    assert not s.status()['flight_authorized']


@pytest.mark.parametrize('bad',[dict(armed=True),dict(lockdown=False),dict(force_failsafe=True),dict(manual_lockdown=True)])
def test_bad_current_state_cannot_complete_startup(bad):
    s=p.StartupHome();s.event(1,0);s.event(0,.2);s.ack(.5,boot_us=9_500_000)
    assert not s.observe(state(**bad),requested_at=2.,now=2.1,boot_ms=10100)


def test_second_press_aborts_irreversibly_and_long_press_is_not_start():
    s=p.StartupHome();s.event(1,0);s.event(0,.2);s.ack(.5,boot_us=9_500_000)
    assert s.observe(state(),requested_at=2.,now=2.1,boot_ms=10100)
    s.event(1,3);s.event(0,3.2)
    assert s.aborted and not s.observe(state(),requested_at=4.,now=4.1,boot_ms=10100)
    s=p.StartupHome();s.event(1,0);s.event(0,1.1)
    assert s.aborted


def test_post_release_reply_cannot_reuse_a_pre_ack_source_sample():
    s=p.StartupHome();s.event(1,0);s.event(0,.2);s.ack(.5,boot_us=10_000_000)
    assert not s.observe(state(),requested_at=2.,now=2.1,boot_ms=10100)
    assert s.observe(state(timestamp_us=10_100_000),requested_at=2.,now=2.1,boot_ms=10200)


def test_post_release_future_source_state_is_not_current_proof():
    s=p.StartupHome();s.event(1,0);s.event(0,.2);s.ack(.5,boot_us=10_000_000)
    assert not s.observe(state(timestamp_us=11_000_000),requested_at=2.,now=2.1,boot_ms=10200)


def test_waiting_for_home_never_requests_shell_and_ack_is_not_proof():
    s=p.StartupHome()
    assert not s.query_due(120.,-100.)
    s.event(1,120.);s.ack(120.1,boot_us=10_000_000)
    assert not s.query_due(122.,-100.)  # Not released.
    s.event(0,120.2)
    assert not s.query_due(121.699,-100.)
    assert s.query_due(121.701,-100.)
    assert not s.query_due(121.8,121.7)
    assert not s.verified  # Query permission and an ACK never certify lock.
    s.event(1,122.)
    assert not s.query_due(125.,-100.)


@pytest.mark.parametrize('command,param',[(400,1),(400,0),(176,1),(21,0),(22,0),(185,0),(185,1),(246,1),(511,1)])
def test_unified_mode_does_not_open_actuation_or_rate_permissions(command,param):
    msg=m.MAVLink(None).command_long_encode(1,1,command,0,param,0,0,0,0,0,0)
    assert not p.allowed(msg,identity_fresh=True,authorized=True,locked_static=True,
        receiving_verified=True,restore_safe=True,home_latched=True)


def test_only_exact_extra_snapshot_command_is_added_to_old_read_policy():
    e=m.MAVLink(None);payload=b'listener vehicle_visual_odometry -n 13\n'
    msg=e.serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0'))
    assert p.allowed(msg,identity_fresh=True)
    assert not p.allowed(msg,identity_fresh=False)
    for payload in (b'commander arm\n',b'listener vehicle_visual_odometry -n 100\n'):
        assert not p.allowed(e.serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0')),identity_fresh=True)


def test_only_bounded_exact_counter_query_is_permitted():
    e=m.MAVLink(None)
    for payload,expected in ((b'uorb top -1 vehicle_visual_odometry\n',True),
                             (b'uorb top vehicle_visual_odometry\n',False),
                             (b'uorb top -1\n',False)):
        msg=e.serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0'))
        assert p.allowed(msg,identity_fresh=True)==expected
        assert not p.allowed(msg,identity_fresh=False)


@pytest.mark.parametrize('name,value',[('EKF2_EV_CTRL',11),('EKF2_EV_CTRL',2),('EKF2_EV_DELAY',65),('EKF2_EV_DELAY',0)])
def test_receive_only_scope_rejects_every_parameter_write_including_restore(name,value):
    import trs_vision_reference_policy as vision
    msg=vision.parameter_message(m.MAVLink(None),name,value)
    assert not p.allowed(msg,receive_rate_only=True,authorized=True,identity_fresh=True,
        locked_static=True,receiving_verified=True,restore_safe=True,home_latched=True)


def token(files,monkeypatch,*,approved=True):
    monkeypatch.setattr(p,'evidence_paths',auth.evidence_paths)
    files.proof.write_text(json.dumps({'rigid_body_revision_id':'offline-revision'}))
    usb=files.root/'usb-pass.json'
    usb.write_text(json.dumps(dict(ground_fusion_verified=True,parameter_restoration_verified=True,
        telemetry_restoration_verified=True,runtime=dict(failure=None,mission_inputs={'rigid_revision':'offline-revision'}))))
    d=p.draft(files.run,files.heading,files.board,files.proof,files.backup,usb)
    now=time.time();d.update(user_authorized=approved,physical_confirmed=approved,issued_unix=now-1,expires_unix=now+120)
    path=files.root/'one-home-permit.json';path.write_text(json.dumps(d))
    return path


def consume(files,path):
    return p.consume(path,backup=files.backup,output=files.run/'report.json',heading=files.heading,board=files.board,baud=115200)


def test_one_use_scope_preserves_holds_and_rejects_unapproved_draft(files,monkeypatch):
    path=token(files,monkeypatch,approved=False)
    with pytest.raises(ValueError):consume(files,path)
    assert not Path(str(path)+'.consumed.json').exists()
    path=token(files,monkeypatch)
    record=consume(files,path)
    assert record['scope']==p.SCOPE and not record['flight_authorized']
    assert json.loads(files.incident.read_text())['block_active_benches']
    assert json.loads(files.hold.read_text())['block_props_off_benches']
    with pytest.raises(FileExistsError):consume(files,path)


def test_permit_is_bound_to_current_code_backup_and_usb_pass(files,monkeypatch):
    path=token(files,monkeypatch);files.board.write_text('{"changed": true}')
    with pytest.raises(ValueError,match='hashes'):consume(files,path)
    assert not Path(str(path)+'.consumed.json').exists()


def test_compact_permit_is_distinct_bound_and_records_its_actual_scope(files,monkeypatch):
    path=token(files,monkeypatch)
    record=json.loads(path.read_text())
    record.update(scope=p.COMPACT_SCOPE,compact_ground_fusion=True)
    path.write_text(json.dumps(record))
    result=consume(files,path)
    assert result['compact_ground_fusion'] and result['scope']==p.COMPACT_SCOPE
    assert not result['flight_authorized']
    assert json.loads(Path(str(path)+'.consumed.json').read_text())['scope']==p.COMPACT_SCOPE


def test_compact_flag_cannot_reuse_original_permit_scope(files,monkeypatch):
    path=token(files,monkeypatch);record=json.loads(path.read_text())
    record['compact_ground_fusion']=True;path.write_text(json.dumps(record))
    with pytest.raises(ValueError,match='scope'):consume(files,path)


def test_binary_scope_is_explicit_and_seals_decoder_and_rate_code(files,monkeypatch):
    path=token(files,monkeypatch);record=json.loads(path.read_text())
    record.update(scope=p.BINARY_SCOPE,compact_ground_fusion=True,short_status=True,binary_diagnostics=True)
    path.write_text(json.dumps(record))
    result=consume(files,path)
    assert result['binary_diagnostics'] and result['scope']==p.BINARY_SCOPE
    assert 'one_home/fusion_binary_receiver.py' in result['hashes']
    assert 'one_home/parameter_snapshot.py' in result['hashes']
    assert not result['flight_authorized']


def test_low_bandwidth_scope_cannot_reuse_binary_permit(files,monkeypatch):
    path=token(files,monkeypatch);record=json.loads(path.read_text())
    record.update(scope=p.BINARY_SCOPE,compact_ground_fusion=True,short_status=True,
        binary_diagnostics=True,low_bandwidth_vision=True)
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError,match='scope'):consume(files,path)
    record['scope']=p.LOW_BANDWIDTH_SCOPE;path.write_text(json.dumps(record))
    result=consume(files,path)
    assert result['low_bandwidth_vision']
    assert 'one_home/low_bandwidth_vision.py' in result['hashes']
    assert not result['flight_authorized']


def test_low_bandwidth_cli_is_opt_in_offline_and_never_authorizes_flight(capsys):
    import drone_preflight as cli
    with pytest.raises(SystemExit):cli.main(['--low-bandwidth-vision'])
    assert cli.main(['--compact-ground-fusion','--short-status','--binary-diagnostics',
        '--low-bandwidth-vision'])==0
    report=json.loads(capsys.readouterr().out)
    assert report['low_bandwidth_vision'] and report['pose_wire_bytes']==48
    assert report['devices_opened']==0 and not report['flight_authorized']


def test_frozen_target_plan_has_distinct_load_limits_and_no_live_access(monkeypatch,capsys):
    import drone_preflight as cli
    monkeypatch.setattr(cli,'prerequisites',lambda *a:pytest.fail('Offline plan cannot open devices'))
    with pytest.raises(SystemExit):cli.main(['--frozen-target-stream'])
    assert cli.main(['--compact-ground-fusion','--short-status','--binary-diagnostics',
        '--low-bandwidth-vision','--frozen-target-stream'])==0
    r=json.loads(capsys.readouterr().out)
    assert r['frozen_target_stream'] and r['target_rate_hz']==16 and r['target_duration_s']==3
    assert r['temporary_intervals_us']=={'32':50000,'248':500000,'252':200000}
    assert r['devices_opened']==0 and not r['flight_authorized']


def test_frozen_target_permit_cannot_reuse_old_scope_or_changed_prior(files,monkeypatch):
    import frozen_target_stream as frozen
    path=token(files,monkeypatch)
    prior=files.root/'prior';prior.mkdir()
    for name in ('report.json','permit.json','permit.json.consumed.json','recovery.json','rates_recovery.json'):
        (prior/name).write_text('{"needs_restore":false,"verified_closed":true}' if name in ('recovery.json','rates_recovery.json') else '{}')
    # Proof validation has independent tests; exercise scope + hash binding here.
    monkeypatch.setattr(frozen,'require_prior',lambda *a:None)
    old=json.loads(path.read_text())
    kwargs=dict(compact_ground_fusion=True,short_status=True,binary_diagnostics=True,
        low_bandwidth_vision=True,frozen_target_stream=True,wireless_proof=prior/'report.json')
    d=p.draft(files.run,files.heading,files.board,files.proof,files.backup,old['usb_proof'],**kwargs)
    d.update(user_authorized=True,physical_confirmed=True,issued_unix=old['issued_unix'],expires_unix=old['expires_unix'])
    d['scope']=p.LOW_BANDWIDTH_SCOPE;path.write_text(json.dumps(d))
    with pytest.raises(ValueError,match='scope'):consume(files,path)
    d['scope']=p.FROZEN_TARGET_SCOPE;path.write_text(json.dumps(d))
    (prior/'report.json').write_text('{"changed":true}')
    with pytest.raises(ValueError,match='hashes'):consume(files,path)
    (prior/'report.json').write_text('{}')
    r=consume(files,path)
    assert r['scope']==p.FROZEN_TARGET_SCOPE and r['frozen_target_stream']
    assert 'wireless_static_report' in r['hashes'] and not r['flight_authorized']


def test_cli_diagnostic_scopes_are_mutually_exclusive():
    import drone_preflight as cli
    with pytest.raises(SystemExit):cli.main(['--receive-rate-only','--compact-ground-fusion'])


def test_offline_binary_plan_never_requires_live_maintenance_or_hardware(monkeypatch,capsys):
    import drone_preflight as cli
    import serial
    monkeypatch.setattr(cli,'prerequisites',lambda *a:pytest.fail('Offline plan cannot demand fresh maintenance'))
    monkeypatch.setattr(serial,'Serial',lambda *a,**k:pytest.fail('Offline plan opened hardware'))
    assert cli.main(['--compact-ground-fusion','--short-status','--binary-diagnostics'])==0
    report=json.loads(capsys.readouterr().out)
    assert report['devices_opened']==0 and not report['prerequisites_checked']
    assert report['temporary_intervals_us']=={'252':200000,'248':500000}
    assert report['requires_matching_installed_binary_candidate']


def test_binary_real_entry_refuses_uninstalled_candidate_before_any_port(monkeypatch,tmp_path,capsys):
    import drone_preflight as cli
    monkeypatch.setattr(cli,'BINARY_INSTALLATION',tmp_path/'not-installed.json')
    monkeypatch.setattr(cli,'require_port_free',lambda *a:pytest.fail('No port access without installed candidate'))
    monkeypatch.setattr(cli,'prerequisites',lambda *a:pytest.fail('No live prerequisites before installation check'))
    assert cli.main(['--run-battery','--legacy-dynamic-ekf','--compact-ground-fusion','--short-status','--binary-diagnostics',
                     '--confirm-protected-stationary-battery-only'])==2
    assert '尚无安装凭据' in capsys.readouterr().out


def test_binary_installation_matches_uploaded_package_and_uid(monkeypatch,tmp_path):
    import drone_preflight as cli
    import hashlib
    package=tmp_path/'candidate.px4';package.write_bytes(b'synthetic candidate')
    receipt=tmp_path/'installation.json'
    monkeypatch.setattr(cli,'BINARY_CANDIDATE',package);monkeypatch.setattr(cli,'BINARY_INSTALLATION',receipt)
    data=dict(firmware_flashed=True,application_crc_verified_by_uploader=True,
              candidate_sha256=hashlib.sha256(package.read_bytes()).hexdigest(),error=None,
              bootloader={'uid':p.UID2})
    receipt.write_text(json.dumps(data));cli.require_binary_installation()
    data['application_crc_verified_by_uploader']=False;receipt.write_text(json.dumps(data))
    with pytest.raises(ValueError):cli.require_binary_installation()


def test_incomplete_restore_stays_durably_blocking(tmp_path):
    run=tmp_path/'run';run.mkdir();journal=p.PoweredRecoveryJournal(run/'recovery.json')
    journal(dict(name='EKF2_EV_CTRL',value=11))
    assert not journal.finish({'EKF2_EV_CTRL'},{},None)
    with pytest.raises(ValueError,match='recovery'):auth.require_no_pending_recovery(tmp_path)
    assert journal.finish({'EKF2_EV_CTRL'},{'EKF2_EV_CTRL':True},state())
    auth.require_no_pending_recovery(tmp_path)


def test_cli_default_and_missing_physical_confirmation_never_open_hardware(monkeypatch,tmp_path):
    import drone_preflight as cli
    import serial
    monkeypatch.setattr(cli,'prerequisites',lambda *a:({'revision':'test'},tmp_path/'backup',tmp_path/'proof'))
    monkeypatch.setattr(serial,'Serial',lambda *a,**k:pytest.fail('Unexpected real serial open'))
    assert cli.main([])==0
    assert cli.main(['--run-battery'])==2
    assert cli.main(['--run-battery','--legacy-dynamic-ekf'])==2


def test_cli_rejects_usb_present_even_with_battery_confirmation(monkeypatch,tmp_path):
    import drone_preflight as cli
    monkeypatch.setattr(cli,'prerequisites',lambda *a:({'revision':'test'},tmp_path/'backup',tmp_path/'proof'))
    usb=tmp_path/'usb';usb.touch();monkeypatch.setattr(cli,'FC_USB',usb)
    monkeypatch.setattr(cli,'require_port_free',lambda *a:pytest.fail('No hardware inspection before scope check'))
    assert cli.main(['--run-battery','--legacy-dynamic-ekf','--confirm-protected-stationary-battery-only'])==2


@pytest.mark.parametrize('value',['0','9','301','nan','inf'])
def test_home_wait_budget_is_bounded_without_opening_devices(value,monkeypatch):
    import drone_preflight as cli
    monkeypatch.setattr(cli,'prerequisites',lambda *a:pytest.fail('No config/hardware before budget validation'))
    with pytest.raises(SystemExit):cli.main(['--home-wait-seconds',value])
