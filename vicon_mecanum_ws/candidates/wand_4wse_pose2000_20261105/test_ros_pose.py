"""Real isolated ROS + loopback exact-C peer; zero access to the real car.

Inject loss of the original Sign/car source while commands and feedback keep
their original deadlines. This does not renew either lost source timestamp.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time

HERE = Path(__file__).resolve().parent
HOST = HERE / 'host'
sys.path.insert(0, str(HOST))
import bootstrap
from ground_entry import profile
from session_channel import Channel


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('case', choices=('wand_short', 'car_short', 'both_short',
                                        'wand_long', 'car_long', 'both_long', 'loop_gap'))
    args = parser.parse_args()
    folder = Path(tempfile.mkdtemp(prefix='ros_' + args.case + '_', dir=HERE / 'evidence'))
    path, consent = profile(HOST, folder, True, seconds=20.)
    p = json.loads(path.read_text())
    assert p['hardware']['esp_ip'] == '127.0.0.1' and p['hardware']['network_interface'] == 'lo'
    assert not {46000, 46001}.intersection({p['hardware']['command_port'], p['hardware']['telemetry_port']})
    os.environ.update(WAND_4WSE_MODE='rehearsal', WAND_4WSE_PROFILE=str(path),
        WAND_PROFILE_APPROVAL=hashlib.sha256(path.read_bytes()).hexdigest(),
        WAND_HARDWARE_CONSENT=consent, PYTHONDONTWRITEBYTECODE='1',
        ROS_DOMAIN_ID='198', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    from ground_runtime import GroundRig
    channel = Channel(readonly=True)
    rig = None
    report = dict(passed=False, case=args.case, physical_output=False,
                  hardware_socket_opened=False, real_vicon_started=False)
    injected = restored = stop_seen = None
    active_while_gap = 0
    reason = ''
    stop = None
    try:
        rig = GroundRig(bootstrap.runtime(), channel, folder)
        assert not rig.physical and rig.rt.domain == 198 and rig.peer is not None
        assert rig.rt.config.pose_timeout_s == 2.
        channel.open('offline-pose-fault-injection', time.monotonic())
        began = time.monotonic()
        start_sent = False
        while time.monotonic() - began < 25.:
            now = time.monotonic()
            channel.begin_tick(now)
            channel.backend_tick(now, now)
            if channel.ready and not start_sent:
                offer = channel.challenge(now)
                if offer:
                    assert channel.receive(channel.owner, dict(type='command', action='start', nonce=offer['nonce'], sequence=1), now)
                    start_sent = True
            channel.finish_tick(time.monotonic())
            rig.step(time.monotonic())
            now = time.monotonic()
            if rig.active_since is not None and now - rig.active_since >= .5 and injected is None:
                injected = now
                if args.case == 'loop_gap':
                    time.sleep(.15)
                else:
                    rig.occlude = args.case.startswith(('wand', 'both'))
                    rig.drop_car = args.case.startswith(('car', 'both'))
            if injected is not None:
                age = now - injected
                if rig.status.get('state') == 'ACTIVE' and age > .3:
                    active_while_gap += 1
                if args.case.endswith('short') and restored is None and age >= 1.5:
                    rig.occlude = rig.drop_car = False
                    restored = now
                if channel.stopped or rig.status.get('state') in ('STOPPING', 'FAULT', 'STOP_UNCONFIRMED'):
                    stop_seen = now
                    reason = channel.fault or rig.status.get('first_stop_reason') or rig.status.get('reason')
                    break
                if restored is not None and now - restored >= 1.:
                    reason = 'OFFLINE_OPERATOR_STOP'
                    channel.stop(reason)
                    break
            time.sleep(.003)
        assert rig.activation_ever, 'simulation did not become ACTIVE'
        assert injected is not None
        if args.case.endswith('short'):
            assert restored is not None and stop_seen is None, (reason, rig.status, rig.gate)
            assert active_while_gap > 10, 'no continued ACTIVE samples during source gap'
        else:
            assert stop_seen is not None, 'no stop after expiry'
            delay = stop_seen - injected
            if args.case == 'loop_gap':
                assert delay < .4
            else:
                assert 1.80 <= delay <= 2.20, (delay, reason)
        report.update(passed=True, active_ever=True, original_pose_timestamps_preserved=True,
            gap_s=1.5 if restored is not None else None,
            stop_delay_s=None if stop_seen is None else stop_seen - injected,
            active_samples_during_gap=active_while_gap, reason=reason,
            pose_timeout_s=2., feedback_timeout_s=.5, command_timeout_s=.1,
            ros_domain=rig.rt.domain, real_ros_and_exact_c_peer=True)
    except BaseException as error:
        report.update(passed=False, error=repr(error))
        raise
    finally:
        if rig is not None:
            try:
                stop = rig.close()
            except BaseException as error:
                report.update(passed=False, close_error=repr(error))
                rig.dispose()
        report['stop'] = stop
        if not stop or not stop.get('gate_output_zero'):
            report['passed'] = False
        (folder / 'RESULT.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(dict(case=args.case, passed=report['passed'], reason=reason,
                             folder=str(folder), stop_delay_s=report.get('stop_delay_s'))), flush=True)
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
