#!/usr/bin/env python3
"""Offline-only continuous flight integration host. No real execution mode."""
import argparse
import json
from pathlib import Path

from synthetic_mission_peer import SyntheticPeer, synthetic_baseline


SCENARIOS = ('nominal','start_withheld','pose_loss','gamepad_loss','home_stop',
    'link_loss','selector_change','fc_reboot','offboard_rejected','arm_rejected',
    'enable_ack_lost','restore_echo_lost','climb_timeout','landing_timeout','cancel','horizontal_push','pose_jump')


def simulate(scenario, *, height=.3, hover_seconds=3.):
    if scenario not in SCENARIOS:
        raise ValueError('Unknown synthetic scenario')
    peer = SyntheticPeer(height=height,hover_seconds=hover_seconds,scenario=scenario)
    disturbance_remaining=0.
    result = peer.prepare()
    prepared = bool(not result['failure'] and peer.runtime.coordinator.fusion.phase == 'HANDED_OFF')
    prepared_at=peer.now;waiting_preserved=False
    if prepared and scenario != 'start_withheld':
        peer.runtime.request_start(operator_confirmed=True)
    for _ in range(10000):
        if not prepared:
            break
        if scenario=='start_withheld' and not waiting_preserved and peer.now-prepared_at>=65.:
            waiting_preserved=peer.runtime.core.phase=='WAIT_EXPLICIT_START' and not peer.runtime.failure
            # No independent 60 s runtime cutoff. The offline operator ends
            # the wait explicitly; real preparation is bounded by its review.
            peer.runtime.request_cancel()
        if scenario in ('horizontal_push','pose_jump') and peer.runtime.core.phase=='HOVER' and not peer.injected:
            peer.injected=True;peer.injection_at=peer.now
            if scenario=='pose_jump':peer.x+=1.  # Discontinuous localization must abort.
            else:disturbance_remaining=.12
        if disturbance_remaining>0:
            # Bounded 1 m/s synthetic push, not a 12 cm teleport in 10 ms.
            # Never a request to physically push a flying aircraft.
            step=min(disturbance_remaining,peer.dt)
            peer.x+=step;disturbance_remaining-=step
        if (peer.runtime.core.phase == 'ASCEND' and not peer.injected
                and peer.now-peer.runtime.core.ascend_at >= .6 and scenario in (
                'pose_loss','gamepad_loss','home_stop','link_loss','selector_change','fc_reboot','cancel')):
            peer.injected = True
            peer.injection_at = peer.now
            if scenario == 'fc_reboot':
                peer.boot_offset -= 5
            elif scenario == 'cancel':
                peer.runtime.request_cancel()
            elif scenario == 'home_stop':
                result = peer.turn(home=(True,))
                peer.turn(home=(False,))
        result = peer.turn()
        if result['closed']:
            break
        phase = peer.runtime.core.phase
        if scenario == 'home_stop' and peer.injected and peer.now-peer.injection_at > 1:
            break
        if phase == 'LAND_TIMEOUT':
            break
        if peer.runtime.coordinator.fusion.phase == 'BLOCKED':
            break
    core = peer.runtime.core
    commands = [msg.command for _,msg in peer.calls if msg.get_type() == 'COMMAND_LONG']
    phases = list(dict.fromkeys(row['phase'] for row in peer.state_rows))
    # The report is explicit about what the artificial peer did, not a claim
    # that the real aircraft would comply with these commands or dynamics.
    if scenario in ('nominal','horizontal_push'):
        passed = prepared and result['closed'] and core.phase == 'COMPLETE' and core.fault is None
        if scenario=='horizontal_push':passed &= abs(peer.x-1.)<.02 and peer.injected
    elif scenario == 'start_withheld':
        passed = (prepared and not peer.runtime.enable_sent and not core.arm_attempted
                  and waiting_preserved and peer.runtime.failure == 'operator_cancel' and result['closed'])
    elif scenario == 'home_stop':
        passed = core.stop_latched and core.phase == 'STOP_LATCHED' and core.home_acknowledged
    elif scenario in ('link_loss','fc_reboot','landing_timeout'):
        passed = core.phase == 'LAND_TIMEOUT' and core.fault is not None and not result['closed']
    elif scenario == 'restore_echo_lost':
        passed = peer.runtime.coordinator.fusion.phase == 'BLOCKED' and not result['closed']
    elif scenario in ('offboard_rejected','arm_rejected','enable_ack_lost'):
        passed = core.phase in ('BLOCKED','COMPLETE') and core.fault is not None and result['closed']
    else:
        passed = core.fault is not None and result['closed'] and 21 in commands
    pose_times = [t for t,msg in peer.calls if msg.get_type() == 'VISION_POSITION_ESTIMATE']
    return dict(scenario=scenario,passed=bool(passed),prepared=prepared,
        synthetic_elapsed_s=peer.now,phases=phases,core_fault=core.fault,
        commands=commands,home_press_count=2 if scenario == 'home_stop' else 1,
        baseline_restored=peer.params == synthetic_baseline() and result['closed'],
        restoration_unconfirmed=bool(peer.runtime.coordinator.fusion.changed and not result['closed']),
        maximum_pose_send_gap_s=max((b-a for a,b in zip(pose_times,pose_times[1:])),default=None),
        injection_at=peer.injection_at,final=result,
        actual_sent_to_fc=0,devices_opened=0,flight_authorized=False,
        scope='Synthetic peer + real wire encoding/parsers; NOT PX4 SITL, hardware or flight proof')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--simulate',action='store_true',help='Run synthetic in-memory peer only')
    ap.add_argument('--scenario',choices=(*SCENARIOS,'all'),default='all')
    ap.add_argument('--height',type=float,default=.3)
    ap.add_argument('--hover-seconds',type=float,default=3.)
    ap.add_argument('--output-dir',type=Path,help='New exclusive directory; never overwrite old evidence')
    args = ap.parse_args(argv)
    if not args.simulate:
        print(json.dumps(dict(mode='offline_plan',hardware_execution_available=False,
            flight_authorized=False,devices_opened=0,
            flow=['one_HOME_locked_preparation','continuous_same_writer_handoff',
                  'wait_explicit_start','synthetic_enable_ACK_and_observed_state',
                  'Offboard_ACK_and_observed_mode','arm_ACK_and_observed_arm',
                  'ascend','hover','LAND','observed_landed_disarmed',
                  'ground_lock','restore_and_echo'],
            next='Use --simulate for the no-hardware regression; no --execute/serial option exists'),
            ensure_ascii=False,indent=2))
        return 0
    if args.output_dir:
        args.output_dir.mkdir(parents=True,exist_ok=False)
    cases = [simulate(s,height=args.height,hover_seconds=args.hover_seconds)
             for s in SCENARIOS if args.scenario in (s,'all')]
    report = dict(passed=all(c['passed'] for c in cases),cases=cases,
        devices_opened=0,actual_sent_to_fc=0,flight_authorized=False,
        scope='Offline-only software integration. Physical authorizer deliberately absent.')
    if args.output_dir:
        with (args.output_dir/'report.json').open('x') as f:
            json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False)
            f.write('\n')
    print(json.dumps(dict(passed=report['passed'],cases=[{k:c[k] for k in
        ('scenario','passed','phases','core_fault','baseline_restored')} for c in cases],
        devices_opened=0,actual_sent_to_fc=0,flight_authorized=False),ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
