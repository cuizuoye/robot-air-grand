#!/usr/bin/env python3
"""Deterministic developer demo. Uses ONLY the synthetic test FC and gamepad.

No real devices, ROS, network, archived flight readiness or live authorization.
The demonstration intentionally lives next to its offline test fixtures.
"""
import argparse
import json
from pathlib import Path

from test_mission_preparation_runtime import Peer
from test_trs_vision_reference import baseline


def run(scenario):
    peer=Peer(fusion=True);runtime=peer.runtime
    phases=[]
    def sample():
        phase=runtime.coordinator.fusion.phase
        if not phases or phases[-1]['phase']!=phase:
            phases.append({'at':peer.now,'phase':phase})
    peer.home_check();sample()
    runtime.request_prepare(operator_confirmed=True)
    for _ in range(400):
        status=peer.turn();sample()
        if status['failure'] or runtime.coordinator.fusion.phase=='HANDED_OFF':break
    prepared=runtime.coordinator.fusion.phase=='HANDED_OFF' and not runtime.failure
    if not prepared:raise RuntimeError('Synthetic preparation failed: '+str(runtime.status()))
    count=runtime.sent_counts['VISION_POSITION_ESTIMATE']
    if scenario=='home':
        peer.turn(home=(True,));peer.turn(home=(False,))
        for _ in range(8):peer.turn()
        fault_check=runtime.stop_latched and runtime.sent_counts['VISION_POSITION_ESTIMATE']==count
    elif scenario=='vicon-loss':
        for _ in range(8):peer.turn(with_pose=False)
        stopped_count=runtime.sent_counts['VISION_POSITION_ESTIMATE']
        for _ in range(8):peer.turn()
        fault_check=bool(runtime.failure) and runtime.sent_counts['VISION_POSITION_ESTIMATE']==stopped_count
    else:
        for _ in range(8):peer.turn()
        fault_check=runtime.sent_counts['VISION_POSITION_ESTIMATE']>count
    runtime.request_close()
    for _ in range(30):
        peer.turn(with_pose=scenario!='vicon-loss');sample()
        if runtime.closed:break
    commands=[int(msg.command) for _,msg in peer.calls if msg.get_type()=='COMMAND_LONG']
    restored=runtime.closed and peer.params==baseline()
    passed=prepared and fault_check and restored and set(commands)<={185,512}
    return dict(synthetic_offline_demo=True,scenario=scenario,passed=passed,
        phases=phases,prepared_before_explicit_start=prepared,scenario_check_passed=fault_check,
        baseline_restored=restored,commands=commands,status=runtime.status(),
        real_devices_opened=0,physical_tx_bytes=0,flight_authorized=False,
        warning='Synthetic MAVLink and gamepad only; NOT SITL, real FC, RF or flight validation')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',choices=('normal','home','vicon-loss'),default='normal')
    parser.add_argument('--output',type=Path,help='New JSON output; never overwrite old evidence')
    args=parser.parse_args()
    result=run(args.scenario)
    text=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)
    if args.output:
        with args.output.open('x') as stream:stream.write(text+'\n')
    print(text)
    return 0 if result['passed'] else 2


if __name__=='__main__':raise SystemExit(main())
