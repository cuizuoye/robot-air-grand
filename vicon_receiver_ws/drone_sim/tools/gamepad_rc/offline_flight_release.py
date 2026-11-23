#!/usr/bin/env python3
"""One no-hardware release check. Does not approve or execute a real flight."""
import argparse
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

TOOLS=Path(__file__).resolve().parent
ROOT=TOOLS.parents[1]
FILES=('continuous_mission_runtime.py','flight_mission_core.py','continuous_execution_io.py',
    'flight_execution_journal.py','flight_execution_review.py','flight_start_confirmation.py',
    'continuous_device_io.py','continuous_device_host.py','offline_flight_release.py',
    'mission_preparation_runtime.py','mission_preparation_coordinator.py','mission_fusion_service.py',
    'mission_live_inputs.py','mission_audit_collector.py','preparation_authorization.py',
    'preparation_host_bindings.py','mavlink_crc_stream.py','compact_vision.py','synthetic_mission_peer.py',
    'continuous_flight.py')
FILES=tuple(dict.fromkeys(FILES+('mission_wire_transport.py','gamepad_estop_monitor.py',
    'rigid_body_revision.py','trs_runtime_monitor.py','usb_combined_fusion_policy.py',
    'usb_yaw_fusion_policy.py','usb_height_fusion_policy.py','usb_fusion_policy.py',
    'control_readonly_policy.py','diagnostic_transaction.py','persistent_shell.py',
    'estop_only_policy.py','ground_estimator_settlement.py','mavlink_readonly_probe.py',
    'trs_uart_readonly_check.py','trs_transport_config.py','trs_vision_reference_policy.py',
    'mission_telemetry_rates.py',
    '../../ros2_ws/src/uav_coord_bridge/uav_coord_bridge/frames.py',
    '../../ros2_ws/src/uav_coord_bridge/uav_coord_bridge/mavlink_vision.py',
    '../../ros2_ws/src/uav_coord_bridge/uav_coord_bridge/mission_preview.py')))
TESTS=('test_continuous_execution.py','test_continuous_device_adapter.py',
    'test_flight_mission_core.py','test_mission_wire_transport.py','test_preparation_host.py',
    'test_mission_preparation_runtime.py','test_mission_preparation_io.py',
    'test_offline_flight_release.py')


def check():
    hashes={};missing=[]
    for name in FILES:
        path=TOOLS/name
        try:
            raw=path.read_bytes();ast.parse(raw,filename=str(path))
            hashes[name]=hashlib.sha256(raw).hexdigest()
        except (OSError,SyntaxError) as exc:missing.append(name+': '+str(exc))
    dependencies={name:importlib.util.find_spec(name) is not None for name in ('numpy','serial','pymavlink','pytest')}
    return dict(scope='Offline software release inspection, not hardware readiness or flight approval',
        source_check_passed=not missing,source_errors=missing,source_sha256=hashes,
        offline_dependencies=dependencies,offline_dependencies_available=all(dependencies.values()),
        features=dict(continuous_takeoff_hover_land=True,real_receive_query_bindings=True,
            guarded_full_writer_api=True,one_use_explicit_start=True,durable_effect_journal=True,
            reviewed_per_message_grant_contract=True,default_deny_hardware_controls=True),
        remaining_physical_conditions=[
            '当前独立电池/TRS/手柄/Vicon的实际联合验收（不是重做标定）',
            '真实回传频率、新鲜度、机载断流动作及独立保护的现场确认',
            '根据现场证据明确审核一次性执行许可；不存在默认批准文件',
            '另行确认首次低空起飞、悬停、降落并验证实际状态'],
        no_device_api_called=True,actual_sent_to_fc=0,physical_authorizer_installed=False,
        native_cli_flight_enabled=False,flight_authorized=False)


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--self-test',action='store_true',help='Run fixed offline/fake-resource tests only')
    ap.add_argument('--json',action='store_true',help='Print full source manifest instead of concise console summary')
    ap.add_argument('--output-dir',type=Path,help='New directory; no evidence overwrites')
    ap.add_argument('--inspect-recovery',type=Path,help='Read a flight journal; never clear/restore it')
    args=ap.parse_args(argv)
    report=check()
    if args.inspect_recovery:
        from flight_execution_journal import FlightExecutionJournal
        try:report['recovery']=FlightExecutionJournal.inspect(args.inspect_recovery)
        except (OSError,ValueError,KeyError) as exc:ap.error(str(exc))
    if args.self_test and args.output_dir is None:
        ap.error('--self-test requires a new --output-dir for retained test evidence')
    if args.output_dir:args.output_dir.mkdir(parents=True,exist_ok=False)
    result=0 if report['source_check_passed'] and report['offline_dependencies_available'] else 2
    if args.self_test and result==0:
        xml=args.output_dir.resolve()/'self_test.xml'
        command=[sys.executable,'-m','pytest','-q',*(str(TOOLS/'test'/name) for name in TESTS),'--junitxml='+str(xml)]
        completed=subprocess.run(command,cwd=ROOT,check=False)
        result=completed.returncode
        root=ET.parse(xml).getroot()
        suites=list(root.iter('testsuite'))
        report['self_test']=dict(exit_code=result,tests=sum(int(s.get('tests',0)) for s in suites),
            failures=sum(int(s.get('failures',0))+int(s.get('errors',0)) for s in suites),xml=str(xml))
    if args.output_dir:
        with (args.output_dir/'report.json').open('x') as f:
            json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    if args.json:print(json.dumps(report,ensure_ascii=False,indent=2))
    else:
        print('离线软件检查：'+('通过' if result==0 else '未通过')+'；未打开硬件、未安装起飞许可。')
        for item in report['source_errors']:print('代码问题：'+item)
        for name,available in report['offline_dependencies'].items():
            if not available:print('缺少软件依赖：'+name+'（本命令不会自动安装）')
        if report.get('self_test'):print('离线测试：'+str(report['self_test']['tests'])+'项，失败'+str(report['self_test']['failures'])+'项。')
        for index,item in enumerate(report['remaining_physical_conditions'],1):print(str(index)+'. '+item)
        if args.output_dir:print('记录：'+str(args.output_dir/'report.json'))
    return result


if __name__=='__main__':raise SystemExit(main())
