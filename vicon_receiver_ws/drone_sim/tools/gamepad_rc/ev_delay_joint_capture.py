#!/usr/bin/env python3
"""One USB + selected TRS/WiFi + Vicon maintenance capture; default LOCAL only.

No fresh physical confirmation -> no port/gamepad/ROS/SDK creation.
No parameter writes, arming, unlock, mode changes or automatic retry.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import time
import uuid

from static_ulog_capture import BASE, BASE_SHA, USB
from prepare_drone import configuration, TRS, RUNS, require_port_free
from rigid_body_revision import digest, REGISTRY, measured_capture_inputs
from drone_preflight import require_binary_installation
from preparation_authorization import RateRecoveryJournal, require_no_pending_recovery
from ev_delay_capture import DelayRecorder, pose_route
from mavlink_link import FixedPeerUDP
from ev_delay_maintenance import MaintenanceCapture, MaintenanceRecordingSession
from ev_delay_maintenance_host import MaintenanceHost
from ev_delay_device_io import Endpoint, LoggerIO
from ev_delay_live_owner import Owner
from frozen_target_stream import clock_bounds_us

SCOPE = 'one-usb-only-motor-power-isolated-joint-vicon-trs-native-ulog-15s-v2'
WIFI_SCOPE = 'one-usb-only-motor-power-isolated-joint-vicon-wifi-native-ulog-15s-v1'


def transport_metadata(args):
    route = getattr(args, 'transport', 'trs')
    metadata = dict(pose_route=route)
    if route == 'trs' and (getattr(args,'wifi_peer',None) or getattr(args,'wifi_bind',None)):
        raise ValueError('WiFi endpoints require explicit --transport wifi')
    if route == 'wifi' and getattr(args,'trs_port',str(TRS)) != str(TRS):
        raise ValueError('Do not mix --trs-port with WiFi capture')
    if route == 'wifi':
        metadata['pose_transport'] = dict(kind='fixed_peer_udp',
            peer=[args.wifi_peer, args.wifi_port], bind=[args.wifi_bind, args.wifi_port],
            evidence='report.json routes and wifi_datagrams.jsonl in this capture directory')
    pose_route(metadata)  # Validate literals before opening any resource.
    return metadata


def physical_conditions(args):
    """Record operator statements literally; telemetry silence is not isolation."""
    removed = getattr(args, 'confirm_props_off_battery_disconnected_usb_only', False) is True
    installed = getattr(args, 'confirm_props_installed_battery_disconnected_usb_only', False) is True
    if removed == installed or not args.authorization_id:
        raise PermissionError('Current props-off OR props-installed/battery-disconnected/USB-only confirmation required')
    if getattr(args, 'confirm_motor_power_isolated', False) is not True:
        raise PermissionError('Confirm battery physically disconnected and no other motor/ESC power source')
    if getattr(args, 'confirm_independent_usb_cutoff', False) is not True:
        raise PermissionError('Independent physical USB power cutoff must be reachable')
    return dict(props_removed=removed, props_installed=installed,
        battery_disconnected=True, usb_only=True, motor_power_isolated=True,
        independent_cutoff_ready=True, scope_manually_approved=True)


def local_inputs(measured_layout_evidence=None):
    """Existing proof reuse, not another physical validation run."""
    if digest(BASE) != BASE_SHA: raise RuntimeError('Pinned saved Logger evidence changed')
    baseline = json.loads(BASE.read_text())
    if not baseline['save_completed'] or baseline['save_exit_code'] != 0 or baseline['error']:
        raise RuntimeError('Saved Logger profile evidence incomplete')
    c = (configuration() if measured_layout_evidence is None
         else measured_capture_inputs(measured_layout_evidence))
    require_binary_installation()
    require_no_pending_recovery(RUNS)
    return c, baseline


def path_guard(path, resolved, stat):
    def check():
        current = path.resolve(strict=True)
        observed = current.stat()
        if current != resolved or (observed.st_dev, observed.st_ino, observed.st_rdev) != stat:
            raise RuntimeError('Owned serial device disappeared/reassigned: '+str(path))
    return check


class JointPump:
    """Only reader for each endpoint. Exceptions quiesce poses, not USB cleanup."""
    def __init__(self, source):
        self.source = source
        self.usb = self.trs = self.owner = None
        self.failure = None
        self.cleanup = False
        self.max_turn_ns = 0

    def __call__(self):
        before = time.monotonic_ns()
        # HOME/GO, the motion window and Logger I/O already have their own
        # deadlines. A second whole-run stopwatch must not abort finalization.
        # USB always remains serviceable for our logger-off/export if the
        # radio, gamepad, Vicon or recording callback has failed.
        try:
            cleaning = self.cleanup or (self.owner and self.owner.cleanup_started)
            if self.usb and self.usb.port.is_open and not (cleaning and self.usb.fault):
                self.usb.poll()
                self.usb.heartbeat()
            if self.trs and self.trs.port.is_open: self.trs.poll()
            if (self.source and not self.source.stopped and not self.failure
                    and not self.cleanup and not (self.owner and self.owner.cleanup_started)):
                self.source.poll()
            if self.owner and not self.owner.closed:
                if not self.failure or self.owner.restoring:
                    self.owner.pump()
                elif self.trs and not self.trs.fault:
                    # Continue clock only for raw post-capture sensor selection.
                    self.owner.rates.probe(time.monotonic_ns(), self.owner.send,
                        identity=self.trs.safe(), restoring=False)
                    self.trs.heartbeat()
        except Exception as exc:
            self.failure = self.failure or str(exc)
            if self.owner:
                self.owner.stop_pose_writes()
                if self.owner.session: self.owner.session.abort(self.failure)
            if not self.cleanup and not (self.owner and self.owner.cleanup_started):
                raise
        self.max_turn_ns = max(self.max_turn_ns, time.monotonic_ns()-before)
        # <=1ms voluntary yield; no blocking serial reads or second reader.
        time.sleep(.0005)


def request_startup_versions(endpoints, pump, *, clock=time.monotonic_ns):
    """One query per route, after observed peer startup; never retry a query."""
    for endpoint in endpoints:
        endpoint.heartbeat()
    deadline = clock()+4_000_000_000
    while True:
        pump()
        now = clock()
        if all(e.heartbeat_ns is not None and 0 <= now-e.heartbeat_ns < 2_000_000_000
               and e.v2_fc_received_ns is not None and 0 <= now-e.v2_fc_received_ns < 500_000_000
               and not e.fault for e in endpoints):
            break
        if now >= deadline:
            raise RuntimeError('USB/TRS peer startup timeout before identity query')
    for endpoint in endpoints:
        endpoint.version()


def motion_cue_ready(fd):
    """One explicit terminal line; keep the normal pump live while waiting."""
    import select
    if not select.select([fd], [], [], 0)[0]: return False
    data = os.read(fd, 64)
    if data.strip() != b'GO':
        raise RuntimeError('Motion cue cancelled/invalid; no capture started')
    return True


def execute(args, c, baseline):
    physical = physical_conditions(args)  # Reject before imports/resources/one-use marker.
    transport = transport_metadata(args)
    wifi = transport['pose_route'] == 'wifi'
    scope = WIFI_SCOPE if wifi else SCOPE
    if c.get('measured_layout'):
        if measured_capture_inputs(c['proof']) != c:
            raise ValueError('Measured-layout evidence changed since inspection')
    # Always use a local operator cue. Chat/tool turnaround must not consume
    # the bounded action window; HOME alone never starts it.
    import sys
    if not sys.stdin.isatty(): raise ValueError('Motion cue requires a local terminal; no automatic start')
    motion_ready = lambda: motion_cue_ready(sys.stdin.fileno())
    import serial
    from preparation_host_bindings import LinuxGamepad
    from ev_delay_vicon_source import ViconSource
    from vicon_receiver_provenance import verify_current
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', args.authorization_id):
        raise ValueError('Authorization ID must be a fresh safe name')
    paths = [USB] if wifi else [USB, Path(args.trs_port)]
    resolved = [p.resolve(strict=True) for p in paths]
    if not wifi and resolved[0] == resolved[1]: raise RuntimeError('Two distinct actual serial devices required')
    require_no_pending_recovery(RUNS)
    for path in paths: require_port_free(path)
    provenance = verify_current()  # Reuse manifest; no rebuild or static recapture.
    run = RUNS/('joint_ev_delay_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    auth = RUNS/'joint_ev_delay_authorizations'
    auth.mkdir(exist_ok=True)
    marker = auth/(args.authorization_id+'.json')
    with marker.open('x') as f:
        json.dump(dict(scope=scope, run=str(run), issued_utc=datetime.now(timezone.utc).isoformat()), f)
    run.mkdir(mode=0o700)
    report = dict(scope=scope, transport=transport, authorization_id=args.authorization_id, physical_confirmed=True,
        physical_conditions=physical, physical_evidence_source='operator statement; not inferred from low/unknown battery voltage',
        hardware_capture_completed=False, calibration_approved=False, parameter_writes=0,
        flight_authorized=False, errors=[], cleanup_errors=[], created_ns=time.monotonic_ns(),
        reused_evidence=dict(logger_profile=str(BASE), logger_sha256=BASE_SHA,
            rigid_proof=str(c['proof']), rigid_revision=c['revision'],
            installation_scope=c.get('evidence_scope', 'historical_same_revision_fusion'),
            heading_quality=c.get('heading_quality'),
            measured_layout_evidence_sha256=c.get('evidence_sha256')),
        source_hashes={p.name:digest(p) for p in Path(__file__).parent.glob('ev_delay_*.py')})
    source = owner = logger = host = pad = pump = None
    datagrams = None
    endpoints = []
    boot_session = str(uuid.uuid4())
    try:
        collector_log = run/'collector_ros_logs'
        collector_log.mkdir(mode=0o700)
        os.environ['ROS_LOG_DIR'] = str(collector_log)  # This process only.
        source = ViconSource(run, provenance, latest_only=wifi)
        pump = JointPump(source)
        if not wifi:
            pad = LinuxGamepad()
            if pad.poll()['home_events']: raise RuntimeError('Release HOME before starting; no initial held input accepted')
        for i, (path, real) in enumerate(zip(paths, resolved)):
            st = real.stat()
            port = serial.Serial(str(path), 115200, timeout=0, write_timeout=.02, exclusive=True)
            try:
                endpoint = Endpoint(port, 219 if i == 0 else 213, baseline['identity'],
                    path_check=path_guard(path, real, (st.st_dev, st.st_ino, st.st_rdev)),
                    joint_rx_evidence=run/('usb_rx.jsonl' if i == 0 else 'trs_rx.jsonl'))
            except BaseException:
                port.close()
                raise
            endpoints.append(endpoint)
        if wifi:
            datagrams = (run/'wifi_datagrams.jsonl').open('x')
            def record_datagram(data, address, accepted):
                datagrams.write(json.dumps(dict(at_ns=time.monotonic_ns(), peer=list(address),
                    accepted=accepted, wire_hex=data.hex()))+'\n')
                datagrams.flush()
            binding = transport['pose_transport']
            port = FixedPeerUDP(tuple(binding['peer']), tuple(binding['bind']), on_datagram=record_datagram)
            try:
                endpoint = Endpoint(port, 213, baseline['identity'], path_check=port.check,
                    joint_rx_evidence=run/'wifi_rx.jsonl')
            except BaseException:
                port.close()
                raise
            endpoints.append(endpoint)
        # Existing pump.trs slot owns the selected radio; never opens both radios.
        pump.usb, pump.trs = endpoints
        # Bind the actual writers before any outbound heartbeat/version packet.
        journal = RateRecoveryJournal(run/'rates_recovery.json')
        owner = Owner(pump.trs, pump.usb, source, pad, journal, mount=c['orientation'], offset=c['origin'],
            boot_session=boot_session, created_ns=report['created_ns'], pump=pump, physical=physical,
            motion_ready=motion_ready, transport=transport['pose_route'])
        pump.owner = owner
        logger = LoggerIO(pump.usb, baseline, pump,
            clock_bounds=lambda now: clock_bounds_us(owner.lock.clock, now), boot_session=boot_session)
        owner.logger = logger
        request_startup_versions(endpoints, pump, clock=time.monotonic_ns)
        deadline = time.monotonic_ns()+4_000_000_000
        while not owner.identity():
            pump()
            if time.monotonic_ns() >= deadline: raise RuntimeError('USB/TRS same-aircraft identity timeout')
        # Vicon startup readiness is used directly by this capture, not a second
        # 15s static test. Do not start Logger or consume HOME while source absent.
        deadline = time.monotonic_ns()+8_000_000_000
        while source.latest is None or not source.latest['pose_published']:
            pump()
            if time.monotonic_ns() >= deadline: raise RuntimeError('Vicon FrameSample startup timeout')
        owner.source_pose()
        gate = MaintenanceCapture(home=owner.home, lock=owner.lock, uid=baseline['identity']['uid2'],
            fc_boot=boot_session, gamepad_generation=owner.generation, created_ns=report['created_ns'],
            start_policy='motor_isolated' if wifi else 'home')
        recorder = DelayRecorder(boot_session, dict(fc_boot_session=boot_session,
            physical_conditions=dict(physical),
            rigid_body_revision_sha256=digest(REGISTRY), orientation_transform_evidence=str(c['proof']),
            installation_evidence_scope=c.get('evidence_scope', 'historical_same_revision_fusion'),
            installation_heading_quality=c.get('heading_quality'),
            installation_is_calibration_approval=False,
            sdk_frame_pose_evidence='Atomic FrameSample from pinned isolated receiver; report.json provenance',
            sdk_rate_evidence='Original per-frame sdk_hz preserved; timing bounds require offline review',
            frame_pose_join='atomic_sdk_frame', timestamp_mode='fc_receive_time',
            clock_uncertainty_ns=None, interval_uncertainty_ns=None,
            timing_review_status='pending_actual_ULog_pairing',
            requested_timing_method='native_fc_observation_intervals',
            motion_window_required=True), capacity=40000)
        recorder.metadata.update(transport)
        recorder.metadata.update(vicon_delivery_policy='latest_only' if wifi else 'capture_history',
            requested_pose_period_ns=owner.period_ns,
            sampling_scope='Exact consumed/sent frames retained; latest-only does not claim every camera frame was recorded')
        session = MaintenanceRecordingSession(gate, recorder)
        host = MaintenanceHost(owner, logger, session, state=logger.state, logger_report=logger.report)
        owner.home_enabled = not wifi
        print(('WiFi 联合采集：保持机体静止，无需手柄/HOME；等本终端提示输入 GO，再等“开始动作”才移动。'
            if wifi else '设备已连接。保持机体静止，短按并松开一次HOME（1秒以内）；随后等本终端提示输入 GO，再等“开始动作”才移动。'), flush=True)
        result = host.run_injected(run)
        trace = result.pop('capture_trace', None)
        report['lifecycle'] = result
        if trace is not None:
            with (run/'capture_raw.json').open('x') as f: json.dump(trace, f, ensure_ascii=False, allow_nan=False)
        report['hardware_capture_completed'] = bool(result['injected_cycle_completed'] and not pump.failure)
        report['physical_resource_factory_implemented'] = True
        # Do not inherit the offline coordinator's false hardware flag as a
        # claim about this run. Conversely, I/O completion is never calibration.
        report['needs_timing_review'] = True
    except BaseException as exc:
        report['errors'].append(type(exc).__name__+': '+str(exc))
    finally:
        if pump: pump.cleanup = True
        # MaintenanceHost already handles its logger. Startup failures before
        # run_injected cannot have started Logger and must not stop another log.
        if owner and not owner.closed:
            try: owner.close_owned()
            except BaseException as exc: report['cleanup_errors'].append(str(exc))
        elif owner is None:
            if pad:
                try: pad.close()
                except BaseException as exc: report['cleanup_errors'].append(str(exc))
            if source:
                try: source.close()
                except BaseException as exc: report['cleanup_errors'].append(str(exc))
        for endpoint in endpoints:
            try:
                if endpoint.port.is_open: endpoint.port.close()
            except BaseException as exc: report['cleanup_errors'].append(str(exc))
            try: endpoint.close_rx_evidence()
            except BaseException as exc: report['cleanup_errors'].append('RX evidence close: '+str(exc))
        report['routes'] = [e.report() for e in endpoints]
        if datagrams:
            try: datagrams.close()
            except BaseException as exc: report['cleanup_errors'].append('Datagram evidence close: '+str(exc))
        if owner: report['owner'] = owner.report
        if source: report['source'] = source.report
        if pump: report.update(pump_failure=pump.failure, maximum_pump_turn_ns=pump.max_turn_ns)
        if report['errors'] or report['cleanup_errors']: report['hardware_capture_completed'] = False
        report['ended_ns'] = time.monotonic_ns()
        with (run/'report.json').open('x') as f: json.dump(report, f, ensure_ascii=False, indent=2, allow_nan=False)
        print('报告：'+str(run/'report.json'), flush=True)
        keep = host and (host.report.get('keep_USB_power_required') or not host.logger_report.get('file_closed_verified'))
        if keep and logger.state['on_sent']:
            print('日志关闭尚未确认，请保持USB供电；不要立即重复运行。', flush=True)
        elif report['hardware_capture_completed']:
            print('本次采集已结束。标定值仍需检查实际ULog与动作质量，不会自动写入参数。', flush=True)
        elif not (logger and logger.state.get('on_sent')):
            print('准备阶段已中止，未启动日志或动作采集。不要按HOME或移动；先查看失败原因。', flush=True)
        else:
            print('采集未完整完成；已停止，保留现有原始记录供诊断，不自动重试。', flush=True)
        if owner and not owner.rates.summary()['restoration_verified']:
            print('临时回传频率恢复未确认，已保留 rates_recovery.json；不要重复启动采集，先恢复原值。', flush=True)
    return 0 if report['hardware_capture_completed'] else 2


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', action='store_true', help='One authorized 15s manual-motion capture')
    ap.add_argument('--manual-motion-start', action='store_true', help='Compatibility flag: local GO is now always required')
    ap.add_argument('--authorization-id')
    props = ap.add_mutually_exclusive_group()
    props.add_argument('--confirm-props-off-battery-disconnected-usb-only', action='store_true')
    props.add_argument('--confirm-props-installed-battery-disconnected-usb-only', action='store_true')
    ap.add_argument('--confirm-motor-power-isolated', action='store_true',
        help='Battery physically unplugged; ESC/motors have no other power source')
    ap.add_argument('--confirm-independent-usb-cutoff', action='store_true',
        help='Operator can physically cut USB power without using the computer')
    ap.add_argument('--trs-port', default=str(TRS))
    ap.add_argument('--transport', choices=('trs', 'wifi'), default='trs')
    ap.add_argument('--wifi-peer', help='WiFi-Link literal IPv4; required for WiFi')
    ap.add_argument('--wifi-bind', help='Dedicated adapter literal IPv4; required for WiFi')
    ap.add_argument('--wifi-port', type=int, default=14550)
    ap.add_argument('--measured-layout-evidence', type=Path,
        help='Explicit new-layout evidence for isolated maintenance capture only; no flight/fusion approval')
    args = ap.parse_args(argv)
    try: transport = transport_metadata(args)
    except ValueError as exc: ap.error(str(exc))
    if args.run:
        try: physical_conditions(args)
        except PermissionError as exc: ap.error(str(exc))
    c, baseline = (local_inputs() if args.measured_layout_evidence is None
                   else local_inputs(args.measured_layout_evidence))
    if not args.run:
        print(json.dumps(dict(mode='local_only_no_devices', scope=WIFI_SCOPE if args.transport=='wifi' else SCOPE,
            transport=transport, inputs_ready=True,
            rigid_revision=c['revision'], logger_baseline=str(BASE), USB=str(USB),
            capture_seconds=15, calibration_approved=False,
            installation_scope=c.get('evidence_scope', 'historical_same_revision_fusion'),
            heading_quality=c.get('heading_quality'), flight_authorized=False,
            remaining='Current physical confirmation, actual joint capture, offline timing/quality review'),
            ensure_ascii=False, indent=2))
        return 0
    return execute(args, c, baseline)


if __name__ == '__main__':
    raise SystemExit(main())
