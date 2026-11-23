#!/usr/bin/env python3
"""Native TRS/Vicon/gamepad host; offline plan by default, no flight mode.

--observe is passive zero-TX. --queries separately enables fixed readonly
requests. Neither mode transmits HOME lock, poses, params, rates or controls.
Never run this as the emergency monitor of an armed aircraft.
"""
import argparse
from contextlib import ExitStack
import datetime
import json
import math
import os
from pathlib import Path
import stat
import time

from continuous_device_io import ReceiveShadowRuntime,ContinuousDeviceIO
from prepare_drone import TRS,RUNS,require_port_free
from rigid_body_revision import REGISTRY,inspect_current_calibration
from preparation_host_bindings import LinuxGamepad,RosLatestPose
from readiness_console import LatestStatusPrinter


class SerialOwner:
    """One explicit stable by-id device, no scanning or reconnection."""
    def __init__(self,path,*,baud=115200):
        if baud!=115200:raise ValueError('Current TRS host baud is pinned to 115200; no radio configuration/fallback')
        self.path=Path(path)
        if (self.path.parent!=Path('/dev/serial/by-id')
                or not self.path.name.startswith('usb-Silicon_Labs_CP210')):
            raise ValueError('Explicit CP210x TRS /dev/serial/by-id path required, never a tty index or FC USB')
        require_port_free(self.path)
        self.target=self.path.resolve(strict=True)
        self.identity=os.stat(self.target)
        if not stat.S_ISCHR(self.identity.st_mode):raise ValueError('TRS is not a character device')
        self.port=None;self.closed=False
        import serial
        port=serial.Serial(port=None,baudrate=baud,timeout=0,write_timeout=0,exclusive=True)
        try:
            # Request inactive lines BEFORE opening; this is not a claim that
            # every USB adapter/driver suppresses all hardware transients.
            port.dtr=False;port.rts=False;port.port=str(self.path);port.open()
            self.port=port
            self.check()
        except BaseException:
            port.close();self.closed=True;raise

    def check(self):
        if self.closed or self.port is None:raise OSError('TRS owner closed')
        current=os.stat(self.path)
        opened=os.fstat(self.port.fileno())
        expected=(self.identity.st_ino,self.identity.st_rdev)
        if (self.path.resolve(strict=True)!=self.target
                or (current.st_ino,current.st_rdev)!=expected
                or (opened.st_ino,opened.st_rdev)!=expected):
            raise OSError('TRS unplugged/re-enumerated/replaced; new explicit session required')

    def close(self):
        if not self.closed:
            self.closed=True
            if self.port is not None:self.port.close()


def configuration():
    # Inspect existing calibration hashes, not require or grant a live pose
    # injection permission. No old passing report supplies live readiness.
    registry=json.loads(REGISTRY.read_text())
    heading,board=Path(registry['heading_candidate_path']),Path(registry['board_candidate_path'])
    checked=inspect_current_calibration(heading,board)
    h,b=json.loads(heading.read_text()),json.loads(board.read_text())
    return dict(revision=checked['active_revision_id'],
        origin=b['board_center_in_rigid_m_candidate'],
        orientation=h['body_orientation_in_rigid_xyzw_candidate'])


class ObservationSession:
    def __init__(self,runtime,io,*,seconds=60):
        if not math.isfinite(seconds) or not 0<seconds<=300:raise ValueError('Bounded observation required')
        self.runtime,self.io,self.seconds=runtime,io,seconds
        self.since=None;self.done=False;self.reason=None
        self.missing=[];self.observed=False

    def step(self,now):
        if self.done:return self.status()
        self.io.step(now)
        i=self.runtime.inputs
        hb=bool(i.hb and 0<=now-i.hb['at']<2 and not i.hb['armed'])
        source=bool(i.pose_at is not None and 0<=now-i.pose_at<=.15)
        pad=bool(i.pad_present and i.pad_event_at is not None and 0<=now-i.pad_event_at<=30)
        checks=(('正确飞控身份（纯接收模式不会主动查询版本）',bool(i.identity)),
                ('当前未解锁心跳',hb),('实时Vicon位姿',source),('手柄当前输入（存在设备不等于已唤醒）',pad))
        self.missing=[name for name,ok in checks if not ok]
        if self.io.failure or self.runtime.failure:
            self.done=True;self.reason='latched_input_or_transport_failure'
        elif not self.missing:
            if self.since is None:self.since=now
            if now-self.since>=1:
                self.done=True;self.observed=True;self.reason='basic_inputs_observed_not_flight_ready'
        else:self.since=None
        if not self.done and now>=self.seconds:
            self.done=True;self.reason='observation_timeout_missing_inputs'
        return self.status()

    def status(self):
        return dict(done=self.done,reason=self.reason,basic_inputs_observed=self.observed,
            missing_conditions=list(self.missing),home_stop_sender_active=False,
            flight_authorized=False,io=self.io.status())


def render(status):
    text='观察模式：不发定位、不改参、不解锁；HOME只记录输入，不会急停。'
    if status['io']['failure']:text+=' 故障锁存：'+status['io']['failure']
    elif status['missing_conditions']:text+=' 缺少：'+'、'.join(status['missing_conditions'])
    else:text+=' 基本输入已齐全，等待稳定1秒；这不是起飞许可。'
    return text


def run_live(args,config):
    if (getattr(args,'session_confirmed',False) is not True
            or not args.confirm_stationary or not args.confirm_disarmed):
        raise ValueError('Explicit stationary/disarmed session confirmation required before opening devices')
    run=args.output_dir or RUNS/('device_adapter_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run=Path(run);run.mkdir(parents=True,exist_ok=False)
    runtime=ReceiveShadowRuntime(config['revision'],body_origin_in_rigid=config['origin'],
        body_orientation_in_rigid=config['orientation'])
    io=None;session=None;error=None;status=None;cleanup_errors=[]
    def cleanup(name,fn):
        try:fn()
        except Exception as exc:cleanup_errors.append(name+': '+str(exc))
    try:
        with ExitStack() as stack:
            owner=SerialOwner(args.trs_port);stack.callback(cleanup,'serial',owner.close)
            pad=LinuxGamepad();stack.callback(cleanup,'gamepad',pad.close)
            pose=RosLatestPose();stack.callback(cleanup,'ROS',pose.close)
            printer=LatestStatusPrinter();stack.callback(cleanup,'console',printer.close)
            io=ContinuousDeviceIO(runtime,owner.port,poll_gamepad=pad.poll,poll_pose=pose.poll,
                now_ros_ns=pose.now_ns,check_port_identity=owner.check,queries=args.queries)
            stack.callback(cleanup,'adapter',io.close)
            session=ObservationSession(runtime,io,seconds=args.seconds)
            start=time.monotonic();next_notice=0.;last_notice=None
            while not session.done:
                now=time.monotonic()-start
                status=session.step(now)
                text=render(status)
                if text!=last_notice or now>=next_notice:
                    printer.publish(text);last_notice=text;next_notice=now+5
                time.sleep(.002)
    except (Exception,KeyboardInterrupt) as exc:
        error=type(exc).__name__+': '+str(exc)
    report=dict(scope='Real-device adapter observation only; NOT flight/active emergency protection',
        error=error,session=session.status() if session else None,
        io=io.status() if io else None,flight_authorized=False,
        any_control_parameter_pose_rate_or_home_command_sent=False,
        resident_monitor_running_after_exit=False,no_automatic_reconnect=True,
        resources_closed=not cleanup_errors,cleanup_errors=cleanup_errors,hardware_execution_available=False)
    with (run/'report.json').open('x') as f:
        json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    notice=('退出时有资源清理错误，请查看报告；' if cleanup_errors else '观察已结束，本次资源已关闭；')
    print(notice+'不再监听HOME。报告：'+str(run/'report.json'),flush=True)
    return 0 if error is None and not cleanup_errors and session and session.observed else 2


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--observe',action='store_true',help='Explicitly open real receive-only resources')
    ap.add_argument('--queries',action='store_true',help='Also allow only fixed readonly queries/heartbeat; never controls or HOME')
    ap.add_argument('--confirm-stationary',action='store_true')
    ap.add_argument('--confirm-disarmed',action='store_true')
    ap.add_argument('--trs-port',default=str(TRS))
    ap.add_argument('--seconds',type=float,default=60)
    ap.add_argument('--output-dir',type=Path)
    args=ap.parse_args(argv)
    if not args.observe:
        if args.queries:ap.error('--queries needs explicit --observe and physical confirmations')
        print(json.dumps(dict(mode='offline_adapter_plan',devices_opened=0,
            physical_tx_bytes=0,baud=115200,topic=RosLatestPose.topic,
            default_transport='zero_TX_receive_only',
            optional_queries='explicit readonly only; no params/rates/poses/control/HOME-stop',
            real_flight_execution_available=False,flight_authorized=False,
            home_stop_sender_active=False,warning='This adapter does not replace the active HOME safety monitor'),
            ensure_ascii=False,indent=2));return 0
    if not args.confirm_stationary or not args.confirm_disarmed:
        ap.error('Observation requires --confirm-stationary --confirm-disarmed; no devices opened')
    if not math.isfinite(args.seconds) or not 5<=args.seconds<=300:
        ap.error('--seconds must be finite and within 5..300')
    try:
        config=configuration()
        args.session_confirmed=True
        return run_live(args,config)
    except (ValueError,OSError,RuntimeError,KeyError) as exc:
        print('未启动或未完成：'+str(exc)+'；不自动重连、改网、解锁或起飞。',flush=True);return 2


if __name__=='__main__':raise SystemExit(main())
