#!/usr/bin/env python3
"""联合操作台：日志显示 + 显式点击启动原联合控制进程。

历史回放与日志跟随共用一个模型。RViz 可选，只在进程专用的本机 ROS
域发布显示 Marker；原始坐标分开，位置对照仅采用本轮控制链路记录的配准。
显示本身无设备连接；运行按钮只通过原入口的控制台命令操作独占后端。
"""
from collections import deque
import argparse
import json
import math
import os
from pathlib import Path
import re
import sys
import time

from mavlink_crc_stream import CrcResyncParser
from mavlink_readonly_probe import PassivePowerEvidence
from px4_modes import decode

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros2_ws/src/uav_coord_bridge'))
RUNS = ROOT / 'preparation_runs'
REPLAYS = ROOT / '空地协同回放'
STREAMS = ('wifi_raw.jsonl', 'traffic.jsonl', 'vicon_frames.jsonl', 'console_events.jsonl', 'cooperative.jsonl', 'car_motion.jsonl')
PHASES = dict(PREPARE='准备定位', WAIT_EXPLICIT_START='等待起飞确认',
    WAIT_OFFBOARD='等待 Offboard', WAIT_ARM='等待解锁', ASCEND='上升', HOVER='悬停',
    LAND='降落', COMPLETE='任务结束', BLOCKED='任务受阻', STOP_LATCHED='停止已锁存',
    LAND_TIMEOUT='降落确认超时', WAIT_READY='等待车机就绪', WAIT_START='等待共同任务启动',
    TAKEOFF='原地起飞 / 小车停车', RENDEZVOUS='移至小车上方 / 小车停车', FOLLOW='小车沿路线行驶 / 无人机跟随',
    WAIT_CAR_STOP='等待小车实际停稳', ALIGN='对准平台和初始相对朝向',
    DESCEND='缓慢下降', HOLD='任务保持 / 小车停车', PILOT_CONTROL='人工接管',
    TO_TAKEOFF='载机驶向固定起飞点', WAIT_TAKEOFF='小车停车，等待无人机离台',
    ROUTE='小车低速执行固定路线', WAIT_LANDING='小车停车，等待无人机落地停桨',
    STOPPING='正在确认停车', ABORTED='小车任务已中止')
LANDING_WAIT = dict(touchdown='PX4 带动力降落，等待飞控判定落地',
    px4_auto_disarm='飞控已判定落地，等待自动锁定停桨',
    platform_position_confirmation='核对平台落点')
FAULTS = {
    'Stale/invalid pose, status, health, emergency availability or mapping':
        '任务因输入状态过期或无效而停止；具体来源查看故障快照',
    'flight_duration_timeout': '任务执行时间已超过上限',
    'normal_descent_tracking_timeout': '缓降未能到达起始高度，转入飞控降落',
    'send_pose_stale': '发送时定位数据已经过期',
    'Preflight Fail': '预飞检查未通过',
    'No manual control input': '未收到遥控输入',
    'No offboard signal': '未收到 Offboard 控制信号',
    'MOTORS_SPUN_NO_LIFTOFF_USER_CONFIRMED': '现场确认：电机转动，未离地',
}


def describe(text):
    text = str(text)
    for raw, zh in FAULTS.items():
        if raw in text: return zh + '\n' + text
    return text


def stamp(row):
    if not isinstance(row, dict):raise ValueError('record must be an object')
    value = row.get('at_ns')
    # native_rc_ground 的旧 Vicon 日志字段名为 at，但值是 callback 纳秒。
    value = (float(value) / 1e9 if value is not None else
             float(row['at']) / (1e9 if 'frame' in row else 1.))
    if not math.isfinite(value) or value < 0: raise ValueError('invalid record time')
    return value


class LogTail:
    """完整行才交付；写到一半的 JSON 不丢弃，不修改来源文件。"""
    def __init__(self, path):
        self.path, self.offset, self.key = Path(path), 0, None
        self.pending = b''
        self.lines = deque()
        self.head = None
        self.errors = 0
        self.changed = False

    def peek(self):
        if self.head is not None: return self.head
        try:
            st = self.path.stat()
            key = (st.st_dev, st.st_ino)
            if self.key is not None and (key != self.key or st.st_size < self.offset):
                # Never silently splice two sessions into one trajectory.
                self.changed = True
                return None
            self.key = key
            if not self.lines:
                with self.path.open('rb') as stream:
                    stream.seek(self.offset)
                    data = stream.read(65536)
                    self.offset += len(data)
                if data:
                    parts = (self.pending + data).split(b'\n')
                    self.pending = parts.pop()
                    self.lines.extend(parts)
                if len(self.pending) > 1024 * 1024:
                    self.pending = b''; self.errors += 1
            while self.lines:
                raw = self.lines.popleft()
                try:
                    row = json.loads(raw)
                    self.head = (stamp(row), row)
                    return self.head
                except (ValueError, KeyError, TypeError): self.errors += 1
        except FileNotFoundError: pass
        return None

    def pop(self):
        row = self.peek()
        self.head = None
        return row

    def unread(self):
        if self.changed:return False
        try:return bool(self.head or self.lines or self.path.stat().st_size > self.offset)
        except FileNotFoundError:return False


class ConsoleModel:
    def __init__(self):
        self.parser = CrcResyncParser()
        self.power = PassivePowerEvidence()
        self.traces = {k: deque(maxlen=6000) for k in ('vicon', 'px4')}
        self.origins = {}
        self.positions, self.times, self.fields = {}, {}, {}
        self.events = deque(maxlen=150)
        self.origin = None
        self.now = 0.
        self.rows = 0
        self.status = {}
        self.report = {}
        self.vicon_ok = None
        self.last_boot = None
        self.last_event = None
        self.last_events = {}
        self.sequence = 0
        self._chunks = {}
        self.cooperative_frame = None
        self.cooperative_synthetic = None
        self.velocity_samples = {}
        self.velocities = {}
        self.speed_traces = {}
        self.headings = {}
        self.cooperative_status = {}
        self.comparison_meta=None
        self.comparison_meta_at=None
        self.comparison_vicon=deque(maxlen=160)
        self.comparison_px4=deque(maxlen=160)
        self.comparison_epoch=None
        self.comparison_boot_regressed=False

    def position_comparison(self,now):
        """Read-only, receipt-time interpolation. Never auto-fit away residuals."""
        def unavailable(reason):return dict(valid=False,reason=reason)
        meta=self.comparison_meta
        if self.cooperative_synthetic:return unavailable('模拟数据，不作为实机位置对照')
        if not meta:return unavailable('本记录无配准信息；等待新版本联合会话')
        if self.comparison_boot_regressed:return unavailable('飞控时间倒退，已停止本轮位置对照')
        if not meta.get('valid') or meta.get('reference')!='uav_body':
            reason=meta.get('reason')
            return unavailable('等待本轮起飞交接冻结配准' if reason=='waiting_for_flight_alignment'
                               else '配准不可用：'+str(reason or '未确认机体参考点'))
        if self.comparison_meta_at is None or not 0<=now-self.comparison_meta_at<=1.:
            return unavailable('配准状态已过期，等待当前联合记录')
        if not self.comparison_vicon or not self.comparison_px4:
            return unavailable('等待飞控与 Vicon 两路位置')
        vicon=list(self.comparison_vicon)
        if not 0<=now-vicon[-1][0]<=1. or not 0<=now-self.comparison_px4[-1][0]<=1.:
            return unavailable('位置已过期，暂停差值显示')
        # Choose the newest FC sample bracketed by actual Vicon body samples.
        # Do not extrapolate or compare the two independently latest positions.
        selected=None
        for at,xyz in reversed(self.comparison_px4):
            if at<vicon[0][0]:break
            if not 0<=now-at<=1.:continue
            for left,right in zip(vicon,vicon[1:]):
                if left[0]<=at<=right[0] and 0<right[0]-left[0]<=.2:
                    selected=(at,xyz,left,right);break
            if selected:break
        if selected is None:return unavailable('两路时间未配对；等待相邻 Vicon 样本（间隔≤200 ms）')
        at,xyz,left,right=selected
        try:
            from uav_coord_bridge.target_mapping import controller_target_to_map
            px4=controller_target_to_map(meta['alignment'],xyz)
        except (ImportError,KeyError,TypeError,ValueError) as exc:
            return unavailable('坐标变换不可用：'+str(exc))
        alpha=(at-left[0])/(right[0]-left[0])
        body=tuple(a+(b-a)*alpha for a,b in zip(left[1],right[1]))
        delta=tuple(a-b for a,b in zip(px4,body))
        return dict(valid=True,px4_map_m=px4,vicon_body_map_m=body,delta_m=delta,
                    horizontal_m=math.hypot(*delta[:2]),total_m=math.sqrt(sum(x*x for x in delta)),
                    age_s=now-at,pair_gap_s=right[0]-left[0],basis='host_receipt_interpolation')

    def measured_motion(self, source, at, xyz):
        """Display-only 0.5s position fit; repeated/stale samples are not speed."""
        xyz = tuple(float(v) for v in xyz)
        if len(xyz)!=3 or not all(math.isfinite(v) for v in xyz):return
        samples=self.velocity_samples.setdefault(source,deque(maxlen=200))
        if samples and at<=samples[-1][0]:return
        if samples and at-samples[-1][0]>.5:
            samples.clear();self.velocities.pop(source,None)
        samples.append((at,*xyz))
        while samples and at-samples[0][0]>.5:samples.popleft()
        self.position(source,at,xyz)
        if len(samples)<3 or at-samples[0][0]<.2:return
        ts=[p[0]-at for p in samples];mean=sum(ts)/len(ts)
        denom=sum((t-mean)**2 for t in ts)
        if denom<=0:return
        velocity=tuple(sum((t-mean)*p[i+1] for t,p in zip(ts,samples))/denom for i in range(3))
        self.velocities[source]=(at,velocity)
        trace=self.speed_traces.setdefault(source,deque(maxlen=6000))
        if not trace or at-trace[-1][0]>=.05:
            trace.append((at,math.hypot(*velocity[:2]),velocity[2]))

    def motion_value(self, source, now):
        item=self.velocities.get(source)
        if item is None or not 0<=now-item[0]<=1.:return None
        v=item[1]
        return dict(horizontal=math.hypot(*v[:2]),vertical=v[2],total=math.sqrt(sum(x*x for x in v)))

    def event(self, at, level, source, text):
        text = describe(text)
        key = (level, source, text)
        if key == self.last_events.get(source): return
        self.last_event = key
        self.last_events[source] = key
        self.events.append(dict(at=at, level=level, source=source, text=text))
        self.sequence += 1

    def position(self, source, at, xyz):
        xyz = tuple(float(v) for v in xyz)
        if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz): return
        trace = self.traces.setdefault(source,deque(maxlen=6000))
        self.origins.setdefault(source, xyz)
        self.positions[source] = xyz
        self.times[source] = at
        # 绘图降采样仅影响显示，原日志和控制输入完整保留。
        if not trace or at - trace[-1][0] >= .05:
            trace.append((at, *xyz))

    def field(self, name, at, value):
        self.fields[name] = value
        self.times[name] = at

    def message(self, msg, at):
        kind = msg.get_type()
        if kind == 'BAD_DATA': return
        if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1): return
        if kind == 'LOCAL_POSITION_NED':
            boot = int(msg.time_boot_ms)
            if self.last_boot is not None and boot < self.last_boot:
                self.traces['px4'].clear()
                self.origins.pop('px4', None)
                self.event(at, 'WARN', '飞控', '飞控时间倒退；已分开本地轨迹，可能重启或乱序')
                self.comparison_px4.clear();self.comparison_boot_regressed=True
            self.last_boot = boot
            # PX4 NED -> 显示 E/N/Up；不与 Vicon 原点或方向进行自动对齐。
            self.position('px4', at, (msg.y, msg.x, -msg.z))
            if all(math.isfinite(v) for v in (msg.x,msg.y,msg.z)) and (not self.comparison_px4 or at>self.comparison_px4[-1][0]):
                self.comparison_px4.append((at,(msg.x,msg.y,msg.z)))
            self.field('speed', at, math.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2))
        elif kind == 'HEARTBEAT':
            self.field('mode', at, decode(msg.custom_mode)[0])
            self.field('armed', at, bool(msg.base_mode & 128))
        elif kind == 'EXTENDED_SYS_STATE':
            self.field('landed', at, {0:'未知', 1:'在地面', 2:'在空中', 3:'起飞中', 4:'降落中'}.get(msg.landed_state, '未知'))
        elif kind in ('BATTERY_STATUS', 'SYS_STATUS'):
            if kind == 'BATTERY_STATUS' and msg.id != 0: return
            self.power.ingest(msg, at)
            row = (self.power.batteries if kind == 'BATTERY_STATUS' else self.power.system_power)[-1]
            self.field(kind, at, dict(percent=row['remaining_percent'],
                voltage=row.get('reported_voltage_v', row.get('voltage_v'))))
            self.power.batteries[:] = self.power.batteries[-1:]
            self.power.system_power[:] = self.power.system_power[-1:]
        elif kind == 'STATUSTEXT':
            text = msg.text.decode(errors='replace') if isinstance(msg.text, bytes) else msg.text
            # MAVLink2 长文本可以分包；只显示完整连续的组，不拼接丢失的片段。
            ident, seq = getattr(msg, 'id', 0), getattr(msg, 'chunk_seq', 0)
            if ident:
                previous = self._chunks.get(ident, (-1, '', at))
                if seq == 0: previous = (-1, '', at)
                if seq != previous[0] + 1 or at - previous[2] > 3:
                    self.event(at, 'WARN', '飞控', '报错文本分包不完整：' + text)
                    self._chunks.pop(ident, None); return
                combined = previous[1] + text.rstrip('\x00')
                if len(text.encode()) >= 50 and '\x00' not in text:
                    self._chunks[ident] = (seq, combined, at)
                    if len(self._chunks) > 64: self._chunks.pop(next(iter(self._chunks)))
                    return
                text = combined; self._chunks.pop(ident, None)
            self.event(at, 'ERROR' if msg.severity <= 3 else 'WARN' if msg.severity == 4 else 'INFO', '飞控', text.rstrip('\x00\t'))
        elif kind == 'COMMAND_ACK' and msg.result not in (0, 5):
            self.event(at, 'ERROR', '飞控', f'指令 {msg.command} 未被接受，ACK result={msg.result}')

    def ingest(self, name, row):
        at = stamp(row)
        if self.origin is None: self.origin = at
        self.now = max(self.now, at)
        self.rows += 1
        if name in ('cooperative.jsonl','car_motion.jsonl'):
            frame=(row['frame_id'],row['frame_revision'])
            if not all(isinstance(x,str) and x for x in frame) or type(row['synthetic']) is not bool:
                raise ValueError('协同记录缺少共同坐标版本或数据来源声明')
            if self.cooperative_frame is not None and (frame!=self.cooperative_frame or row['synthetic']!=self.cooperative_synthetic):
                raise ValueError('协同坐标/刚体版本或模拟来源改变，不能拼接轨迹')
            self.cooperative_frame=frame; self.cooperative_synthetic=row['synthetic']
            if name=='cooperative.jsonl':
                meta=row.get('position_comparison')
                if meta is not None and not isinstance(meta,dict):
                    self.comparison_meta=None
                    raise ValueError('位置对照元数据必须为对象')
                self.comparison_meta=meta;self.comparison_meta_at=at
                if meta:
                    epoch=json.dumps((meta.get('boot_session'),meta.get('alignment')),sort_keys=True)
                    if epoch!=self.comparison_epoch:
                        self.comparison_vicon.clear();self.comparison_px4.clear()
                        self.comparison_epoch=epoch
                    if not meta.get('valid'):self.comparison_vicon.clear();self.comparison_px4.clear()
            for kind in ('car','uav') if name=='cooperative.jsonl' else ('car',):
                received=row.get(kind+'_received_ns')
                sample_at=float(received)/1e9 if received is not None else at
                if not math.isfinite(sample_at) or sample_at<0 or sample_at>at+.05:
                    raise ValueError('位置接收时间无效')
                self.measured_motion('coop_'+kind,sample_at,row[kind+'_position_map_m'])
                if kind=='uav' and self.comparison_meta and self.comparison_meta.get('valid'):
                    xyz=tuple(float(v) for v in row['uav_position_map_m'])
                    if len(xyz)==3 and all(math.isfinite(v) for v in xyz) and (not self.comparison_vicon or sample_at>self.comparison_vicon[-1][0]):
                        self.comparison_vicon.append((sample_at,xyz))
                yaw=row.get(kind+'_yaw_rad')
                if isinstance(yaw,(float,int)) and math.isfinite(yaw):self.headings['coop_'+kind]=yaw
            self.status=dict(row['status'])
            self.cooperative_status=dict(self.status)
            if self.status.get('uav_target_map_m') is not None:
                self.position('coop_target',at,self.status['uav_target_map_m'])
            self.times['host']=at
            if name=='cooperative.jsonl':
                for field,key in (('armed','observed_armed'),('landed','observed_landed'),('car_stopped','observed_car_stopped')):
                    self.field(field,at,row[key])
            else:self.field('car_stopped',at,self.status['car_stopped'])
            phase=self.status['phase']; fault=self.status.get('fault')
            landing=LANDING_WAIT.get(self.status.get('landing_waiting_for'))
            self.event(at,'ERROR' if fault else 'INFO','协同模拟' if row['synthetic'] else '协同记录',
                PHASES.get(phase,phase)+('：'+fault if fault else '：'+landing if landing else ''))
        elif name == 'vicon_frames.jsonl':
            f = row['frame']
            if not isinstance(f, dict):raise ValueError('frame must be an object')
            if (f.get('subject'), f.get('segment'), f.get('frame_id')) != ('MircoAir', 'MircoAir', 'map'): return
            good = bool(f.get('sdk_ok') and f.get('pose_published') and not f.get('occluded') and not f.get('queue_dropped'))
            if self.vicon_ok is not None and good != self.vicon_ok:
                self.event(at, 'INFO' if good else 'WARN', 'Vicon', '刚体可见性恢复' if good else '刚体遮挡或该帧不可用')
            self.vicon_ok = good
            if good: self.position('vicon', at, f['world_pose'][:3])
        elif name in ('wifi_raw.jsonl', 'traffic.jsonl'):
            if row.get('direction', 'rx_datagram') != 'rx_datagram' or row.get('accepted') is not True: return
            for msg in self.parser.parse_buffer(bytes.fromhex(row['wire_hex'])): self.message(msg, at)
        elif name == 'console_events.jsonl':
            text = str(row.get('text', ''))
            try: value = json.loads(text)
            except (TypeError, ValueError): value = None
            if isinstance(value, dict) and 'phase' in value:
                self.status = value
                self.times['host'] = at
                phase = value['phase']
                self.event(at, 'ERROR' if value.get('failure') else 'INFO', '任务',
                    PHASES.get(phase, phase) + ('：' + str(value['failure']) if value.get('failure') else ''))
            elif not isinstance(value, dict):
                self.event(at, 'ERROR' if 'fault' in text.lower() or 'rejected' in text.lower() else 'INFO', '程序', text)
            # 起飞确认口令不进入图形界面；操作仍由原来的控制终端负责。

    def age(self, key, now):
        at = self.times.get(key)
        return None if at is None or now < at else now - at

    def battery(self, now):
        for kind in ('BATTERY_STATUS', 'SYS_STATUS'):
            age = self.age(kind, now)
            row = self.fields.get(kind)
            if row and age is not None and age <= 10 and row['percent'] is not None:
                return dict(row, source=kind, age=age)
        return None


def active_joint_directory(base, *, now=None):
    """Read-only discovery; never launch a receiver/controller to create data."""
    now=time.monotonic() if now is None else now
    candidates=[]
    for path in Path(base).glob('*/session/cooperative.jsonl'):
        try:
            st=path.stat()
            if time.time()-st.st_mtime>5:continue
            with path.open('rb') as f:
                f.seek(max(0,st.st_size-65536));lines=f.read().splitlines()
            # The writer may be halfway through the last record.
            for raw in reversed(lines):
                try:row=json.loads(raw);at=stamp(row)
                except (ValueError,TypeError,KeyError):continue
                if row.get('synthetic') is False and 0<=now-at<=3:
                    candidates.append((at,path.parent))
                break
        except OSError:continue
    return max(candidates,key=lambda p:p[0])[1] if candidates else None


def refresh_replay_library(base=RUNS,library=REPLAYS):
    """Index closed real joint runs only. Link replay data; never move originals."""
    from datetime import datetime
    import hashlib
    base,library=Path(base),Path(library)
    library.mkdir(parents=True,exist_ok=True)
    index_path=library/'回放索引.json'
    try:cache=json.loads(index_path.read_text())
    except (OSError,ValueError):cache={}
    if not isinstance(cache,dict):cache={}
    updated={};entries=[]
    def read(path):
        try:value=json.loads(path.read_text());return value if isinstance(value,dict) else {}
        except (OSError,ValueError):return {}
    for log in sorted(base.glob('*/session/cooperative.jsonl')):
        session=log.parent;run=session.parent;stop=session/'joint_stop.json'
        # This file is emitted when the original joint owner closes, including
        # abnormal cleanup. Active/preparation-only runs are not replay tests.
        if not stop.is_file():continue
        plan_path=run/'plan.json'
        fingerprint=[[p.stat().st_size,p.stat().st_mtime_ns] if p.exists() else None
                     for p in (log,stop,plan_path)]
        key=str(session.resolve())
        cached=cache.get(key,{})
        item=cached.get('entry') if cached.get('fingerprint')==fingerprint else None
        if cached.get('fingerprint')!=fingerprint:
            first=last=None;armed=False;airborne=False;align=False;arrived=False;synthetic=False;distance=0.
            faults=set();car_origin=None;broken=0
            with log.open() as stream:
                for line in stream:
                    try:
                        row=json.loads(line)
                        if not isinstance(row,dict):raise ValueError('not object')
                        if row.get('synthetic') is not False:synthetic=True;continue
                        status=row.get('status') or {}
                        at=float(row['at_ns'])/1e9
                        xyz=tuple(float(v) for v in row['car_position_map_m'])
                        if len(xyz)!=3 or not all(math.isfinite(v) for v in (*xyz,at)):raise ValueError('nonfinite')
                        first=at if first is None else first;last=at
                        if car_origin is None:car_origin=xyz
                        distance=max(distance,math.dist(xyz[:2],car_origin[:2]))
                        armed|=row.get('observed_armed') is True
                        airborne|=row.get('observed_armed') is True and row.get('observed_landed') is False
                        phase=status.get('car_task_phase',status.get('phase'))
                        align|=phase=='TO_TAKEOFF'
                        arrived|=phase=='WAIT_TAKEOFF' and row.get('observed_car_stopped') is True
                        if status.get('fault'):faults.add(str(status['fault']))
                    except (ValueError,TypeError,KeyError):broken+=1
            item=None
            if not synthetic and first is not None and (armed or align and distance>=.05):
                plan=read(plan_path);closure=read(stop)
                flight=armed
                rectangle=len(plan.get('route_local_forward_left_m',[]))==4
                task=('圆周跟随' if plan.get('circle_route') else '矩形跟随' if rectangle else '往返跟随') if airborne else '起飞尝试' if armed else '小车自动就位'
                result=('流程完成' if closure.get('cooperative_complete') is True and closure.get('car_stop_confirmed') is True and not closure.get('errors')
                        else '未完整完成') if flight else ('已就位' if arrived and closure.get('car_stop_confirmed') is True else '就位未完成')
                if closure.get('errors') or not closure.get('car_stop_confirmed'):result+='·停车待确认'
                match=re.search(r'(20\d{6})_(\d{6})(?:_|$)',run.name)
                dt=datetime.strptime(''.join(match.groups()),'%Y%m%d%H%M%S') if match else datetime.fromtimestamp(log.stat().st_mtime)
                date=dt.strftime('%Y-%m-%d_%H-%M-%S')
                name=f'{date}_{task}_{result}_{hashlib.sha256(key.encode()).hexdigest()[:5]}'
                item=dict(source_session=key,source_run=str(run.resolve()),record_time=dt.isoformat(),
                    time_basis='run_directory' if match else 'log_modification_time',task=task,result=result,
                    duration_s=round(last-first,1),airborne_observed=airborne,armed_observed=armed,
                    car_max_displacement_m=round(distance,3),faults=sorted(faults),malformed_rows=broken,
                    directory=str(library/('飞行跟随' if flight else '小车就位')/name))
        updated[key]=dict(fingerprint=fingerprint,entry=item)
        if item is None:continue
        # Reconstruct links even with a cached classification. No large log copy.
        directory=Path(item['directory']);directory.mkdir(parents=True,exist_ok=True)
        sources={name:session/name for name in STREAMS if (session/name).is_file()}
        for name in ('analysis.json','speed_and_landing_analysis.json','alignment_analysis.json','plan.json'):
            path=session/name if (session/name).is_file() else run/name
            if path.is_file():sources[name]=path
        for name,source in sources.items():
            dest=directory/name
            if dest.is_symlink() and dest.resolve()==source.resolve():continue
            if dest.exists() or dest.is_symlink():raise ValueError('回放目录中存在非预期文件，未覆盖：'+str(dest))
            dest.symlink_to(source.resolve())
        (directory/'回放说明.json').write_text(json.dumps(item,ensure_ascii=False,indent=2)+'\n')
        entries.append(item)
    # Only this small index is replaced. Historical source logs are read-only.
    tmp=index_path.with_suffix('.tmp');tmp.write_text(json.dumps(updated,ensure_ascii=False,indent=2)+'\n');tmp.replace(index_path)
    (library/'使用说明.txt').write_text('在操作台点击“打开历史记录”，选择“飞行跟随”或“小车就位”下的一个日期任务文件夹即可回放，无需再找 session。\n'
        '目录只收录已收尾且记录过解锁/飞行或小车就位运动的真实联合会话；连接准备、模拟和离线测试不收录。\n'
        '每个回放目录通过文件链接引用原始记录，避免重复占用磁盘；不要删除 preparation_runs 中的原始数据。\n'
        '“未完整完成”保留当时的实际轨迹，不表示正常完成。记录时间优先取原轮次目录，无时间时取日志修改时间。\n')
    return sorted(entries,key=lambda e:e['record_time'],reverse=True)


class Session:
    def __init__(self, directory, *, follow=False):
        self.directory = Path(directory).resolve()
        if not self.directory.is_dir(): raise ValueError('请选择现有采集/飞行记录目录')
        self.follow = follow
        self.model = ConsoleModel()
        # 两种入口的 RX 日志互斥，不重复解码同一份接收流。
        names = ['traffic.jsonl' if (self.directory/'traffic.jsonl').exists() and not (self.directory/'wifi_raw.jsonl').exists() else 'wifi_raw.jsonl',
                 'vicon_frames.jsonl', 'console_events.jsonl', 'cooperative.jsonl', 'car_motion.jsonl']
        self.tails = [LogTail(self.directory/n) for n in names]
        self.cursor = None
        self.eof = False
        self.summary_loaded = False
        self.bounds = None
        self.seeking = False

    def read_bounds(self):
        """Read timestamps only, on a separate reader; never consume playback."""
        low=high=None
        for tail in self.tails:
            if not tail.path.is_file():continue
            with tail.path.open('rb') as stream:
                for line in stream:
                    if not line.endswith(b'\n'):continue
                    try:at=stamp(json.loads(line))
                    except (ValueError,KeyError,TypeError,OverflowError):continue
                    low=at if low is None else min(low,at)
                    high=at if high is None else max(high,at)
        return (low,high) if low is not None else None

    def seek(self, seconds):
        """Rebuild all display state up to a time, including CRC/parser history.

        pump() performs bounded work so dragging never blocks the GUI for an
        entire log. Forward and backward seeks use the same historical decoder.
        """
        if self.follow:raise ValueError('实时日志不支持历史跳转')
        if self.bounds is None:raise ValueError('时间轴尚未加载')
        seconds=float(seconds)
        if not math.isfinite(seconds):raise ValueError('invalid seek time')
        bounds=self.bounds
        self.__init__(self.directory,follow=False)
        self.bounds=bounds
        self.cursor=bounds[0]+max(0.,min(seconds,bounds[1]-bounds[0]))
        self.seeking=True

    def pump(self, dt=.1, speed=1., *, all_available=False):
        deadline = time.monotonic() + .035
        if self.seeking:dt=0.;all_available=False
        if self.cursor is not None and not self.follow:
            self.cursor += dt * speed
            if self.bounds:self.cursor=min(self.cursor,self.bounds[1])
        count = 0
        while count < 1500 and time.monotonic() < deadline:
            heads = [(t.peek()[0], i) for i, t in enumerate(self.tails) if t.peek() is not None]
            if not heads:
                self.eof = not any(t.unread() for t in self.tails)
                if self.eof:self.seeking=False
                break
            self.eof = False
            at, index = min(heads)
            if self.cursor is None: self.cursor = at
            if not self.follow and not all_available and at > self.cursor:
                self.seeking=False
                break
            _, row = self.tails[index].pop()
            try: self.model.ingest(self.tails[index].path.name, row)
            except (ValueError, KeyError, TypeError, OverflowError) as exc:
                self.model.event(at, 'WARN', '日志', '跳过无法显示的记录：' + str(exc))
            count += 1
        if all_available and self.model.origin is not None: self.cursor = self.model.now
        if self.eof and not self.follow and not self.summary_loaded:
            p = self.directory/'analysis.json'
            if p.is_file():
                try:
                    value = json.loads(p.read_text())
                    if not isinstance(value, dict):raise ValueError('analysis must be an object')
                    self.model.report = value
                    for key in ('result', 'failure', 'user_observation'):
                        if value.get(key): self.model.event(self.model.now, 'INFO' if key != 'failure' else 'ERROR', '历史结论', value[key])
                except (OSError, ValueError): pass
            self.summary_loaded = True


def rviz_environment():
    # 仅影响新开的显示进程，不修改用户环境、Vicon 接收器或小车 ROS 域。
    env = dict(os.environ, ROS_DOMAIN_ID='186', ROS_LOCALHOST_ONLY='0',
        ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='',
        RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    for name in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                 'ROS_DISCOVERY_SERVER', 'CYCLONEDDS_URI'):
        env.pop(name, None)
    return env


def display_vehicle_parts(car=False):
    """Approximate visuals only, +X forward. Never collision/control geometry.

    Car dimensions reuse mecanum_description/urdf/mecanum_car.urdf.xacro
    placeholder body (24x20x6 cm), wheelbase 18 cm and 60 mm wheels.
    The quad is a generic 32 cm display model, not a measured MVD35 CAD model.
    """
    parts=[]
    dark=(.24,.30,.36);accent=(.15,.65,.48) if car else (1.,.48,.10)
    def add(shape,xyz,size,color=dark,rpy=(0.,0.,0.)):
        parts.append(dict(shape=shape,xyz=xyz,size=size,color=color,rpy=rpy))
    if car:
        add('CUBE',(0.,0.,0.),(.24,.20,.06),accent)
        add('CUBE',(0.,0.,.038),(.23,.23,.016),(.75,.81,.86))
        add('CUBE',(.11,0.,.052),(.025,.10,.012),accent)
        for x in (-.09,.09):
            for y in (-.10,.10):
                add('CYLINDER',(x,y,-.03),(.06,.06,.0306),rpy=(math.pi/2,0.,0.))
                # Readable hubs, fixed appearance (no invented wheel telemetry).
                add('CYLINDER',(x,y+math.copysign(.017,y),-.03),(.027,.027,.004),
                    (.55,.62,.68),rpy=(math.pi/2,0.,0.))
    else:
        add('CUBE',(0.,0.,.022),(.13,.085,.045))
        add('CUBE',(.064,0.,.048),(.035,.065,.014),accent)
        for x in (-.105,.105):
            for y in (-.105,.105):
                a=math.atan2(y,x)
                add('CUBE',(x/2,y/2,.012),(math.hypot(x,y),.022,.018),rpy=(0.,0.,a))
                add('CYLINDER',(x,y,.025),(.032,.032,.034),accent)
                add('CYLINDER',(x,y,.046),(.11,.11,.004),(.38,.45,.52))
                add('CUBE',(x,y,.050),(.10,.012,.005),accent,rpy=(0.,0.,a))
        for y in (-.043,.043):add('CUBE',(0.,y,-.012),(.12,.012,.018))
    return parts


def display_part_pose(position,yaw,part):
    """Apply measured heading to a visual part; positions stay in log frame."""
    x,y,z=part['xyz'];c,s=math.cos(yaw),math.sin(yaw)
    r,p,h=part['rpy'];h+=yaw
    cr,sr=math.cos(r/2),math.sin(r/2);cp,sp=math.cos(p/2),math.sin(p/2)
    ch,sh=math.cos(h/2),math.sin(h/2)
    return ((position[0]+c*x-s*y,position[1]+s*x+c*y,position[2]+z),
        (sr*cp*ch-cr*sp*sh,cr*sp*ch+sr*cp*sh,cr*cp*sh-sr*sp*ch,cr*cp*ch+sr*sp*sh))


class RvizView:
    """Vehicle models and measured traces, isolated from all control topics."""
    def __init__(self,model=None,source='coop_uav'):
        import subprocess
        import tempfile
        import yaml
        import rclpy
        from visualization_msgs.msg import MarkerArray
        self.rclpy = rclpy
        self.context = rclpy.Context()
        # Initialize only this display context with local discovery. Restore
        # the parent environment before any future mission subprocess starts.
        previous=dict(os.environ);env=rviz_environment()
        try:
            os.environ.clear();os.environ.update(env)
            rclpy.init(args=[], context=self.context,domain_id=186)
        finally:
            os.environ.clear();os.environ.update(previous)
        self.node = rclpy.create_node('drone_log_view', context=self.context)
        self.pub = self.node.create_publisher(MarkerArray, '/drone_console/markers', 1)
        self.temp=tempfile.TemporaryDirectory(prefix='air_ground_rviz_')
        config=yaml.safe_load(Path(__file__).with_suffix('.rviz').read_text())
        # Fit available trajectories once; leave subsequent camera motion manual.
        keys=('coop_uav','coop_car') if source=='coop_uav' else (source,)
        positions=[p[1:4] for k in keys for p in (model.traces.get(k,[]) if model else [])]
        if positions:
            view=config['Visualization Manager']['Views']['Current']
            low=[min(p[i] for p in positions) for i in range(3)]
            high=[max(p[i] for p in positions) for i in range(3)]
            center=[(low[i]+high[i])/2 for i in range(3)]
            view['Focal Point']=dict(zip(('X','Y','Z'),center))
            view['Distance']=max(2.2,1.3*max(high[i]-low[i] for i in range(3))+.6)
        self.label_size=min(.16,max(.045,.024*config['Visualization Manager']['Views']['Current']['Distance']))
        path=Path(self.temp.name)/'scene.rviz';path.write_text(yaml.safe_dump(config,allow_unicode=True))
        self.proc = subprocess.Popen(['/opt/ros/jazzy/bin/rviz2', '-d', str(path)],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def draw(self, model, source, history):
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker, MarkerArray
        result = MarkerArray()
        wipe = Marker(); wipe.action = Marker.DELETEALL
        result.markers.append(wipe)
        sources=('coop_uav','coop_car') if source=='coop_uav' else (source,)
        for key in sources:
            points = model.traces.get(key,[])
            if not points:continue
            anchor=Marker();anchor.header.frame_id='drone_display'
            anchor.header.stamp=self.node.get_clock().now().to_msg()
            position=model.positions.get(key,tuple(points[-1][1:4]))
            yaw=model.headings.get(key,0.)
            color=(.12,.85,.58) if key=='coop_car' else (1.,.47,.06)
            # Do not draw a connecting segment across a missing-data interval.
            groups=[];chunk=[]
            for p in points:
                if chunk and p[0]-chunk[-1][0]>1.:
                    groups.append(chunk);chunk=[]
                chunk.append(p)
            if chunk:groups.append(chunk)
            for i,group in enumerate(groups):
                if len(group)<2:continue
                trace=Marker();trace.header=anchor.header;trace.ns=key+'_trace';trace.id=i
                trace.type=Marker.LINE_STRIP;trace.action=Marker.ADD;trace.pose.orientation.w=1.
                trace.scale.x=.018 if key=='coop_car' else .024
                trace.color.r,trace.color.g,trace.color.b=color;trace.color.a=1.
                trace.points=[Point(x=p[1],y=p[2],z=p[3]) for p in group]
                result.markers.append(trace)
            start=Marker();start.header=anchor.header;start.ns=key+'_start';start.id=0
            start.type=Marker.SPHERE;start.action=Marker.ADD;start.pose.orientation.w=1.
            start.pose.position.x,start.pose.position.y,start.pose.position.z=model.origins.get(key,points[0][1:4])
            start.scale.x=start.scale.y=start.scale.z=.075
            start.color.r,start.color.g,start.color.b=color;start.color.a=1.
            result.markers.append(start)
            for i,part in enumerate(display_vehicle_parts(key=='coop_car')):
                body=Marker();body.header=anchor.header;body.ns=key+'_body';body.id=i
                body.type=getattr(Marker,part['shape']);body.action=Marker.ADD
                xyz,q=display_part_pose(position,yaw,part)
                body.pose.position.x,body.pose.position.y,body.pose.position.z=xyz
                body.pose.orientation.x,body.pose.orientation.y,body.pose.orientation.z,body.pose.orientation.w=q
                body.scale.x,body.scale.y,body.scale.z=part['size']
                body.color.r,body.color.g,body.color.b=part['color'];body.color.a=1.
                result.markers.append(body)
            label = Marker(); label.header = anchor.header; label.ns = key+'_label'; label.id = 2
            label.type = Marker.TEXT_VIEW_FACING; label.action = Marker.ADD
            label.pose.orientation.w = 1.; label.pose.position.x = points[-1][1]
            label.pose.position.y = points[-1][2]; label.pose.position.z = points[-1][3] + .30
            label.scale.z = self.label_size; label.color.a = 1.
            label.color.r,label.color.g,label.color.b=color
            names={'vicon':'VICON','px4':'PX4',
                   'coop_uav':'UAV','coop_car':'CAR'}
            prefix='DEMO-' if key.startswith('coop_') and model.cooperative_synthetic else 'REPLAY-' if history else ''
            reference=model.now if history else time.monotonic()
            motion=model.motion_value(key,reference)
            speed=f"{motion['horizontal']:.2f}m/s" if motion else '--m/s'
            age=model.age(key,reference)
            state='STALE-' if age is None or age>1. else ''
            label.text = prefix + state + names[key]+'\n'+speed
            if key=='coop_uav' and key in model.origins:
                label.text+=f"\ndZ:{position[2]-model.origins[key][2]:+.2f}m"
            result.markers.append(label)
        self.pub.publish(result)

    def close(self):
        if self.proc.poll() is None:
            self.proc.terminate()
        self.node.destroy_node(); self.context.shutdown()
        self.temp.cleanup()


class Console:
    def __init__(self, root, session=None, *, auto_live=False):
        import tkinter as tk
        from tkinter import ttk
        from tkinter import font
        self.tk, self.root, self.session = tk, root, session
        # 高分屏使用明确的大号像素字体，同时扩大侧栏和曲线区域；只影响此窗口。
        root.tk.call('tk', 'scaling', 4/3)
        font.nametofont('TkDefaultFont').configure(size=-21)
        self.rviz = None
        self.playing = True
        self.last_tick = time.monotonic()
        self.event_version = -1
        self.auto_live=auto_live
        self.last_discovery=0.
        from joint_console_run import JointConsoleRun,PLANS
        self.runner=JointConsoleRun();self.close_pending=False;self.preview_dialog=False
        root.title('空地协同操作台 · 实时轨迹与速度')
        root.geometry(f'{min(2460,root.winfo_screenwidth()-100)}x{min(1470,root.winfo_screenheight()-200)}+40+40'); root.minsize(1500, 950); root.configure(bg='#edf2f7')
        style = ttk.Style(root);style.theme_use('clam')
        style.configure('TButton',font=('Noto Sans CJK SC',-22),padding=(15,10),
            background='#f5f8fb',foreground='#29485e',bordercolor='#b4c6d6',borderwidth=1)
        style.map('TButton',background=[('disabled','#e8edf2'),('active','#e1edf7')],
            foreground=[('disabled','#72869b')])
        style.configure('Replay.TButton',font=('Noto Sans CJK SC',-24,'bold'),
            background='#49d5ff',foreground='#081625',bordercolor='#49d5ff')
        style.map('Replay.TButton',background=[('disabled','#d6e3ec'),('active','#8be5ff')],
            foreground=[('disabled','#8298aa'),('!disabled','#081625')])
        style.configure('Stop.TButton',foreground='#a43b3b',bordercolor='#91565a')
        style.configure('TCombobox',font=('Noto Sans CJK SC',-22),padding=7,
            fieldbackground='#ffffff',background='#e2edf5',foreground='#29485e',arrowcolor='#49d5ff')
        style.map('TCombobox',fieldbackground=[('readonly','#ffffff')],foreground=[('readonly','#29485e')])
        root.option_add('*TCombobox*Listbox.background','#ffffff')
        root.option_add('*TCombobox*Listbox.foreground','#29485e')
        style.configure('Replay.Horizontal.TScale',background='#49d5ff',troughcolor='#314a63',
            bordercolor='#314a63',lightcolor='#8be5ff',darkcolor='#49d5ff',sliderlength=32,arrowsize=28,gripcount=0)
        style.map('Replay.Horizontal.TScale',background=[('disabled','#52677b'),('active','#8be5ff')])
        top = tk.Frame(root, bg='#15334b'); top.pack(fill='x')
        tk.Label(top, text='空地协同操作台  /  Vicon', font=('Noto Sans CJK SC', -38, 'bold'),
                 fg='white', bg='#15334b').pack(anchor='w', padx=18, pady=(10, 2))
        self.banner = tk.StringVar()
        tk.Label(top, textvariable=self.banner, fg='#bfe8f5', bg='#15334b',
                 font=('Noto Sans CJK SC', -22)).pack(anchor='w', padx=18, pady=(0, 10))
        tk.Frame(root,bg='#49d5ff',height=3).pack(fill='x')
        bar = tk.Frame(root, bg='#edf2f7'); bar.pack(fill='x', padx=12, pady=8)
        for text, fn in (('自动接入实时任务', self.live), ('打开历史记录', lambda:self.choose(False)), ('跟随运行日志', lambda:self.choose(True)),
                         ('打开 RViz 三维', self.open_rviz)):
            ttk.Button(bar, text=text, command=fn).pack(side='left', padx=3)
        self.source = tk.StringVar(value='小车世界坐标' if session and (session.directory/'car_motion.jsonl').exists() else '车机共同坐标' if session and (session.directory/'cooperative.jsonl').exists() else 'Vicon 刚体原点')
        ttk.Combobox(bar, textvariable=self.source, state='readonly', width=20, font=('Noto Sans CJK SC', -22),
                     values=['Vicon 刚体原点', 'PX4 本地位置', '车机共同坐标', '小车世界坐标']).pack(side='right')
        workflow=tk.Frame(root,bg='#dceaf4');workflow.pack(fill='x',padx=16,pady=(0,8))
        actions=tk.Frame(workflow,bg='#dceaf4');actions.pack(fill='x',padx=8,pady=6)
        self.task_choice=tk.StringVar(value='矩形 3×2 m')
        ttk.Combobox(actions,textvariable=self.task_choice,values=list(PLANS),state='readonly',width=18,
            font=('Noto Sans CJK SC',-22)).pack(side='left',padx=4)
        self.run_buttons={}
        for key,label,fn in (('prepare','①准备连接',self.prepare_run),('align','②小车自动就位',self.align_car),
            ('preview','③生成起飞确认',self.prepare_takeoff),('stop','停止并收尾',self.stop_run),
            ('raw','查看原始报文',self.show_run_details)):
            button=ttk.Button(actions,text=label,command=fn,style='Stop.TButton' if key=='stop' else 'TButton');button.pack(side='left',padx=5)
            self.run_buttons[key]=button
        tk.Label(workflow,text='① 自动检查 / 重连两张专用 Tenda，再准备定位与遥测。请保持车机静止、Pocket 油门最低 / SB 关闭，关闭其他控制入口；就位和起飞另行确认。',
            bg='#dceaf4',fg='#163a56',anchor='w',justify='left',font=('Noto Sans CJK SC',-20),
            wraplength=2300).pack(fill='x',padx=12,pady=(0,4))
        self.run_guide=tk.StringVar()
        tk.Label(workflow,textvariable=self.run_guide,bg='#dceaf4',fg='#163a56',anchor='w',justify='left',
            font=('Noto Sans CJK SC',-22),wraplength=2300).pack(fill='x',padx=12,pady=(0,8))
        body = tk.Frame(root, bg='#edf2f7'); body.pack(fill='both', expand=True, padx=16)
        plots = tk.Frame(body, bg='#edf2f7'); plots.pack(side='left', fill='both', expand=True)
        self.map = tk.Canvas(plots, bg='#ffffff', highlightthickness=1, highlightbackground='#e2eaf0', height=280)
        self.map.pack(fill='both', expand=True)
        self.height = tk.Canvas(plots, bg='#ffffff', highlightthickness=1, highlightbackground='#e2eaf0', height=160)
        self.height.pack(fill='x', pady=(10, 10))
        self.speed_plot=tk.Canvas(plots,bg='#ffffff',highlightthickness=1,highlightbackground='#e2eaf0',height=150)
        self.speed_plot.pack(fill='x',pady=(0,10))
        sidebar = tk.Frame(body, bg='#ffffff', width=580); sidebar.pack(side='right', fill='y', padx=(14, 0))
        sidebar.pack_propagate(False)
        self.speed_labels={}
        for key,label,color in (('coop_uav','无人机','#d78621'),('coop_car','小车','#2b8c6b')):
            v=tk.StringVar(value=label+'  — m/s')
            tk.Label(sidebar,textvariable=v,anchor='w',justify='left',bg='#ffffff',fg=color,
                font=('Noto Sans CJK SC',-34,'bold'),padx=12,pady=6).pack(fill='x')
            self.speed_labels[key]=v
        self.telemetry = tk.StringVar()
        tk.Label(sidebar, textvariable=self.telemetry, anchor='nw', justify='left', wraplength=550,
                 font=('Noto Sans CJK SC', -24), bg='#ffffff', fg='#29485e', padx=12, pady=12).pack(fill='x')
        self.comparison_text=tk.StringVar(value='统一坐标位置对照：等待本轮数据')
        tk.Label(sidebar,textvariable=self.comparison_text,anchor='nw',justify='left',wraplength=550,
                 font=('Noto Sans CJK SC',-21),bg='#e8f3fa',fg='#163a56',padx=12,pady=10).pack(fill='x')
        self.detail = tk.Text(sidebar, height=15, wrap='word', relief='flat', font=('Noto Sans CJK SC', -22),
                              bg='#fff3db', fg='#29485e', insertbackground='#49d5ff', padx=10, pady=8)
        self.detail.pack(fill='both', expand=True, padx=8, pady=8)
        # Playback lives directly above the event log, apart from motion controls.
        self.replay_panel=tk.Frame(root,bg='#e1f2fa',highlightthickness=1,highlightbackground='#397b9b')
        self.replay_panel.pack(fill='x',padx=16,pady=(8,8))
        replay_head=tk.Frame(self.replay_panel,bg='#e1f2fa');replay_head.pack(fill='x',padx=14,pady=(8,2))
        tk.Label(replay_head,text='历史回放  /  TIMELINE',bg='#e1f2fa',fg='#08769c',
            font=('Noto Sans CJK SC',-23,'bold')).pack(side='left')
        self.timeline_text=tk.StringVar(value='打开一轮历史记录，即可拖动回放')
        tk.Label(replay_head,textvariable=self.timeline_text,bg='#e1f2fa',fg='#29485e',
            font=('Noto Sans CJK SC',-22)).pack(side='right')
        replay_controls=tk.Frame(self.replay_panel,bg='#e1f2fa');replay_controls.pack(fill='x',padx=14,pady=(4,10))
        self.pause_button=ttk.Button(replay_controls,text='暂停回放',command=self.pause,style='Replay.TButton',width=11)
        self.pause_button.pack(side='left',padx=(0,12))
        self.timeline=ttk.Scale(replay_controls,from_=0,to=1,orient='horizontal',
            command=self.preview_seek,style='Replay.Horizontal.TScale')
        self.timeline.pack(side='left',fill='x',expand=True,padx=(0,12))
        self.timeline.state(['disabled'])
        self.timeline.bind('<ButtonPress-1>',self.begin_seek)
        self.timeline.bind('<ButtonRelease-1>',self.commit_seek)
        self.timeline.bind('<KeyRelease>',self.commit_seek)
        self.replay_buttons=[]
        for label,fn in (('从头播放',self.restart),('显示全程',self.finish)):
            button=ttk.Button(replay_controls,text=label,command=fn);button.pack(side='left',padx=4)
            self.replay_buttons.append(button)
        self.timeline_session=None;self.dragging=False
        tk.Label(root, text='故障与事件（保留飞控原文；记录中的告警不等于当前仍存在）',
                 font=('Noto Sans CJK SC', -22), bg='#edf2f7', fg='#526477', anchor='w').pack(fill='x', padx=16)
        self.events = tk.Text(root, height=4, wrap='word', font=('Noto Sans CJK SC', -22), bg='#15283a', fg='#e6eff7', relief='flat')
        self.events.pack(fill='x', padx=16, pady=(4, 8))
        for level, color in (('ERROR','#ff9999'), ('WARN','#ffd580'), ('INFO','#c5e4f7')): self.events.tag_config(level, foreground=color)
        self.path_text = tk.StringVar()
        tk.Label(root, textvariable=self.path_text, bg='#15334b', fg='white', anchor='w',
                 font=('Noto Sans CJK SC', -19), padx=16, pady=7).pack(fill='x')
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.tick()

    def consent(self,title,conditions,summary):
        """Unchecked, explicit per-run physical review; never inferred from logs."""
        from tkinter import ttk
        tk=self.tk;dialog=tk.Toplevel(self.root);dialog.title(title);dialog.transient(self.root)
        dialog.geometry('1280x1050');dialog.grab_set();accepted={}
        tk.Label(dialog,text=summary,justify='left',anchor='w',wraplength=1220,
            font=('Noto Sans CJK SC',-23),padx=20,pady=15).pack(fill='x')
        area=tk.Frame(dialog);area.pack(fill='both',expand=True)
        canvas=tk.Canvas(area,highlightthickness=0);scroll=ttk.Scrollbar(area,orient='vertical',command=canvas.yview)
        canvas.configure(yscrollcommand=scroll.set);scroll.pack(side='right',fill='y');canvas.pack(side='left',fill='both',expand=True)
        inner=tk.Frame(canvas);item=canvas.create_window((0,0),window=inner,anchor='nw')
        canvas.bind('<Configure>',lambda e:canvas.itemconfigure(item,width=e.width))
        inner.bind('<Configure>',lambda e:canvas.configure(scrollregion=canvas.bbox('all')))
        checks={}
        for key,label in conditions.items():
            v=tk.BooleanVar(value=False);checks[key]=v
            tk.Checkbutton(inner,text=label,variable=v,anchor='w',justify='left',wraplength=1150,
                font=('Noto Sans CJK SC',-21),padx=16,pady=8).pack(fill='x')
        footer=tk.Frame(dialog);footer.pack(fill='x',padx=20,pady=16)
        def agree():
            from tkinter import messagebox
            if not all(v.get() for v in checks.values()):
                messagebox.showinfo('尚未确认','请仅在条件实际满足时勾选；不满足可取消本步骤。',parent=dialog);return
            accepted.update({k:True for k in checks});dialog.destroy()
        ttk.Button(footer,text='取消',command=dialog.destroy).pack(side='right',padx=8)
        ttk.Button(footer,text='确认以上条件，继续本步骤',command=agree).pack(side='right',padx=8)
        self.root.wait_window(dialog);return accepted

    def run_error(self,exc):
        from joint_console_run import explain_error
        self.runner.issue=explain_error(str(exc),self.runner.stage())

    def prepare_run(self):
        from joint_console_run import PLANS,PROFILE,review_conditions
        try:
            plan_path=PLANS[self.task_choice.get()]
            profile=json.loads(PROFILE.read_text())
            # The single explicit click replaces the ground checklist only.
            # These are operator acknowledgements, not measured hardware results.
            consent=dict.fromkeys(review_conditions(False,profile),True)
            consent['_confirmation_method']='single_prepare_click'
            directory=self.runner.launch(plan_path,consent,existing=active_joint_directory(RUNS))
            self.auto_live=False;self.session=None;self.event_version=-1;self.source.set('车机共同坐标')
            self.owned_directory=directory
        except Exception as exc:self.run_error(exc)

    def align_car(self):
        from tkinter import messagebox
        if not messagebox.askokcancel('小车即将运动','确认车机静止、无人机未解锁；小车周围及绕行区域无人、无其他障碍。程序按两台设备的外框尺寸绕开无人机，再到其左侧并排停稳。\n\n点击确定后仅小车运动，起飞仍需另行确认。',parent=self.root):return
        try:self.runner.align()
        except Exception as exc:self.run_error(exc)

    def prepare_takeoff(self):
        from joint_console_run import review_conditions,route_description
        if not self.runner.can_preview():return
        p=self.runner.plan
        consent=self.consent('生成本轮起飞预览',review_conditions(True,self.runner.profile),
            f"小车已停车。本轮高度 {p['clearance_m']:.2f} m，车速 {p['car_speed_mps']:.2f} m/s。\n"
            f"路线：{route_description(p)}\n"
            '无人机升高→移至车上方→跟随→返回本轮原始地面起点→缓降。\n'
            '请确认已在 Position 模式，Pocket 油门最低、SB 关闭。此步骤不会起飞，之后还有最终确认。')
        if not consent:return
        try:self.runner.request_preview(consent)
        except Exception as exc:self.run_error(exc)

    def confirm_takeoff(self):
        from tkinter import messagebox
        from joint_console_run import route_description
        preview=self.runner.preview
        if preview is None or self.preview_dialog:return
        self.preview_dialog=True
        try:
            joint=preview.get('cooperative') or {};resolved=joint.get('resolved_start') or {}
            text=(f"本轮起飞高度：{preview.get('height_m')} m\n"
                  f"小车速度：{self.runner.plan['car_speed_mps']} m/s\n"
                  f"地面起飞/返回点：{resolved.get('uav_ground_position_map_m')}\n"
                  f"路线：{route_description(self.runner.plan)}\n\n"
                  '确认后将启动电机并自动完成本轮跟随、返回和降落。\n确认人员已离开活动范围，并保持 Pocket 随时可接管。\n'
                  '预览按原入口限时有效；超时不会自动起飞。')
            if messagebox.askyesno('确认起飞并运行',text,parent=self.root):
                self.runner.takeoff(preview,confirmed=True)
            else:self.runner.preview=None
        except Exception as exc:self.run_error(exc);self.runner.preview=None
        finally:self.preview_dialog=False

    def stop_run(self):
        try:self.runner.stop()
        except Exception as exc:self.run_error(exc)

    def show_run_details(self):
        tk=self.tk;dialog=tk.Toplevel(self.root);dialog.title('本轮阶段与原始报文')
        dialog.geometry('1400x900');text=tk.Text(dialog,wrap='word',font=('Noto Sans CJK SC',-21))
        text.pack(fill='both',expand=True)
        content={'阶段':self.runner.stage(),'说明':self.runner.guidance(),'错误':self.runner.issue,
                 '网卡准备':self.runner.network,'原始状态':self.runner.status,'记录目录':str(self.runner.run or '')}
        text.insert('end',json.dumps(content,ensure_ascii=False,indent=2));text.configure(state='disabled')

    def poll_run(self):
        try:self.runner.poll()
        except Exception as exc:self.run_error(exc)
        if self.runner.running and getattr(self,'owned_directory',None) and self.owned_directory.is_dir():
            if self.session is None or self.session.directory!=self.owned_directory.resolve():
                self.session=Session(self.owned_directory,follow=True);self.source.set('车机共同坐标');self.event_version=-1
        self.run_guide.set(self.runner.stage()+'  |  '+self.runner.guidance())
        enabled={'prepare':not self.runner.running and (self.runner.proc is None or self.runner.cleanup_ok),
                 'align':self.runner.can_align(),'preview':self.runner.can_preview() and not self.runner.preview_requested,
                 'stop':self.runner.running and not self.runner.cancelled,'raw':True}
        for key,value in enabled.items():self.run_buttons[key].configure(state='normal' if value else 'disabled')
        if self.runner.preview and not self.preview_dialog:self.root.after_idle(self.confirm_takeoff)
        if self.close_pending and self.runner.finished and self.runner.cleanup_ok:
            self.close_pending=False;self.close();return False
        return True

    def live(self):
        self.auto_live=True;self.session=None;self.last_discovery=0.
        self.event_version=-1;self.source.set('车机共同坐标');self.playing=True

    def choose(self, follow):
        from tkinter import filedialog, messagebox
        if not follow:
            if getattr(self,'history_loading',False):return
            import queue
            from threading import Thread
            self.history_loading=True;result=queue.Queue()
            def scan():
                try:result.put((refresh_replay_library(),None))
                except Exception as exc:result.put((None,str(exc)))
            def ready():
                try:entries,error=result.get_nowait()
                except queue.Empty:self.root.after(100,ready);return
                self.history_loading=False
                if error:messagebox.showerror('回放目录整理失败',error,parent=self.root);return
                if not entries:messagebox.showinfo('暂无运动记录','尚无已收尾的真实空地协同运动记录；纯连接准备不会加入回放目录。',parent=self.root);return
                self.choose_replay_directory(False,REPLAYS)
            Thread(target=scan,daemon=True).start();self.root.after(100,ready)
            return
        self.choose_replay_directory(True,RUNS)

    def choose_replay_directory(self,follow,directory):
        from tkinter import filedialog, messagebox
        path = filedialog.askdirectory(title='选择日期＋任务名称的回放文件夹' if not follow else '选择正在增长的日志目录', initialdir=str(directory))
        if path:
            if not any((Path(path)/name).is_file() for name in STREAMS):
                messagebox.showinfo('请选择具体的一轮','请进入“飞行跟随”或“小车就位”，再选择一个日期开头的任务文件夹。',parent=self.root);return
            try: self.session = Session(path, follow=follow)
            except ValueError as exc: messagebox.showerror('无法打开', str(exc)); return
            self.playing = True; self.event_version = -1
            self.auto_live=False
            if (self.session.directory/'cooperative.jsonl').exists():self.source.set('车机共同坐标')
            if (self.session.directory/'car_motion.jsonl').exists():self.source.set('小车世界坐标')

    def pause(self):
        if self.session and not self.session.follow:
            if self.session.eof:self.restart()
            else:self.playing = not self.playing
            self.update_timeline()

    @staticmethod
    def clock_text(seconds):
        seconds=max(0,int(seconds))
        return f'{seconds//60:02d}:{seconds%60:02d}'

    def preview_seek(self,value):
        s=self.session
        if self.dragging and s and s.bounds:
            self.timeline_text.set(f'跳转至 {self.clock_text(float(value))} / {self.clock_text(s.bounds[1]-s.bounds[0])} · 松手定位')

    def begin_seek(self,event=None):
        if self.session and not self.session.follow and self.session.bounds:
            self.dragging=True;self.seek_resume=self.playing;self.playing=False

    def commit_seek(self,event=None):
        s=self.session
        if not s or s.follow or not s.bounds:return
        resume=self.seek_resume if self.dragging else self.playing
        self.dragging=False;self.fast_forward=False
        s.seek(self.timeline.get());self.playing=resume;self.event_version=-1
        for plot in (self.map,self.height,self.speed_plot):plot.delete('all')
        self.telemetry.set('正在恢复所选时刻…');self.comparison_text.set('位置对照：等待跳转完成')
        for value in self.speed_labels.values():value.set('— m/s')
        self.text(self.events,'');self.text(self.detail,'正在跳转；所选时刻的轨迹和状态加载完成后一起显示。')
        if self.rviz:self.rviz.draw(s.model,'coop_uav',True)

    def update_timeline(self):
        s=self.session
        historical=bool(s and not s.follow)
        self.pause_button.configure(text=('重新播放' if s and s.eof else '暂停回放' if self.playing else '继续回放') if historical else '暂停回放')
        for button in (self.pause_button,*self.replay_buttons):button.state(['!disabled'] if historical else ['disabled'])
        if s is not self.timeline_session:
            self.timeline_session=s;self.dragging=False
            self.timeline.state(['disabled']);self.timeline.set(0)
            self.timeline_error=None
            if s and not s.follow:
                # Bounds are independent of playback; an old scan cannot update
                # a subsequently selected record or live session.
                import queue
                from threading import Thread
                result=queue.Queue()
                def scan():
                    try:result.put((s.read_bounds(),None))
                    except Exception as exc:result.put((None,str(exc)))
                def ready():
                    if self.session is not s:return
                    try:bounds,error=result.get_nowait()
                    except queue.Empty:self.root.after(100,ready);return
                    s.bounds=bounds
                    self.timeline_error=error or ('没有可回放的时间戳' if bounds is None else None)
                Thread(target=scan,daemon=True).start();self.root.after(100,ready)
        if not s or s.follow:
            self.timeline_text.set('实时显示不支持拖动' if s else '历史回放进度 · 打开记录后可拖动')
            return
        if not s.bounds:
            self.timeline_text.set('时间轴不可用：'+self.timeline_error if self.timeline_error else '正在读取历史时间轴…')
            return
        duration=s.bounds[1]-s.bounds[0]
        self.timeline.configure(to=max(duration,.001));self.timeline.state(['!disabled'])
        if not self.dragging:
            elapsed=max(0.,min((s.cursor if s.cursor is not None else s.bounds[0])-s.bounds[0],duration))
            self.timeline.set(elapsed)
            self.timeline_text.set(f'{self.clock_text(elapsed)} / {self.clock_text(duration)} · '+('正在跳转…' if s.seeking else '拖动可快进 / 回退'))

    def restart(self):
        if self.session:
            self.auto_live=False
            if not self.session.follow and self.session.bounds:self.session.seek(0)
            else:self.session = Session(self.session.directory, follow=False)
            self.fast_forward=False
            self.playing = True; self.event_version = -1

    def finish(self):
        if self.session and not self.session.follow: self.fast_forward = True; self.playing = True

    def open_rviz(self):
        from tkinter import messagebox
        if self.rviz:
            if self.rviz.proc.poll() is None:return
            self.rviz.close();self.rviz=None
        source='coop_car' if self.source.get()=='小车世界坐标' else 'coop_uav' if self.source.get()=='车机共同坐标' else 'vicon' if self.source.get().startswith('Vicon') else 'px4'
        try: self.rviz = RvizView(self.session.model if self.session else None,source)
        except Exception as exc: messagebox.showerror('RViz 未启动', str(exc))

    def text(self, widget, value):
        widget.configure(state='normal'); widget.delete('1.0', 'end'); widget.insert('end', value); widget.configure(state='disabled')

    def plots(self, model, source):
        c, h = self.map, self.height
        c.delete('all'); h.delete('all')
        self.draw_speeds(model)
        w, height = max(c.winfo_width(), 400), max(c.winfo_height(), 200)
        points = list(model.traces.get(source,[]))
        color = '#1288b5' if source == 'vicon' else '#d78621'
        title=(('小车世界坐标 · ' if source=='coop_car' else '车机共同坐标 · ')+('离线模拟，非实机' if model.cooperative_synthetic else '已配准记录')) if source.startswith('coop_') else ('Vicon 世界坐标 X / Y' if source=='vicon' else 'PX4 本地坐标 East / North')
        c.create_text(14, 24, anchor='w', text=title+'  ·  单位 m', fill='#29485e', font=('Noto Sans CJK SC', -24, 'bold'))
        if source=='coop_uav':
            c.create_text(w-15,24,anchor='e',text='橙：无人机   绿：小车   紫：目标',fill='#526477')
        if not points:
            c.create_text(w/2, height/2, text='等待该数据源的有效位置记录', fill='#6a8092'); return
        extent=points+list(model.traces.get('coop_car',[]))+list(model.traces.get('coop_target',[])) if source=='coop_uav' else points
        xmin, xmax = min(p[1] for p in extent), max(p[1] for p in extent)
        ymin, ymax = min(p[2] for p in extent), max(p[2] for p in extent)
        cx, cy = (xmin+xmax)/2, (ymin+ymax)/2
        span = max(xmax-xmin, ymax-ymin, .6)*1.25
        scale = min(w-110, height-90)/span
        ox, oy = w/2, (height+25)/2
        def xy(p): return ox+(p[1]-cx)*scale, oy-(p[2]-cy)*scale
        base = 10**math.floor(math.log10(span/5))
        step = next(base*k for k in (1,2,5,10) if base*k >= span/5)
        for axis, center in ((0,cx), (1,cy)):
            for i in range(math.floor((center-span/2)/step), math.ceil((center+span/2)/step)+1):
                val = i*step
                if axis == 0:
                    x = ox+(val-cx)*scale
                    if not 60<=x<=w-20:continue
                    c.create_line(x,40,x,height-30,fill='#e2eaf0')
                    c.create_text(x,height-15,text=f'{val:.2f}',fill='#6a8092')
                else:
                    y = oy-(val-cy)*scale
                    if not 50<=y<=height-35:continue
                    c.create_line(50,y,w-20,y,fill='#e2eaf0')
                    c.create_text(45,y,text=f'{val:.2f}',anchor='e',fill='#6a8092')
        coords = [v for p in points for v in xy(p)]
        if len(coords) >= 4: c.create_line(*coords, fill=color, width=2)
        if source=='coop_uav':
            for key,fill,label in (('coop_car','#2b8c6b','小车'),('coop_target','#9265b3','无人机目标')):
                other=list(model.traces.get(key,[])); line=[v for p in other for v in xy(p)]
                if len(line)>=4:c.create_line(*line,fill=fill,width=2,dash=(4,3))
                if other:
                    x,y=xy(other[-1]);c.create_rectangle(x-4,y-4,x+4,y+4,fill=fill,outline='white');c.create_text(x+7,y-10,text=label,anchor='w',fill=fill)
        for p, fill, label in ((points[0],'#8497a4','显示段起点'), (points[-1],color,'无人机' if source=='coop_uav' else '最新记录')):
            x,y=xy(p); c.create_oval(x-5,y-5,x+5,y+5,fill=fill,outline='white'); c.create_text(x+9,y+(12 if p is points[0] else -12),text=label,anchor='w',fill=fill)
        for key in ('coop_car','coop_uav') if source=='coop_uav' else ():
            if key in model.headings and model.traces.get(key):
                x,y=xy(model.traces[key][-1]);yaw=model.headings[key]
                c.create_line(x,y,x+24*math.cos(yaw),y-24*math.sin(yaw),arrow='last',width=3,
                              fill='#2b8c6b' if key=='coop_car' else '#d78621')
        hw, hh = max(h.winfo_width(),400), max(h.winfo_height(),100)
        h.create_text(14,24,anchor='w',text='相对首个有效样本的高度变化（非离地测距）',fill='#29485e',font=('Noto Sans CJK SC', -22,'bold'))
        first = model.origins[source]
        z0=first[2]; zmin=min(0.,min(p[3]-z0 for p in points)); zmax=max(.3,max(p[3]-z0 for p in points)); zs=max(.3,zmax-zmin)*1.15
        t0=model.origin or points[0][0]; duration=max(1.,points[-1][0]-t0)
        coords=[]
        for p in points:
            coords += [60+(p[0]-t0)/duration*(hw-85), hh-30-(p[3]-z0-zmin)/zs*(hh-70)]
        if len(coords)>=4:h.create_line(*coords,fill=color,width=2)
        h.create_text(14,52,anchor='w',text=f'{zmax:.2f}m',fill='#6a8092'); h.create_text(14,hh-30,anchor='w',text=f'{zmin:.2f}m',fill='#6a8092')
        h.create_text(60,hh-12,text='0s',fill='#6a8092'); h.create_text(hw-45,hh-12,text=f'{duration:.1f}s',fill='#6a8092')

    def draw_speeds(self,model):
        c=self.speed_plot;c.delete('all');w=max(c.winfo_width(),400);h=max(c.winfo_height(),100)
        c.create_text(14,24,anchor='w',text='水平速度 · Vicon 位置差分（0.5 s 窗口）',fill='#29485e',font=('Noto Sans CJK SC',-22,'bold'))
        end=model.now;begin=max(model.origin or end,end-60.)
        series={key:[p for p in model.speed_traces.get(key,[]) if p[0]>=begin]
                for key in ('coop_car','coop_uav')}
        maximum=max(.4,max((p[1] for values in series.values() for p in values),default=0.)*1.15)
        duration=max(1.,end-begin)
        for fraction in (0.,.5,1.):
            y=h-30-fraction*(h-90)
            c.create_line(60,y,w-15,y,fill='#e2eaf0')
            c.create_text(53,y,text=f'{maximum*fraction:.2f}',anchor='e',fill='#6a8092')
        for key,color in (('coop_car','#2b8c6b'),('coop_uav','#d78621')):
            chunk=[];previous=None
            for at,speed,_ in series[key]:
                if previous is not None and at-previous>.5:
                    if len(chunk)>=4:c.create_line(*chunk,fill=color,width=2)
                    chunk=[]
                chunk.extend((60+(at-begin)/duration*(w-80),h-30-speed/maximum*(h-90)));previous=at
            if len(chunk)>=4:c.create_line(*chunk,fill=color,width=2)
        c.create_text(60,h-10,text=f'{max(0.,begin-(model.origin or begin)):.0f}s',fill='#6a8092')
        c.create_text(w-35,h-10,text=f'{end-(model.origin or end):.1f}s',fill='#6a8092')

    def tick(self):
        now=time.monotonic(); dt=min(now-self.last_tick, .5); self.last_tick=now
        if not self.poll_run():return
        if self.auto_live and not self.runner.running and now-self.last_discovery>=2.:
            self.last_discovery=now
            directory=active_joint_directory(RUNS,now=now)
            if directory and (not self.session or self.session.directory!=directory.resolve()):
                self.session=Session(directory,follow=True);self.source.set('车机共同坐标')
                self.playing=True;self.event_version=-1
        s=self.session
        if not s:
            self.comparison_text.set('统一坐标位置对照：等待本轮数据')
            self.banner.set('原联合程序正在准备连接 · 等待本轮日志' if self.runner.running else '尚未启动车机 · 点击①准备连接，或打开历史记录')
            self.telemetry.set('尚未收到本轮位置数据。\n连接准备完成后，\n轨迹和速度会自动显示。')
            self.path_text.set('显示读取日志；运行按钮复用原联合入口。小车就位与起飞分别确认。')
            for key,label in (('coop_uav','无人机'),('coop_car','小车')):self.speed_labels[key].set(label+'  — m/s')
            self.map.delete('all');self.height.delete('all');self.speed_plot.delete('all')
            self.map.create_text(40,55,anchor='w',text='等待实时车机位置，或打开已有记录回放',fill='#526477',font=('Noto Sans CJK SC',-29))
            issue=self.runner.issue
            self.text(self.detail,('发生阶段：'+issue['phase']+'\n原因：'+issue['reason']+'\n处理：'+issue['action']+'\n原始报文：'+issue['raw']) if issue else
                '①准备连接\n②小车自动就位\n③生成起飞确认\n最终确认后自动跟随、返回和降落。\n\n准备和就位不会自动起飞。')
        else:
            if self.playing or s.follow or s.seeking: s.pump(dt, all_available=getattr(self,'fast_forward',False))
            if s.seeking:
                # Spend the short batches on decoding, not repeatedly drawing
                # intermediate prefixes. Keep Tk responsive during a long seek.
                self.banner.set('历史回放 · 正在恢复所选时刻的轨迹与状态…')
                self.update_timeline();self.root.after(10,self.tick);return
            if s.eof:self.fast_forward=False
            m=s.model; source='coop_car' if self.source.get()=='小车世界坐标' else 'coop_uav' if self.source.get()=='车机共同坐标' else 'vicon' if self.source.get().startswith('Vicon') else 'px4'
            reference=now if s.follow else m.now
            mode='日志跟随' if s.follow else '历史回放'
            state=('已读到文件末尾' if s.eof else '读取中') if s.follow else ('回放结束' if s.eof else '播放中' if self.playing else '已暂停')
            if s.seeking:state='正在跳转，重建所选时刻的状态'
            self.banner.set(f'{mode}  ·  {state}  ·  {(m.now-(m.origin or m.now)):.1f} s  ·  显示与运行分开，起飞需要确认')
            if s.follow:
                ages=[m.age(key,reference) for key in ('coop_car','coop_uav')]
                fresh=all(age is not None and 0<=age<=1. for age in ages)
                self.banner.set(('实时更新' if fresh else '数据已中断 / 等待新样本')+' · '+self.banner.get())
            if m.cooperative_synthetic:self.banner.set('离线车机模拟 · 非实机联动 · '+self.banner.get())
            for key,label in (('coop_uav','无人机'),('coop_car','小车')):
                v=m.motion_value(key,reference)
                self.speed_labels[key].set(f"{label}  {v['horizontal']:.3f} m/s" if v else label+'  — m/s（无新鲜速度）')
            self.path_text.set(str(s.directory))
            def value(key):
                age=m.age(key,reference)
                if age is None:return '未收到'
                suffix=f'  [{age:.1f}s 前]' if age>2 else ''
                val=m.fields.get(key)
                if key=='armed':val='已解锁' if val else '已锁定'
                if key=='landed' and type(val) is bool:val='在地面' if val else '在空中'
                return str(val)+suffix
            b=m.battery(reference)
            battery=f"{b['percent']}%  /  {b['voltage']:.2f} V" if b and b['voltage'] is not None else f"{b['percent']}%" if b else '未知 / 无新鲜记录'
            if b and b['percent']<=30:battery+='  ⚠ 低电量'
            phase=m.status.get('phase')
            lines=[f"任务：{PHASES.get(phase,phase) if phase else '无任务事件记录'}", f"模式：{value('mode')}",
                   f"解锁状态：{value('armed')}", f"落地状态：{value('landed')}", f'电量：{battery}']
            landing=LANDING_WAIT.get(m.status.get('landing_waiting_for'))
            if landing:lines.append('降落阶段：'+landing)
            positions=(('coop_car','小车'),) if source=='coop_car' else (('coop_car','小车'),('coop_uav','无人机'),('coop_target','目标')) if source=='coop_uav' else (('vicon','Vicon'),('px4','PX4 E/N/U'))
            ages=[]
            for key,label in positions:
                p=m.positions.get(key); age=m.age(key,reference)
                lines.append(label+('：未收到位置' if p is None else f'：{p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f} m'))
                if age is not None:ages.append(age)
            if ages:lines.append(f'位置数据年龄（最大）：{max(ages):.2f} s')
            motion=m.motion_value('coop_uav',reference)
            if motion:lines.append(f"无人机垂直速度：{motion['vertical']:+.3f} m/s")
            if source=='coop_uav' and all(k in m.positions for k in ('coop_car','coop_uav')):
                car,uav=m.positions['coop_car'],m.positions['coop_uav']
                lines.append(f'水平间距：{math.hypot(car[0]-uav[0],car[1]-uav[1]):.3f} m（最近位置）')
            self.telemetry.set('\n'.join(lines))
            comparison=m.position_comparison(reference)
            comparison_lines=['位置对照 · Vicon 世界坐标 / 机体参考点']
            if comparison['valid']:
                comparison_lines.extend([
                    'Vicon XYZ：'+', '.join(f'{v:+.3f}' for v in comparison['vicon_body_map_m'])+' m',
                    '飞控 XYZ：'+', '.join(f'{v:+.3f}' for v in comparison['px4_map_m'])+' m',
                    'ΔX/Y/Z：'+', '.join(f'{v*100:+.1f}' for v in comparison['delta_m'])+' cm',
                    f"水平差 {comparison['horizontal_m']*100:.1f} cm · 三维差 {comparison['total_m']*100:.1f} cm",
                    f"样本年龄 {comparison['age_s']:.2f}s · 配对间隔 {comparison['pair_gap_s']*1000:.0f}ms",
                    '差值=飞控−Vicon；按接收时间插值，包含传输时延影响。'])
            else:comparison_lines.append(comparison['reason'])
            self.comparison_text.set('\n'.join(comparison_lines))
            detail=['原始轨迹：Vicon 为刚体原点，PX4 为飞控本地坐标，分别显示。蓝色位置对照面板使用本轮配准后的机体参考点。']
            if source=='coop_uav':
                detail=['车机图使用记录中明确声明的共同坐标和版本；不将 PX4 本地坐标直接叠加。',
                    '当前是合成运动学模型，不能证明实机跟随或平台降落精度。' if m.cooperative_synthetic else '当前显示协同记录。',
                    '小车意图：'+str(m.cooperative_status.get('car_action'))+'；无人机意图：'+str(m.cooperative_status.get('uav_action')),
                    '路线进度：'+str(m.cooperative_status.get('route_index'))+'/'+str(m.cooperative_status.get('route_count')),
                    '速度为 Vicon 实测位置的滑动拟合值，不是设定速度；约半秒窗口会平滑变化。']
            if s.follow:detail.append('若原飞行/定位程序未运行，本窗口不会自行连接硬件；旧读数会显示年龄。')
            else:detail.append('当前是历史记录，电量和状态不代表现在的无人机。')
            if m.status.get('operator_action'):detail.append(m.status['operator_action'])
            if m.status.get('missing'):detail.append('程序报告缺少：\n'+'\n'.join(m.status['missing']))
            if m.status.get('failure'):detail.append('最近任务故障：\n'+describe(m.status['failure']))
            if m.status.get('failure_snapshot'):detail.append('原始故障快照：\n'+json.dumps(m.status['failure_snapshot'],ensure_ascii=False,indent=2))
            if m.report.get('result'):detail.append('本记录结论：\n'+describe(m.report['result']))
            if m.report.get('failure'):detail.append('本记录停止原因：\n'+describe(m.report['failure']))
            if any(t.changed for t in s.tails):detail.append('日志文件被替换/截短，请重新选择会话，避免跨会话混合。')
            if self.runner.issue:
                issue=self.runner.issue
                detail.insert(0,'发生阶段：'+issue['phase']+'\n原因：'+issue['reason']+'\n处理：'+issue['action']+'\n原始报文：'+issue['raw'])
            self.text(self.detail,'\n\n'.join(detail))
            if m.sequence != self.event_version:
                self.events.configure(state='normal');self.events.delete('1.0','end')
                for e in m.events:
                    self.events.insert('end',f"[{e['at']-(m.origin or e['at']):6.1f}s] {e['level']} {e['source']}  {e['text']}\n",e['level'])
                self.events.see('end'); self.events.configure(state='disabled'); self.event_version=m.sequence
            self.plots(m,source)
            if self.rviz:self.rviz.draw(m,source,not s.follow)
        self.update_timeline()
        self.root.after(100,self.tick)

    def close(self):
        if self.runner.running:
            from tkinter import messagebox
            if messagebox.askyesno('先停止并收尾','联合后端仍在运行。是否请求原程序停车/降落，并在确认收尾后关闭窗口？\n不会立即杀死后端或空中停桨。',parent=self.root):
                self.close_pending=True;self.stop_run()
            return
        if self.rviz:self.rviz.close()
        self.root.destroy()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,help='已有飞行/采集日志目录')
    parser.add_argument('--follow',action='store_true',help='跟随增长中的日志；默认历史回放')
    parser.add_argument('--live',action='store_true',help='自动接入新一轮联合运行日志；不启动控制程序')
    parser.add_argument('--rviz',action='store_true',help='同时打开本机隔离域 RViz 显示')
    parser.add_argument('--summary',action='store_true',help='只离线解析并输出摘要，不打开窗口')
    args=parser.parse_args(argv)
    if args.live and (args.session or args.follow or args.summary):parser.error('--live 不与 --session/--follow/--summary 同用')
    if (args.summary or args.follow) and not args.session:parser.error('请指定 --session')
    if args.summary and args.follow:parser.error('--summary 仅用于历史记录')
    session=Session(args.session,follow=args.follow) if args.session else None
    if args.summary:
        while not session.eof:session.pump(all_available=True)
        m=session.model
        print(json.dumps(dict(rows=m.rows,positions=m.positions,points={k:len(v) for k,v in m.traces.items()},
            fields=m.fields,events=list(m.events),crc=m.parser.summary(),source=str(session.directory),
            historical=True,hardware_connections=0,control_messages_sent=0),ensure_ascii=False,indent=2))
        return 0
    import tkinter as tk
    root=tk.Tk();ui=Console(root,session,auto_live=args.live)
    if args.rviz:root.after(300,ui.open_rviz)
    root.mainloop();return 0


if __name__=='__main__':raise SystemExit(main())
