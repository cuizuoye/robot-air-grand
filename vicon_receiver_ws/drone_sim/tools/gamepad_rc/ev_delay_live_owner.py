"""Joint collector's selected radio/Vicon owner, driven by one shared pump.

No normal flight runner, no dynamic EKF setup, no parameter recovery.
"""
import json
import math
import time

from pymavlink.dialects.v20 import common as m
import compact_vision
import low_bandwidth_vision
from mavlink_crc_stream import CrcResyncParser
from mavlink_readonly_probe import readonly_message
from one_home_preflight_policy import StartupHome
from short_status_transport import ShortStatusTransport
from static_lock_gate import StaticLockGate
from frozen_target_stream import source_age, clock_bounds_us
from ev_delay_maintenance import MaintenanceObservation, IsolatedGroundState
from trs_powered_fusion_bench import _write_owner_packet
from uav_coord_bridge.mavlink_vision import transform_rigid_pose
from vicon_fc_attitude_check import attitude_comparison

READ_PARAMS = ('EKF2_EV_NOISE_MD', 'EKF2_EVP_NOISE', 'EKF2_EVA_NOISE',
               'EKF2_EV_CTRL', 'EKF2_EV_DELAY', 'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z')


def proven_attitude(endpoint, clock, now):
    for msg, received in reversed(endpoint.attitudes):
        try:
            source_age(clock, int(msg.time_boot_ms), received, now, maximum=.5, label='maintenance attitude')
        except RuntimeError:
            continue
        return msg, received
    raise RuntimeError('No fresh source-timestamped FC attitude on '+str(endpoint.port.port))


class Owner:
    def __init__(self, endpoint, usb, source, pad, journal, *, mount, offset, boot_session,
                 created_ns, pump, physical, notice=print, motion_ready=None, transport='trs'):
        if transport not in ('trs','wifi'): raise ValueError('Unknown capture transport')
        if transport == 'wifi' and pad is not None: raise ValueError('WiFi capture does not use a gamepad')
        self.transport = transport
        # Reuse the established 32 Hz capture cadence. 20 Hz leaves no
        # scheduling margin below the analyzer's 50 ms visual interval bound.
        self.period_ns = 31_250_000
        self.ep, self.port, self.usb = endpoint, endpoint.port, usb
        self.source, self.pad, self.pump_all = source, pad, pump
        self.clock = endpoint.clock
        self.mount, self.offset, self.boot_session = mount, offset, boot_session
        self.created_ns, self.notice = created_ns, notice
        self.motion_ready = motion_ready
        self.motion_cue_ns = None
        self.physical_conditions = dict(physical)
        self.physical = (all(self.physical_conditions.get(k) is True for k in
            ('battery_disconnected', 'usb_only', 'motor_power_isolated',
             'independent_cutoff_ready', 'scope_manually_approved'))
            and (self.physical_conditions.get('props_removed'), self.physical_conditions.get('props_installed'))
                in ((True, False), (False, True)))
        if not self.physical: raise PermissionError('Explicit USB-only motor power isolation required')
        self.generation = '' if transport == 'wifi' else json.dumps(pad.generation, sort_keys=True)
        self.home = None if transport == 'wifi' else StartupHome()
        self.lock = (IsolatedGroundState if transport == 'wifi' else StaticLockGate)(source=(1, 1), target=(250, 213))
        self.rates = ShortStatusTransport(self.lock, endpoint.enc, journal)
        if transport == 'wifi': self.rates.enabled = True  # Clock probes only; no rate changes.
        self.restoring = self.configuring = self.home_enabled = False
        self.home_sent = False
        self.home_sent_ns = None
        self.config = None
        self.session = None
        self.stop_at_ns = self.next_pose_ns = 0
        self.alignment = None
        self.pose_enabled = False
        self.warming = False
        self.warmup_gate = None
        self.warmup_pending = {}
        self.warmup_sent = []
        self.quiesced = False
        self.cleanup_started = False
        self.closed = False
        self.fault = None
        self.wire_decoder = CrcResyncParser()
        self.report = dict(parameter_writes=0, poses=0, errors=[], boot_binding=[],
            physical_conditions=dict(self.physical_conditions))
        self.next_notice = 0
        self.ep.writer, self.ep.hook = self.send, self.receive
        self.usb.peer_hook = self.receive_forwarded

    def identity(self):
        return self.ep.safe() and self.usb.safe() and self.ep.fw_custom == self.usb.fw_custom

    def receive(self, msg, at):
        history = len(self.lock.clock.history)
        prior = self.lock.clock.history[-1] if self.lock.clock.history else None
        self.rates.ingest(msg, at)
        if self.session and self.pose_enabled and at < self.stop_at_ns:
            current = self.lock.clock.history[-1] if self.lock.clock.history else None
            if current != prior or len(self.lock.clock.history) != history:
                if not self.session.observe_clock(at_ns=at): raise RuntimeError(self.session.gate.fault)
        if (self.home is not None and msg.get_type() == 'COMMAND_ACK' and msg.command == 185 and self.home_sent
                and (msg.target_system, msg.target_component) == (250, 213)
                and self.home.ack_at is None):
            if msg.result != m.MAV_RESULT_ACCEPTED or not 0 <= at-self.home_sent_ns < 1_000_000_000:
                raise RuntimeError('Current HOME rejected/late')
            self.home.ack(at/1e9)

    def send(self, msg, *, source=None):
        kind = msg.get_type()
        now = self.clock()
        if self.transport == 'wifi' and kind == 'COMMAND_LONG' and msg.command in (185,510,511):
            raise PermissionError('WiFi isolated capture forbids output/rate commands')
        if kind == 'VISION_POSITION_ESTIMATE' and self.warming:
            self.check_warmup()
            status = self.report['visual_preroll']
            if (self.session is not None or self.quiesced or self.cleanup_started
                    or source is None or now >= self.warmup_deadline
                    or status['sent'] >= 160
                    or status['last_sent_ns'] is not None and now-status['last_sent_ns'] < self.period_ns
                    or not low_bandwidth_vision.pose_allowed(msg, {k:v['value'] for k,v in self.config.items()})):
                raise PermissionError('Stationary visual pre-roll policy rejected')
            self.ep.write(msg)  # Same owner, encoder, bounded serial write and TX journal.
            wire = bytes(msg.get_msgbuf()).hex()
            sent = self.clock()
            self.warmup_pending[wire] = sent
            if self.transport == 'wifi':
                p, q = transform_rigid_pose(source['world_pose'][:3], source['world_pose'][3:],
                                           self.offset, self.mount)
                self.warmup_sent.append(dict(source_stamp_ns=source['ros_stamp_ns'],
                    position=[msg.x,msg.y,msg.z], q=compact_vision.aligned_pose(p,q,self.alignment)[1]))
            status['sent'] += 1
            status['last_sent_ns'] = sent
            return True
        if self.session and self.pose_enabled:
            if now >= self.stop_at_ns:
                self.pose_enabled = False
                return False
            if now >= self.stop_at_ns-50_000_000:
                return False  # Do not begin a potentially 20ms write at the deadline.
            packet = bytes(msg.pack(self.ep.enc))
            _write_owner_packet(self.port, packet, recording=self.session, original=msg,
                observation=self.observation, clock=self.clock, wire_decoder=self.wire_decoder,
                source_stamp_ns=None if source is None else source['ros_stamp_ns'],
                source_frame_id=None if source is None else source['frame_id'],
                noise_parameters={k: v['value'] for k, v in self.config.items()})
            self.ep.enc.seq = (self.ep.enc.seq+1) % 256
            self.ep.tx.append(dict(at_ns=self.clock(), type=kind, wire_hex=packet.hex()))
            return True
        allowed = kind in ('HEARTBEAT', 'COMMAND_LONG') and readonly_message(msg)
        if kind == 'PARAM_REQUEST_READ':
            name = msg.param_id.decode() if isinstance(msg.param_id, bytes) else msg.param_id
            allowed = (self.identity() and name.rstrip('\0') in READ_PARAMS
                and (msg.target_system, msg.target_component, msg.param_index) == (1, 1, -1))
        if kind == 'COMMAND_LONG' and msg.command == 185:
            allowed = bool(self.physical and self.home_enabled and self.identity() and self.home.needs_stop and not self.home_sent
                and not self.home.aborted and (msg.target_system, msg.target_component, msg.confirmation) == (1, 1, 0)
                and msg.param1 == 2 and all(getattr(msg, 'param'+str(i)) == 0 for i in range(2, 8)))
            if allowed: self.home_sent = True; self.home_sent_ns = now
        if kind == 'TIMESYNC' or kind == 'COMMAND_LONG' and msg.command in (510, 511):
            # Recovery is restricted by ShortStatusTransport to the saved rate
            # and fresh unarmed same-endpoint identity. It must not depend on
            # a capture lock that has expired/closed, or on USB already closed
            # by the logger finalizer. Never clear the capture's failure latch.
            allowed = self.physical and self.rates.allowed(msg, identity=self.ep.safe(),
                locked=False if self.restoring else self.lock.assess(now, identity_verified=self.ep.safe())['valid'],
                restoring=self.restoring, bootstrap=False if self.restoring or self.home is None else self.home.short_bootstrap_ready())
        if not allowed: raise PermissionError(self.transport+' maintenance policy rejected '+kind)
        self.ep.write(msg)

    def verify_configuration(self):
        if self.pose_enabled: raise RuntimeError('No config queries in capture window')
        current = {name: self.ep.read(name, self.pump_all) for name in READ_PARAMS}
        low_bandwidth_vision.noise_evidence({k: v['value'] for k, v in current.items()})
        if self.config is not None:
            if any((r['type'], r['wire_value_hex']) != (self.config[n]['type'], self.config[n]['wire_value_hex'])
                   for n, r in current.items()):
                raise RuntimeError('EKF parameters changed; no automatic restoration')
            self.report['parameters_after'] = current
        else:
            self.config = current
            self.report['parameters_before'] = current
        return True

    def wait_ground_ready(self):
        if self.transport != 'wifi': raise RuntimeError('Explicit WiFi isolation route required')
        deadline = self.clock()+5_000_000_000
        while not self.lock.assess(self.clock(), identity_verified=self.identity())['valid']:
            self.pump_all()
            if self.clock() >= deadline: raise RuntimeError('Current unarmed/clock evidence unavailable')
        self.report['preparation_basis'] = 'operator-confirmed motor isolation + actual unarmed heartbeat and FC clock; no HOME'

    def frame(self, row, at):
        if self.session and self.pose_enabled and at < self.stop_at_ns:
            previous = self.session.adapter.previous
            if not self.session.observe_frame(row, callback_ns=at):
                self.report.setdefault('first_failed_source_frame', dict(
                    previous=None if previous is None else dict(previous),
                    rejected=dict(row), callback_ns=at, reason=self.session.gate.fault))
                raise RuntimeError(self.session.gate.fault)

    def source_pose(self):
        row = self.source.latest
        if row is None: raise RuntimeError('No Vicon FrameSample')
        if ((row['subject'], row['segment']) != ('MircoAir', 'MircoAir')
                or not row['sdk_ok'] or row['occluded'] or not row['pose_published']
                or row['queue_dropped'] or row['frame_id'] != 'map'):
            raise RuntimeError('Invalid/occluded Vicon source')
        age = self.source.now_ros_ns()-row['ros_stamp_ns']
        if not 0 <= age < 150_000_000: raise RuntimeError('Vicon source header stale/future')
        p, q = transform_rigid_pose(row['world_pose'][:3], row['world_pose'][3:], self.offset, self.mount)
        return row, p, q, age

    def observation(self, now):
        if not self.identity(): raise RuntimeError('USB/'+self.transport+' identity or continuous boot binding lost')
        # Both streams must fit the SAME request-bounded FC boot clock. Host
        # receive time is never substituted for either source timestamp.
        attitude, received = proven_attitude(self.ep, self.lock.clock, now)
        ua, ur = proven_attitude(self.usb, self.lock.clock, now)
        row, p, q, age = self.source_pose()
        fcq = [attitude.q1, attitude.q2, attitude.q3, attitude.q4]
        comparison = attitude_comparison(q, fcq)
        tilt = comparison['gravity_axis_difference_deg']
        # Retain the exact input even when the gate refuses the first frame.
        # Before bind_recording the raw recorder has no samples yet.
        pair = dict(at_ns=now, source_frame=dict(row), transformed_wxyz=list(q),
            fc_attitude=attitude.to_dict(), fc_received_ns=received,
            usb_attitude=ua.to_dict(), usb_received_ns=ur,
            comparison=comparison, source_header_age_ns=age,
            mount_xyzw=list(self.mount), offset_m=list(self.offset))
        self.report['last_tilt_pair'] = pair
        dynamic = self.session is not None and self.session.gate.started_ns is not None
        pair['comparison_scope'] = ('unpaired_dynamic_diagnostic' if dynamic
                                    else 'stationary_start_alignment')
        if dynamic and math.isfinite(tilt) and 0 <= tilt <= 180:
            diagnostic = self.report.setdefault('dynamic_tilt_diagnostic', dict(
                observations=0, max_difference_deg=0., above_static_threshold=0,
                time_paired=False, calibration_approved=False))
            diagnostic['observations'] += 1
            diagnostic['max_difference_deg'] = max(diagnostic['max_difference_deg'], tilt)
            diagnostic['above_static_threshold'] += int(tilt > 5)
            if tilt > 5:
                self.report.setdefault('first_dynamic_tilt_disagreement_pair', pair)
        elif not math.isfinite(tilt) or not 0 <= tilt <= 5:
            self.report.setdefault('first_failed_tilt_pair', pair)
        return MaintenanceObservation(now_ns=now, uid=self.ep.expected['uid2'], firmware='1.14.3',
            fc_boot=self.boot_session, identity_received_ns=self.ep.identity_ns,
            attitude_received_ns=received, identity_continuity_received_ns=received,
            vicon_received_ns=row['callback_ns'], gamepad_generation=self.generation,
            tilt_difference_deg=tilt, **self.physical_conditions,
            log_ready=self.logger.report.get('logger_active_confirmed_ns') is not None,
            source_valid=True, config_readonly_unchanged=self.config is not None,
            recorder_healthy=self.session is None or not self.session.recorder.fault,
            attitude_source_fresh=True)

    def bind_recording(self, session, *, stop_at_ns):
        now = self.clock()
        row, p, q, age = self.source_pose()
        attitude, received = proven_attitude(self.ep, self.lock.clock, now)
        if not self.report.get('visual_preroll', {}).get('received_confirmed'):
            raise RuntimeError('Visual pre-roll receipt not confirmed')
        # Keep the pre-roll alignment: no coordinate jump at GO.
        self.session, self.stop_at_ns = session, stop_at_ns
        session.recorder.metadata.update(alignment_yaw_rad=self.alignment,
            body_orientation_in_rigid_xyzw=list(self.mount),
            ulog_boot_binding_evidence='report.json: same UID/custom firmware and both actual boot timestamps bounded by '+self.transport+' TIMESYNC',
            shared_fc_clock_evidence='report.json: usb/'+self.transport+' identity wires, boot_binding and calibration_clock events')
        ua, ur = proven_attitude(self.usb, self.lock.clock, now)
        self.report['boot_binding'].append(dict(at_ns=now, bounds_us=clock_bounds_us(self.lock.clock, now),
            pose_route=self.transport,
            radio_boot_ms=int(attitude.time_boot_ms), radio_received_ns=received,
            usb_boot_ms=int(ua.time_boot_ms), usb_received_ns=ur,
            radio_identity_received_ns=self.ep.identity_ns, usb_identity_received_ns=self.usb.identity_ns))
        self.pose_enabled = True
        self.source.on_frame = self.frame
        # Retain the actual last callback time; do not retime the seed frame.
        self.frame(row, row['callback_ns'])
        if not session.observe_clock(at_ns=self.clock()): raise RuntimeError(session.gate.fault)
        self.next_pose_ns = self.clock()
        self.next_notice = 0
        session.recorder.metadata['motion_window'] = dict(
            started_ns=session.gate.started_ns, deadline_ns=stop_at_ns,
            go_ns=self.report.get('motion_go_ns'), cue_ns=None, ended_ns=None)
        session.recorder.metadata['visual_preroll'] = dict(self.report['visual_preroll'],
            motion_start_ns=session.gate.started_ns,
            motion_start_clock_reference=dict(self.lock.clock.reference),
            motion_start_fc_bounds_us=list(clock_bounds_us(self.lock.clock, session.gate.started_ns)))

    def receive_forwarded(self, msg, at):
        """Exact CRC-valid USB echo proves FC receipt, not Logger subscription."""
        if self.transport == 'wifi' or not self.warming or msg.get_type() != 'VISION_POSITION_ESTIMATE': return
        sent = self.warmup_pending.pop(bytes(msg.get_msgbuf()).hex(), None)
        if sent is None or not 0 <= at-sent < 500_000_000: return
        status = self.report['visual_preroll']
        status['received'] += 1
        if status['first_received_ns'] is None: status['first_received_ns'] = at
        status['last_received_ns'] = at

    def check_warmup(self):
        if not self.warmup_gate.check_preparation(self.observation(self.clock())):
            raise RuntimeError(self.warmup_gate.fault or 'Visual pre-roll proof lost')
        _, p, q, _ = self.source_pose()
        p0, q0 = self.warmup_origin
        angle = math.degrees(2*math.acos(min(1., abs(sum(a*b for a,b in zip(q,q0))))))
        if math.dist(p, p0) > .02 or angle > 5:
            raise RuntimeError('Keep stationary during visual pre-roll; capture not started')

    def prepare_visual_preroll(self, gate):
        self.warmup_gate = gate
        if not gate.check_preparation(self.observation(self.clock())):
            raise RuntimeError(gate.fault or 'Visual pre-roll preparation rejected')
        _, p, q, _ = self.source_pose()
        attitude, _ = proven_attitude(self.ep, self.lock.clock, self.clock())
        self.alignment = compact_vision.yaw_alignment(q, [attitude.q1,attitude.q2,attitude.q3,attitude.q4])
        self.warmup_origin = (p, q)
        started = self.clock()
        self.warmup_deadline = started+5_000_000_000
        status = self.report['visual_preroll'] = dict(scope='stationary-before-GO-v1',
            started_ns=started, ended_ns=None, sent=0, received=0,
            first_received_ns=None, last_received_ns=None, last_sent_ns=None,
            received_confirmed=False, logger_subscription_verified=False)
        self.notice('日志已启动。保持静止：正在预热定位并等待飞控回传；暂不要输入 GO。', flush=True)
        self.warming = True
        self.warmup_sent = []
        self.next_pose_ns = started
        # USB shell can outrun the first wireless pose. Allow the current
        # preroll to arrive before inspecting its native receipt.
        next_read = started+300_000_000
        last_visual_stamp = 0
        try:
            while True:
                self.pump_all()
                self.check_warmup()
                now = self.clock()
                if self.transport == 'wifi' and self.warmup_sent and now >= next_read:
                    visual = self.logger.read_visual()
                    received = self.clock()
                    match = compact_vision.match(visual,self.warmup_sent)
                    if visual['timestamp'] <= last_visual_stamp or match is None:
                        raise RuntimeError('Native visual pre-roll not fresh/matching current sends')
                    last_visual_stamp = visual['timestamp']
                    status['received'] += 1
                    if status['first_received_ns'] is None: status['first_received_ns'] = received
                    status['last_received_ns'] = received
                    status['receipt_source'] = 'USB native vehicle_visual_odometry; no forwarding required'
                    status.setdefault('snapshots', []).append(dict(at_ns=received, visual=visual, match=match))
                    next_read = received+700_000_000
                    now = received
                if now >= self.warmup_deadline:
                    raise RuntimeError('Visual pre-roll receipt timeout; no GO or automatic retry')
                if (now-started >= 3_000_000_000 and status['received'] >= 3
                        and now-status['first_received_ns'] >= 2_000_000_000
                        and 0 <= now-status['last_received_ns'] < 500_000_000):
                    status['received_confirmed'] = True
                    break
        finally:
            self.warming = False
            status['ended_ns'] = self.clock()
            self.warmup_pending.clear()
            self.warmup_sent.clear()

    def prepare_recording(self, gate):
        if self.motion_ready is None:
            raise RuntimeError('Local GO callback required; no automatic action window')
        self.notice('日志已启动。保持静止，正在记录动作前的IMU。', flush=True)
        until = self.clock()+2_000_000_000
        while self.clock() < until:
            self.pump_all()
            if not self.identity(): raise RuntimeError('Identity lost during IMU pre-roll')
        self.prepare_visual_preroll(gate)
        if self.motion_ready is not None:
            self.notice('静止定位预热完成，已确认飞控接收。动作窗口尚未开始：现在可输入 GO，继续保持静止等待倒计时。', flush=True)
            deadline = self.clock()+60_000_000_000
            while not self.motion_ready():
                self.pump_all()
                if not self.identity(): raise RuntimeError('Identity lost waiting for motion cue')
                if self.clock() >= deadline: raise RuntimeError('Motion cue timeout; no automatic start')
            self.report['motion_go_ns'] = self.clock()
            # Countdown is before gate.begin(), so none of the 15 seconds is
            # lost to human reaction or chat latency. Keep the same pump alive.
            for count in (3, 2, 1):
                self.notice('准备 '+str(count)+'：保持静止，等“开始动作”。', flush=True)
                until = self.clock()+1_000_000_000
                while self.clock() < until:
                    self.pump_all()
                    if not self.identity(): raise RuntimeError('Identity lost during motion countdown')

    def announce_motion(self):
        """Called only after the first successful recorded pose write."""
        if self.motion_cue_ns is not None:return
        now=self.clock()
        if not 0 <= now-self.session.gate.started_ns <= 500_000_000:
            raise RuntimeError('First recorded pose too late; no motion cue or window extension')
        self.motion_cue_ns=now
        self.session.recorder.metadata['motion_window']['cue_ns']=now
        self.report['motion_cue_ns']=now
        self.notice('\a开始动作：记录和定位发送已开始。前后、左右轻缓交替倾斜约10–15度，变化节奏；按本机提示换段和放稳。', flush=True)

    def pump(self):
        now = self.clock()
        if self.restoring or self.cleanup_started:
            # Keep only the owned telemetry/clock path alive during cleanup.
            # A failed/removed gamepad or source cannot prevent rate restoration.
            self.rates.probe(now, self.send, identity=self.ep.safe(), restoring=self.restoring)
            self.ep.heartbeat()
            return
        if self.pose_enabled and now >= self.stop_at_ns: self.stop_pose_writes()
        if self.transport == 'trs':
            events = self.pad.poll()['home_events'] if not self.closed else ()
            for down in events:
                if not self.home_enabled: raise RuntimeError('HOME pressed before readiness; release and start a fresh run')
                self.home.event(1 if down else 0, now/1e9)
                if self.home.aborted: raise RuntimeError('Repeated/held HOME: capture stopped')
                if self.home.needs_stop and not self.home_sent:
                    self.send(self.ep.enc.command_long_encode(1, 1, 185, 0, 2, 0, 0, 0, 0, 0, 0))
            if self.home.press is not None and self.home.release is None and now/1e9-self.home.press > 1:
                raise RuntimeError('HOME held longer than one second')
            if self.home_sent and self.home.ack_at is None and now-self.home_sent_ns >= 1_000_000_000:
                raise RuntimeError('HOME ACK timeout; no retry')
            if (self.home.short_bootstrap_ready() and not self.rates.changed and not self.configuring and not self.quiesced):
                self.configuring = True
                try: self.rates.configure(self.send, self.pump_all, self.clock)
                finally: self.configuring = False
        now = self.clock()
        self.rates.probe(now, self.send, identity=self.ep.safe(), restoring=self.restoring)
        if self.session and self.pose_enabled:
            latest = self.lock.clock.history[-1] if self.lock.clock.history else None
            if latest and latest['event'] == 'request' and latest['at_ns'] == now:
                self.session.observe_clock(at_ns=self.clock())
        if self.home is not None and self.home.short_bootstrap_ready() and not self.home.verified:
            self.home.observe_short(self.lock, now_ns=self.clock(),
                post_home_ns=int(max(self.home.ack_at, self.home.release)*1e9), identity_verified=self.identity())
        self.ep.heartbeat()
        if self.warming:
            self.check_warmup()
            now = self.clock()
            if now >= self.warmup_deadline:
                raise RuntimeError('Visual pre-roll receipt timeout; no GO or automatic retry')
            if now >= self.next_pose_ns:
                row, p, q, age = self.source_pose()
                msg, _, _ = low_bandwidth_vision.message(self.ep.enc, row['ros_stamp_ns']//1000 if self.transport == 'wifi' else (now-age)//1000,
                    p, q, self.alignment, {k:v['value'] for k,v in self.config.items()})
                self.send(msg, source=row)
                self.next_pose_ns = self.clock()+self.period_ns
        if self.pose_enabled:
            now = self.clock()
            if now >= self.stop_at_ns: self.stop_pose_writes(); return
            if now >= self.next_pose_ns:
                row, p, q, age = self.source_pose()
                msg, _, _ = low_bandwidth_vision.message(self.ep.enc, row['ros_stamp_ns']//1000 if self.transport == 'wifi' else (now-age)//1000,
                    p, q, self.alignment, {k: v['value'] for k, v in self.config.items()})
                sent = self.send(msg, source=row)
                self.next_pose_ns = self.clock()+self.period_ns
                self.report['poses'] += bool(sent)
                if sent:self.announce_motion()
            elapsed = (now-self.session.gate.started_ns)/1e9
            stages = [(7, '继续横滚和俯仰，换一种不规则节奏，不要只转航向。'),
                      (14, '缓慢回正，等待采集结束提示。')]
            if self.next_notice < len(stages) and elapsed >= stages[self.next_notice][0]:
                self.notice(stages[self.next_notice][1], flush=True)
                self.next_notice += 1

    def stop_pose_writes(self):
        self.warming = False
        self.warmup_pending.clear()
        if self.session is not None and 'motion_window' in self.session.recorder.metadata:
            window=self.session.recorder.metadata['motion_window']
            if window['ended_ns'] is None:window['ended_ns']=self.clock()
        if self.pose_enabled: self.notice('定位发送已停止，请放稳无人机并保持USB供电，正在关闭和导出日志。', flush=True)
        self.pose_enabled = False
        self.quiesced = True

    def stop_inputs(self):
        self.cleanup_started = True
        self.source.stop_inputs()
    def detach_recording(self): self.session = None

    def close_owned(self):
        if self.closed: return
        errors = []
        self.stop_pose_writes()
        try:
            self.restoring = True
            self.rates.restore(self.send, self.pump_all, self.clock)
        except BaseException as exc: errors.append('rate restore: '+str(exc))
        self.report.update(home=None if self.home is None else self.home.status(), rates=self.rates.summary(), clock=self.lock.clock.summary())
        for label, close in ((self.transport, self.port.close), ('Vicon', self.source.close), *([('gamepad', self.pad.close)] if self.pad is not None else [])):
            try: close()
            except BaseException as exc: errors.append(label+': '+str(exc))
        self.closed = True
        self.report['cleanup_errors'] = errors
        if errors: raise RuntimeError('; '.join(errors))
