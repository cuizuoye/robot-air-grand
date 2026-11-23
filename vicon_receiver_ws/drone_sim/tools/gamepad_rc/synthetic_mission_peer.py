"""Synthetic PX4/TRS/Vicon peer, in memory only. NOT flight dynamics or SITL.

Uses actual MAVLink bytes and fixed NSH text parsers. All telemetry, ACKs,
fusion/Commander state and kinematic motion here are explicitly fabricated
test stimuli, never evidence about a physical aircraft.
"""
from collections import deque
import struct

from pymavlink.dialects.v20 import common as m

from continuous_mission_runtime import ContinuousMissionRuntime
from mavlink_crc_stream import CrcResyncParser
from mission_audit_collector import COMMANDS
from trs_uart_readonly_check import UID2
import control_readonly_policy as control
import trs_vision_reference_policy as vision


def synthetic_baseline():
    values = dict.fromkeys(vision.READ_PARAMS, 0)
    values.update(EKF2_HGT_REF=3, EKF2_EV_CTRL=2, EKF2_BARO_CTRL=1,
                  COM_RC_IN_MODE=4, COM_OBL_RC_ACT=4)
    return values


class SyntheticPeer:
    dt = .01

    def __init__(self, *, height=.3, hover_seconds=3., scenario='nominal',
                 fixed_configuration=False, fixed_profile=None):
        self.runtime = ContinuousMissionRuntime('SYNTHETIC-ONLY-rigid-revision',
            body_origin_in_rigid=[0,0,0], body_orientation_in_rigid=[0,0,0,1],
            height=height, hover_seconds=hover_seconds,
            fixed_configuration=fixed_configuration,fixed_profile=fixed_profile)
        self.encoder = m.MAVLink(None, srcSystem=1, srcComponent=1)
        self.decoder = CrcResyncParser()
        self.pending, self.shell = deque(), deque()
        self.cursor = 0
        self.now = 0.
        self.params = synthetic_baseline()
        if fixed_configuration and fixed_profile is not None:
            self.params={n:s['value'] for n,s in fixed_profile['parameters'].items()}
        self.locked = self.armed = self.offboard = self.landing = False
        self.z = -.1
        self.x,self.y=1.,2.
        self.vx=self.vy=0.
        self.ground_z = self.z
        self.target = None
        self.velocity = 0.
        self.last_pose = None
        self.visual_wait = None
        self.calls = []
        self.scenario = scenario
        self.injected = False
        self.injection_at = None
        self.state_rows = []
        self.boot_offset = 10.
        self.intervals={111:0};self.next_sync_request=0.

    def pack(self, msg):
        wire = msg.pack(self.encoder)
        self.encoder.seq = (self.encoder.seq+1) % 256
        return wire

    def _shell_text(self, text):
        data = text.encode()
        for i in range(0, len(data), 70):
            part = data[i:i+70]
            self.shell.append(self.pack(self.encoder.serial_control_encode(
                10,0,0,0,len(part),part.ljust(70,b'\0'))))

    def _reply_text(self, key):
        if key == 'critical_status':
            parts=[self._reply_text(k).split('TOPIC: ',1)[1].rsplit('nsh> ',1)[0]
                   for k in ('local_position','commander')]
            return 'nsh> '+COMMANDS[key].decode()+''.join('TOPIC: '+p for p in parts)+'nsh> '
        stamp = int((self.now+self.boot_offset)*1e6)
        common = dict(timestamp=stamp)
        if key == 'actuator':
            v = dict(timestamp=stamp, armed=self.armed, lockdown=self.locked,
                     manual_lockdown=False, force_failsafe=False)
        elif key == 'selector':
            v = dict(timestamp=stamp, primary_instance=1, instance_changed_count=2,
                instances_available=2, healthy=[True,True]+[False]*7,
                gyro_fault_detected=False, accel_fault_detected=False,
                accel_device_id=1, gyro_device_id=2, last_instance_change=1,
                baro_device_id=3, mag_device_id=0,
                combined_test_ratio=[0.]*9, relative_test_ratio=[0.]*9)
            if self.injected and self.scenario == 'selector_change':
                v['instance_changed_count'] += 1
        elif key == 'commander':
            v = dict(timestamp=stamp, pre_flight_checks_pass=True,
                     arming_state=2 if self.armed else 1, nav_state=14 if self.offboard else 4)
        elif key == 'local_position':
            v = dict(timestamp=stamp, timestamp_sample=stamp-100, x=self.x,y=self.y,z=self.z,
                heading=0., vx=self.vx,vy=self.vy,vz=self.velocity,eph=0.,epv=0.,evh=0.,evv=0.,
                xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True,
                heading_good_for_control=False,xy_reset_counter=0,z_reset_counter=0,heading_reset_counter=0)
        elif key == 'failsafe':
            v = {**common, **dict.fromkeys(control.FAILSAFE_FLAGS,False)}
        elif key.startswith('status_'):
            v = {**common, 'timestamp_sample':stamp-100, **dict.fromkeys(control.STATUS_FLAGS,False)}
        elif key.startswith('flags_'):
            v = {**common, 'timestamp_sample':stamp-100,
                **dict.fromkeys(('cs_gps','cs_mag_hdg','cs_mag_3d','cs_gps_yaw','cs_ev_vel',
                    'fs_bad_pos_n','fs_bad_pos_e','fs_bad_pos_d','fs_bad_hdg','reject_hor_pos',
                    'reject_ver_pos','reject_yaw','cs_yaw_align','cs_rng_hgt','cs_gps_hgt',
                    'cs_fake_hgt','reject_optflow_y'),False),
                **dict.fromkeys(('cs_tilt_align','cs_baro_hgt','cs_ev_pos','cs_ev_hgt','cs_ev_yaw'),True),
                'cs_in_air':self.armed and self.z < self.ground_z-.01}
        else:
            prefix, slot = key.rsplit('_',1)
            v = dict(timestamp=stamp,timestamp_sample=stamp-100,time_last_fuse=stamp-10,
                estimator_instance=1-int(slot),fusion_enabled=True,fused=True,innovation_rejected=False)
            for field in ('observation','observation_variance','innovation','innovation_variance','test_ratio'):
                v[field] = [.01,.01] if prefix == 'aid' else .01
        topic = COMMANDS[key].decode().split()[1]
        def val(value):
            return '['+', '.join(str(x) for x in value)+']' if isinstance(value,list) else str(value)
        return ('nsh> '+COMMANDS[key].decode()+'TOPIC: '+topic+'\n'
                + ''.join(f' {k}: {val(value)}\n' for k,value in v.items())+'\nnsh> ')

    def output_records(self):
        return list(self.runtime.frames),sum(self.runtime.sent_counts.values())

    def run_owner_turn(self,now,**kwargs):
        return self.runtime.turn(now,**kwargs)

    def inspect_output(self):
        rows,total = self.output_records()
        count = total-self.cursor
        if count > len(rows) or count < 0:
            raise RuntimeError('Synthetic capture ring overflow')
        for row in rows[-count:] if count else ():
            msg = self.decoder.parse_buffer(bytes.fromhex(row['wire_hex']))[0]
            self.calls.append((self.now,msg))
            kind = msg.get_type()
            if kind == 'COMMAND_LONG':
                if msg.command == 512:
                    self.pending.append(self.pack(self.encoder.autopilot_version_encode(
                        0,(1<<24)+(14<<16)+(3<<8),0,0,54,bytes(8),bytes(8),bytes(8),
                        0,0,1,bytes.fromhex(UID2))))
                    continue
                result = m.MAV_RESULT_ACCEPTED
                if msg.command==510:
                    self.pending.append(self.pack(self.encoder.message_interval_encode(
                        message_id=int(msg.param1),interval_us=self.intervals[int(msg.param1)])))
                    continue
                elif msg.command==511:
                    self.intervals[int(msg.param1)]=max(0,int(msg.param2))
                elif msg.command == 185:
                    self.locked = msg.param1 == 2
                    if self.locked:
                        self.armed = False
                elif msg.command == 176:
                    if self.scenario == 'offboard_rejected':
                        result = m.MAV_RESULT_DENIED
                    else:
                        self.offboard = True
                elif msg.command == 400:
                    if self.scenario == 'arm_rejected':
                        result = m.MAV_RESULT_DENIED
                    else:
                        self.armed = True
                elif msg.command == 21:
                    self.landing = True
                else:
                    raise AssertionError('Unexpected synthetic command '+str(msg.command))
                if not (msg.command == 185 and msg.param1 == 0 and self.scenario == 'enable_ack_lost'):
                    self.pending.append(self.pack(self.encoder.command_ack_encode(msg.command,result,
                        target_system=250,target_component=220)))
            elif kind=='TIMESYNC':
                if msg.tc1==0:self.pending.append(self.pack(self.encoder.timesync_encode(
                    int((self.now+self.boot_offset)*1e9),msg.ts1)))
            elif kind in ('PARAM_REQUEST_READ','PARAM_SET'):
                name = msg.param_id.rstrip('\0')
                if kind == 'PARAM_SET':
                    value = msg.param_value if msg.param_type == 9 else struct.unpack('<i',struct.pack('<f',msg.param_value))[0]
                    self.params[name] = value
                    if self.scenario == 'restore_echo_lost' and name == 'EKF2_EV_CTRL' and value == 2:
                        continue
                value = self.params[name]
                if self.runtime.fixed_configuration:
                    from ekf_static_profile import PARAMETER_TYPES
                    from rc_watch import NativeRCMonitor
                    floating={**PARAMETER_TYPES,**NativeRCMonitor.PARAMETER_TYPES}[name]==9
                else:floating = name == 'EKF2_EV_DELAY'
                wire = float(value) if floating else struct.unpack('<f',struct.pack('<i',int(value)))[0]
                self.pending.append(self.pack(self.encoder.param_value_encode(name.encode(),wire,
                    9 if floating else 6,926,0)))
            elif kind == 'VISION_POSITION_ESTIMATE':
                self.last_pose = msg
            elif kind == 'SET_POSITION_TARGET_LOCAL_NED':
                self.target = msg
            elif kind == 'SERIAL_CONTROL' and msg.count:
                payload = bytes(msg.data[:msg.count])
                key = next(k for k,v in COMMANDS.items() if v == payload)
                if self.shell or self.visual_wait is not None:
                    raise AssertionError('Overlapping synthetic NSH transactions')
                if key.startswith('visual_series'):
                    self.visual_wait = self.now
                else:
                    self._shell_text(self._reply_text(key))
        self.cursor = total

    def turn(self, *, home=(), with_pose=True, pad=True, extra_rx=b''):
        self.now = round(self.now+self.dt,9)
        old_x,old_y=self.x,self.y
        if self.armed and self.target:
            self.x+=max(-.1*self.dt,min(.1*self.dt,self.target.x-self.x))
            self.y+=max(-.1*self.dt,min(.1*self.dt,self.target.y-self.y))
        self.vx,self.vy=(self.x-old_x)/self.dt,(self.y-old_y)/self.dt
        old_z = self.z
        if self.landing and self.scenario != 'landing_timeout':
            self.z = min(self.ground_z,self.z+.15*self.dt)
            if self.z >= self.ground_z-1e-8:
                self.armed = False
        elif self.armed and self.target and self.scenario != 'climb_timeout':
            self.z += max(-.15*self.dt,min(.15*self.dt,self.target.z-self.z))
        self.velocity = (self.z-old_z)/self.dt
        boot = int((self.now+self.boot_offset)*1000)
        stream = b''.join(self.pack(msg) for msg in (
            self.encoder.heartbeat_encode(2,m.MAV_AUTOPILOT_PX4,128 if self.armed else 0,
                (6 if self.offboard else 4)<<16,0),
            self.encoder.attitude_quaternion_encode(boot,1,0,0,0,0,0,0),
            self.encoder.local_position_ned_encode(boot,self.x,self.y,self.z,self.vx,self.vy,self.velocity),
            self.encoder.extended_sys_state_encode(0,m.MAV_LANDED_STATE_IN_AIR if self.armed or self.z < self.ground_z-.001
                else m.MAV_LANDED_STATE_ON_GROUND)))
        if self.visual_wait is not None and self.now-self.visual_wait >= .352:
            msg = self.last_pose
            text = 'nsh> '+COMMANDS['visual_series'].decode()
            for i in range(12):
                stamp = int((self.now+self.boot_offset-(11-i)*.032)*1e6)
                text += ('TOPIC: vehicle_visual_odometry\n'
                    f' timestamp: {stamp}\n timestamp_sample: {stamp}\n pose_frame: 1\n'
                    ' velocity_frame: 0\n reset_counter: 0\n quality: 0\n'
                    f' position: [{msg.x}, {msg.y}, {msg.z}]\n q: [1, 0, 0, 0]\n')
            self._shell_text(text+'nsh> ')
            self.visual_wait = None
        if self.intervals[111]>0 and self.now>=self.next_sync_request:
            stream+=self.pack(self.encoder.timesync_encode(0,int((self.now+self.boot_offset)*1e9)))
            self.next_sync_request=self.now+self.intervals[111]/1e6
        for queue in (self.pending,self.shell):
            while queue and len(stream)+len(queue[0])+len(extra_rx) <= self.runtime.max_rx_bytes:
                stream += queue.popleft()
        ns = int((100+self.now)*1e9)
        if self.injected:
            with_pose &= self.scenario != 'pose_loss'
            pad &= self.scenario != 'gamepad_loss'
            if self.scenario == 'link_loss':
                stream = b''
        result = self.run_owner_turn(self.now,rx=stream+extra_rx,now_ros_ns=ns,
            pose=(ns,'map',[self.x,-self.y,-self.z],[0,0,0,1]) if with_pose else None,
            pad_generation='SYNTHETIC-PAD',pad_present=pad,
            pad_event=bool(home) or int(self.now*100) % 50 == 0,home_events=home)
        self.inspect_output()
        self.state_rows.append(dict(at=self.now,phase=self.runtime.core.phase,
            fusion=self.runtime.coordinator.fusion.phase,x=self.x,y=self.y,z=self.z,armed=self.armed,
            locked=self.locked,parameters=dict(self.params),failure=self.runtime.failure))
        return result

    def prepare(self):
        for _ in range(10):
            self.turn()
        self.turn(home=(True,))
        self.turn(home=(False,))
        for _ in range(2500):
            self.turn()
            if self.runtime.failure or self.runtime.coordinator.fusion.phase == 'HANDED_OFF':
                break
        return self.runtime.status()
