"""One-writer MAVLink boundary for the new mission core.

Offline by default. A supplied real port STILL rejects all mission traffic
without a separate reviewed authorizer, and flight traffic is forbidden while
the FC USB exists. This file never opens devices, creates authorizations,
clears holds, modifies parameters, unlocks outputs or starts a mission.
"""
import math

from pymavlink.dialects.v20 import common as m
from mavlink_readonly_probe import FC_USB, readonly_message
from disarmed_offboard_policy import POSITION_YAW_MASK
import compact_vision


class MissionWireTransport:
    def __init__(self, *, port=None, authorizer=None, usb_present=None):
        self.port = port
        self.authorizer = authorizer
        self.usb_present = usb_present if usb_present is not None else FC_USB.exists
        self.encoder = m.MAVLink(None,srcSystem=250,srcComponent=220)
        self.core = None
        self.records = []
        self.physical_tx_bytes = 0
        self.uncertain_delivery = False

    def attach(self, core):
        if self.core is not None:
            raise ValueError('One immutable mission owner per transport')
        self.core = core

    def candidate_allowed(self, msg):
        c = self.core
        if c is None:
            return False
        if msg.get_type() == 'VISION_POSITION_ESTIMATE':
            return not c.stop_latched and compact_vision.pose_allowed(msg)
        if msg.get_type() == 'SET_POSITION_TARGET_LOCAL_NED':
            if c.origin is None or c.phase not in ('WARMUP','WAIT_OFFBOARD','WAIT_ARM','ASCEND','HOVER','DESCEND'):
                return False
            if c.cooperative is not None and c.phase in ('ASCEND','HOVER'):
                return c.cooperative.candidate_allowed(msg,c)
            xy_allowed = (abs(msg.x-c.origin.position_ned[0])<=1e-6
                          and abs(msg.y-c.origin.position_ned[1])<=1e-6)
            if c.motion is not None or c.phase == 'DESCEND':
                # An enlarged radius alone would permit arbitrary targets.
                # Accept only this owner's current, time-parameterized target.
                xy_allowed = all(abs(a-b)<=1e-6 for a,b in zip(
                    (msg.x,msg.y,msg.z), c.expected_motion_target()))
            return bool((msg.target_system,msg.target_component,msg.coordinate_frame,msg.type_mask)
                == (1,1,m.MAV_FRAME_LOCAL_NED,POSITION_YAW_MASK)
                and all(math.isfinite(x) for x in (msg.x,msg.y,msg.z,msg.yaw))
                and xy_allowed
                and c.origin.position_ned[2]-c.preview.height-1e-6 <= msg.z <= c.origin.position_ned[2]+1e-6
                and abs(math.atan2(math.sin(msg.yaw-c.origin.yaw_ned),math.cos(msg.yaw-c.origin.yaw_ned)))<=1e-6
                and all(getattr(msg,k)==0 for k in ('vx','vy','vz','afx','afy','afz','yaw_rate')))
        if msg.get_type() != 'COMMAND_LONG' or (msg.target_system,msg.target_component)!=(1,1):
            return False
        params=tuple(getattr(msg,f'param{i}') for i in range(1,8))
        if msg.command==176:
            return bool(c.phase=='WAIT_OFFBOARD' and c.offboard_attempted and msg.confirmation==0
                        and params==(1,6,0,0,0,0,0))
        if msg.command==400:
            return bool(c.phase=='WAIT_ARM' and c.arm_attempted and msg.confirmation==0
                        and params==(1,0,0,0,0,0,0))  # Never force-arm or disarm.
        if msg.command==21:
            return c.phase=='LAND' and msg.confirmation==0 and params==(0,)*7
        if msg.command==185:
            return bool(c.phase=='STOP_LATCHED' and c.stop_latched
                        and msg.confirmation in (0,1,2) and params==(2,0,0,0,0,0,0))
        return False

    def __call__(self,msg):
        is_readonly=readonly_message(msg)
        is_stop_only=bool(msg.get_type()=='COMMAND_LONG' and msg.command==185
                          and self.candidate_allowed(msg))
        if self.uncertain_delivery and not is_stop_only:
            raise OSError('Previous transport write uncertain; no automatic continuation')
        if not is_readonly and not self.candidate_allowed(msg):
            raise ValueError('Message outside the selected one-shot mission envelope')
        if self.port is not None and not is_readonly:
            # No built-in authorizer and no CLI switch to silently opt in.
            if self.usb_present() or self.authorizer is None or self.authorizer(self.core,msg) is not True:
                raise PermissionError('Hardware mission dispatch not separately authorized; no bytes sent')
        wire=msg.pack(self.encoder)
        if self.port is not None:
            try:
                written=self.port.write(wire)
                if written!=len(wire):
                    if type(written) is int and written>0:self.physical_tx_bytes+=written
                    raise OSError('Short write; command reception is uncertain')
            except Exception:
                self.uncertain_delivery=True
                raise
            self.physical_tx_bytes+=len(wire)
        self.encoder.seq=(self.encoder.seq+1)%256
        self.records.append({'type':msg.get_type(),'wire_hex':wire.hex(),
                             'physical_dispatch':self.port is not None})
        return True

    def status(self):
        return {'mode':'offline_wire_capture' if self.port is None else 'guarded_port',
                'recorded_messages':len(self.records),'physical_tx_bytes':self.physical_tx_bytes,
                'physical_dispatch_messages':sum(r['physical_dispatch'] for r in self.records),
                'hardware_authorizer_installed':self.authorizer is not None,
                'uncertain_delivery':self.uncertain_delivery,
                'flight_authorized':None if self.port is not None and self.authorizer is not None else False,
                'authorization_scope':'None granted here; a future real runner must validate its external per-message authorizer'}
