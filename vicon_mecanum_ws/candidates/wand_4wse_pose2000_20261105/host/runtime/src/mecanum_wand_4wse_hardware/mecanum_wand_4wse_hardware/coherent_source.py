"""One low-depth evidence stream; UI poses remain separate and read-only."""
from dataclasses import asdict
import math
import time
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from std_msgs.msg import String
from .source_relay import SourceRelay as Parent
from mecanum_wand_4wse_control.core import Pose
from mecanum_wand_4wse_control.pose_policy import POSE_TIMEOUT_S
from mecanum_wand_4wse_control.source_bundle import make_bundle,decode_bundle
from mecanum_wand_4wse_control.cached_packet import decode_packet
from mecanum_wand_4wse_control.rotation_source import canonical
from mecanum_wand_4wse_control.safety_nodes import pose_message

BUNDLE_QOS=QoSProfile(depth=4,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)

class SourceRelay(Parent):
    def create_timer(self,period,callback,*a,**kw):
        # Align immutable evidence publication with the100Hz controller.
        # Raw Vicon capture/age and independent control watchdogs do not change.
        if period==.005 and getattr(callback,'__name__','')=='tick':period=.01
        return super().create_timer(period,callback,*a,**kw)
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        from mecanum_wand_4wse_control.car_frame_quarantine import CarFrameQuarantine
        self.car_frames=CarFrameQuarantine(self.runtime.config)
        self.bundle_pub=self.create_publisher(String,'source_bundle',BUNDLE_QOS)
        self.last_bundle_order=None
        self.last_good_packet=None
    def tick(self):
        now=time.monotonic();ros=self.get_clock().now().nanoseconds*1e-9
        conflicts=[];errors={};value=None
        try:value=self.reader.poll()
        except (ValueError,TypeError,KeyError,AttributeError):self.rejected+=1
        now=time.monotonic();ros=self.get_clock().now().nanoseconds*1e-9
        if value:
            conflicts=value['conflicts'];errors=dict(value['rejected'])
            if not conflicts:
                car=self.car_frames.offer(value.get('car'),now,ros);p=None;q=None
                # Keep car-only evidence alive for STOP confirmation while
                # Sign is occluded. These poses cannot bypass the bound gate.
                if car:
                    raw=Pose(**car['pose'])
                    q=Pose(raw.x,raw.y,raw.z,math.atan2(math.sin(raw.yaw+self.runtime.yaw_offset),math.cos(raw.yaw+self.runtime.yaw_offset)))
                    if -.005<=ros-car['stamp']<=POSE_TIMEOUT_S and (self.last_car is None or car['stamp']>self.last_car):
                        self.car_pub.publish(pose_message(q,self.header(car['stamp']),car['roll'],car['pitch'],'map'));self.last_car=car['stamp']
                if value.get('packet'):
                    try:
                        candidate=decode_packet(value['packet'],now,ros)
                        old=self.last_good_packet
                        if old is None or candidate.stamp>old.stamp:self.last_good_packet=candidate
                        elif candidate.stamp==old.stamp and candidate.digest!=old.digest:self.rejected+=1
                    except (ValueError,TypeError,KeyError,AttributeError):self.rejected+=1
                # Pair fresh car evidence with a still-fresh last-good Sign
                # during a missing/bad sample. Original Sign stamp/receipt
                # remain unchanged; after2000ms it cannot form any bundle.
                p=self.last_good_packet
                try:
                    if p:p.pose(now,ros)
                except (ValueError,TypeError,KeyError,AttributeError):p=None
                try:
                    if p and q:
                        c=dict(pose=asdict(q),stamp=car['stamp'],received=car['received'],roll=car['roll'],pitch=car['pitch'])
                        wire=make_bundle(p,c);b=decode_bundle(wire,now,ros)
                        if self.last_bundle_order is None or (all(x>=y for x,y in zip(b.order,self.last_bundle_order)) and b.order!=self.last_bundle_order):
                            self.bundle_pub.publish(String(data=canonical(wire)));self.last_bundle_order=b.order
                        if self.last_packet is None or p.stamp>self.last_packet:
                            self.packet_pub.publish(String(data=canonical(p.payload())))
                            self.wand_pub.publish(pose_message(p.pose(now,ros),self.header(p.stamp),world='map'));self.last_packet=p.stamp
                except (ValueError,TypeError,KeyError,AttributeError):self.rejected+=1
        # An empty/late polling result is NOT a graph conflict or a new good
        # pose. Last-good original timestamps expire normally at 2000ms.
        errors['ipc_or_geometry']=self.rejected
        errors['car_single_flip_quarantined']=self.car_frames.quarantined
        errors['car_unsafe_forwarded']=self.car_frames.unsafe_forwarded
        status=(tuple(conflicts),tuple(sorted(errors.items())))
        if status!=self.last_status or now-self.status_at>=.05:
            # No IPC sample neither invents a conflict nor clears the last
            # independently observed conflict. Only a fresh audit may do so.
            if value is not None:self.mode_pub.publish(String(data='conflict' if conflicts else 'wand'))
            self.info.publish(String(data=canonical(dict(issued_steady=now,issued_ros=ros,rejected=errors,conflicts=conflicts,passive_process=True,
                car_frame_quality=self.car_frames.snapshot()))))
            self.last_status,self.status_at=status,now
