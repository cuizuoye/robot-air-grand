"""Atomic source references, quarantined bad sensor frames, original watchdogs."""
import json
import math
import time
from rclpy.node import Node
from std_msgs.msg import String
from .optimized_nodes import Supervisor as ParentSupervisor,Gate as ParentGate,COMMAND_QOS,stale_virgin_zero
from .coherent_source import BUNDLE_QOS
from .bound_rotation import diagnostic_value
from mecanum_wand_4wse_control.source_bundle import BundleCache
from mecanum_wand_4wse_control.safety_nodes import decode_frame
from mecanum_wand_4wse_control.rotation_source import canonical
from mecanum_wand_4wse_control.safety_core import ZERO
from mecanum_wand_4wse_control.verified_inputs import VerifiedInputs,VerifiedGateInputs

class Supervisor(ParentSupervisor):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw);self.bundles=BundleCache();self.selected_bundle=None;self.bundle_rejected=0
        self.core.inputs=VerifiedInputs(self.core.c);self.core.inputs.graph_ok=False
        self.core.follower.geometry_proof=lambda:getattr(self.core.inputs,'verified_last_wand',None)
        self.create_subscription(String,'source_bundle',self.on_bundle,BUNDLE_QOS)
    def create_publisher(self,msg_type,topic,qos,*a,**kw):
        if topic!='candidate':return super().create_publisher(msg_type,topic,qos,*a,**kw)
        pub=Node.create_publisher(self,msg_type,topic,COMMAND_QOS,*a,**kw);node=self
        class BoundPublisher:
            def publish(self,msg):
                f=json.loads(msg.data);b=node.selected_bundle
                msg.data=canonical(dict(frame=f,bundle_digest=b.digest if b else None,
                    sign_digest=b.packet.digest if b else None,sign_stamp=b.packet.stamp if b else None,car_stamp=b.car_stamp if b else None));pub.publish(msg)
        return BoundPublisher()
    def on_packet(self,msg):pass  # UI/legacy packet topic is not control evidence.
    def on_pose(self,source,msg):
        if source=='car' and self.core.state not in ('ARMED','ACTIVE'):super().on_pose(source,msg)
    def on_bundle(self,msg):
        now,ros=self.clocks()
        try:
            b,_=self.bundles.add(json.loads(msg.data),now,ros)
            old=self.selected_bundle
            if old and (any(x<y for x,y in zip(b.order,old.order)) or b.order==old.order):return
            self.core.inputs.observe_pose('car',b.car,b.car_stamp,b.car_received,ros,b.roll,b.pitch)
            self.core.inputs.observe_wand_packet(b.packet,now,ros)
            self.selected_bundle=b;self.selected=b.packet
        except (ValueError,TypeError,KeyError,AttributeError):
            self.bundle_rejected+=1  # no source, pose, receipt, lease or timer renewal

class Gate(ParentGate):
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw);self.bundles=BundleCache();self.bundle_rejected=0
        self.core.inputs=VerifiedGateInputs(self.core.c);self.core.inputs.graph_ok=False
        self.create_subscription(String,'source_bundle',self.on_bundle,BUNDLE_QOS)
    def on_packet(self,msg):pass
    def on_pose(self,source,msg):
        if source=='car' and self.core.frame and self.core.frame.state in ('STOPPING','STOP_UNCONFIRMED','PAUSED','FAULT'):
            # A stop does not depend on recovering Sign. Only raw car evidence
            # and true disabled feedback can confirm standstill.
            from mecanum_wand_4wse_control.safety_nodes import SafetyNode
            SafetyNode.on_pose(self,source,msg)
    def on_bundle(self,msg):
        now,ros=self.clocks()
        try:self.bundles.add(json.loads(msg.data),now,ros)
        except (ValueError,TypeError,KeyError,AttributeError):self.bundle_rejected+=1;return
        self.process_pending()
    def on_candidate(self,msg):
        try:
            v=json.loads(msg.data)
            if set(v)!={'frame','bundle_digest','sign_digest','sign_stamp','car_stamp'}:raise ValueError('BOUND_SCHEMA')
            f=decode_frame(canonical(v['frame']));now,ros=self.clocks()
            if stale_virgin_zero(f,self.core,now,ros):self.publish_output(ZERO);return
            if f.sequence<=self.core.last_sequence or (self.pending and f.sequence<=self.pending[-1][1].sequence):return
            if not f.enabled and f.command==ZERO and f.state in ('STANDBY','STOPPING','STOP_UNCONFIRMED','PAUSED','FAULT'):
                self.pending.clear();self.core.receive(f,now,ros);self.publish_output(ZERO);return
            for key in ('bundle_digest','sign_digest'):
                value=v[key]
                if not isinstance(value,str) or len(value)!=64 or any(c not in '0123456789abcdef' for c in value):raise ValueError('BOUND_DIGEST_SCHEMA')
            if any(type(v[k]) not in (int,float) or not math.isfinite(v[k]) or v[k]<=0 for k in ('sign_stamp','car_stamp')):
                raise ValueError('BOUND_STAMP_SCHEMA')
            if len(self.pending)>=4:raise ValueError('BOUND_QUEUE_OVERFLOW')
            self.pending.append((v,f,now));self.process_pending()
        except (ValueError,TypeError,KeyError,AttributeError):
            self.pending.clear();self.core.trip('ROTATION_BOUND_COMMAND_INVALID');self.publish_output(ZERO)
    def process_pending(self):
        if getattr(self,'processing_bound',False):return
        self.processing_bound=True;began=time.monotonic();drained=0
        try:
            while self.pending and drained<4 and time.monotonic()-began<.003:
                v,f,arrived=self.pending[0];now,ros=self.clocks()
                try:
                    b=self.bundles.exact(v['bundle_digest'],now,ros)
                    if b is None:
                        if now-arrived>.040:raise ValueError('SELECTED_BUNDLE_MISSING')
                        break
                    p=b.packet
                    if (v['sign_digest'],v['sign_stamp'],v['car_stamp'])!=(p.digest,p.stamp,b.car_stamp):raise ValueError('BOUND_EVIDENCE_MISMATCH')
                    if f.issued_steady<max(p.received,b.car_received) or f.issued_ros<max(p.stamp,b.car_stamp)-.005:raise ValueError('SOURCE_AFTER_COMMAND')
                    self.core.inputs.observe_pose('car',b.car,b.car_stamp,b.car_received,ros,b.roll,b.pitch)
                    sign=p.pose(now,ros);self.core.inputs.observe_wand_packet(p,now,ros)
                    for name,pose,stamp in (('car',b.car,b.car_stamp),('wand',sign,p.stamp)):
                        stored=self.core.inputs.poses.get(name)
                        if not stored or stored['pose']!=pose or stored['source_ros']!=min(stamp,ros):raise ValueError('SELECTED_POSE_NOT_APPLIED')
                    accepted=self.core.receive(f,now,ros);self.pending.popleft();drained+=1
                    timer=self.core.clock.previous
                    try:self.core.tick(now,ros)
                    finally:self.core.clock.previous=timer
                    if not accepted or self.core.fault or self.core.hard_fault:
                        self.pending.clear();self.publish_output(ZERO);break
                    if f.state=='ACTIVE':self.publish_output(self.core.output);self.commit_count+=1
                except (ValueError,TypeError,KeyError) as error:
                    # Revoke FIRST. Untrusted diagnostics must never prevent
                    # zero output, even if formatting/logging itself fails.
                    self.pending.clear();self.core.trip('ROTATION_SELECTED_SOURCE_INVALID');self.publish_output(ZERO)
                    if self.selection_failure is None:
                        detail=dict(reason=str(error),sequence=f.sequence,wait_s=now-arrived,issued_age_s=now-f.issued_steady,
                            sign_source_age_s=ros-v['sign_stamp'],car_source_age_s=ros-v['car_stamp'],bundle_digest=v.get('bundle_digest'))
                        self.selection_failure={k:diagnostic_value(x) for k,x in detail.items()}
                        try:self.get_logger().warn('COHERENT_SOURCE_REJECTED '+canonical(self.selection_failure))
                        except Exception:pass
                    break
        finally:self.bound_drained+=drained;self.processing_bound=False
