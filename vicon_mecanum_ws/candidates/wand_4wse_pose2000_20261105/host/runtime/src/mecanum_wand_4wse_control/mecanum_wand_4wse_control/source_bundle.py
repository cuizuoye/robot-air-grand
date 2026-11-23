"""Immutable selected Sign+car evidence; no ROS/network and no restamping."""
from collections import OrderedDict
from dataclasses import dataclass
import hashlib
import math
from .cached_packet import decode_packet
from .rotation_source import canonical
from .core import Pose
from .pose_policy import POSE_TIMEOUT_S

@dataclass(frozen=True)
class Bundle:
    packet: object
    car: Pose
    car_stamp: float
    car_received: float
    roll: float
    pitch: float
    digest: str
    @property
    def order(self):return self.packet.stamp,self.car_stamp

def make_bundle(packet,car):
    value=dict(schema=1,packet=packet.payload(),car=car)
    return dict(value,digest=hashlib.sha256(canonical(value).encode()).hexdigest())

def decode_bundle(value,now,ros):
    if not isinstance(value,dict) or set(value)!={'schema','packet','car','digest'} or type(value['schema']) is not int or value['schema']!=1:
        raise ValueError('SOURCE_BUNDLE_SCHEMA')
    base={k:value[k] for k in ('schema','packet','car')}
    if hashlib.sha256(canonical(base).encode()).hexdigest()!=value['digest']:raise ValueError('SOURCE_BUNDLE_DIGEST')
    p=decode_packet(value['packet'],now,ros);car=value['car']
    if not isinstance(car,dict) or set(car)!={'pose','stamp','received','roll','pitch'}:raise ValueError('SOURCE_CAR_SCHEMA')
    if not isinstance(car['pose'],dict) or set(car['pose'])!={'x','y','z','yaw'}:raise ValueError('SOURCE_CAR_POSE')
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in (*car['pose'].values(),car['stamp'],car['received'],car['roll'],car['pitch'])):
        raise ValueError('SOURCE_CAR_NONFINITE')
    if not 0<=now-car['received']<=POSE_TIMEOUT_S or not -.005<=ros-car['stamp']<=POSE_TIMEOUT_S or car['stamp']<=0:
        raise ValueError('SOURCE_CAR_AGE')
    return Bundle(p,Pose(**car['pose']),car['stamp'],car['received'],car['roll'],car['pitch'],value['digest'])

class BundleCache:
    def __init__(self):self.items=OrderedDict();self.orders={};self.rejected=0
    def add(self,value,now,ros):
        b=decode_bundle(value,now,ros)
        old=self.orders.get(b.order)
        if old is not None and old!=b.digest:raise ValueError('SOURCE_SAME_ORDER_CONFLICT')
        if b.digest in self.items:return self.items[b.digest],False
        self.items[b.digest]=b;self.orders[b.order]=b.digest
        for k,v in list(self.items.items()):
            if now-min(v.packet.received,v.car_received)>POSE_TIMEOUT_S or ros-min(v.packet.stamp,v.car_stamp)>POSE_TIMEOUT_S:
                del self.items[k];self.orders.pop(v.order,None)
        while len(self.items)>64:
            _,v=self.items.popitem(last=False);self.orders.pop(v.order,None)
        return b,True
    def exact(self,key,now,ros):
        b=self.items.get(key)
        if b is None:return None
        b.packet.pose(now,ros)
        if not 0<=now-b.car_received<=POSE_TIMEOUT_S or not -.005<=ros-b.car_stamp<=POSE_TIMEOUT_S:raise ValueError('SELECTED_CAR_STALE')
        return b
