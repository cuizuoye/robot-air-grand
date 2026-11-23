"""Lazy, explicitly constructed Linux gamepad and ROS latest-pose bindings.

Importing this module opens no input/serial/network resources. A live host
constructs these only after an explicit USB-ground physical confirmation.
"""
import fcntl
import os

from gamepad_estop_monitor import EVENT,HOME,find_pad
from trs_runtime_monitor import gamepad_identity,require_gamepad_identity


class LinuxGamepad:
    def __init__(self, *, controls=False):
        self.controls=controls
        self.axes={};self.buttons=set()
        self.device=find_pad();self.path=self.device['path']
        self.generation=gamepad_identity(self.path)
        self.fd=os.open(self.path,os.O_RDONLY|os.O_NONBLOCK)
        self.pending=b'';self.initial=None
        try:
            require_gamepad_identity(self.fd,self.path,self.generation)
            bits=fcntl.ioctl(self.fd,0x80000000|(96<<16)|(ord('E')<<8)|0x18,bytes(96))
            if bits[HOME//8]&(1<<(HOME%8)):self.initial=True
            if self.controls:
                from gamepad_rc import KEY_NAME,CODE_OF,evdev_live_axis
                self.buttons={name for code,name in KEY_NAME.items() if bits[code//8]&(1<<(code%8))}
                self.axes={code:evdev_live_axis(self.fd,code) for code in CODE_OF.values()}
                if any(type(v) is not int for v in self.axes.values()):
                    raise OSError('All four live axes required; never assume neutral')
        except Exception:
            os.close(self.fd);raise

    def poll(self):
        require_gamepad_identity(self.fd,self.path,self.generation)
        events=[];active=False;edges=[]
        if self.initial is not None:events.append(self.initial);self.initial=None;active=True
        try:
            chunk=os.read(self.fd,EVENT.size*64)
            if not chunk:raise OSError('Gamepad disconnected')
            self.pending+=chunk
        except BlockingIOError:pass
        while len(self.pending)>=EVENT.size:
            _,_,kind,code,value=EVENT.unpack(self.pending[:EVENT.size]);self.pending=self.pending[EVENT.size:]
            if kind==0 and code==3:raise OSError('SYN_DROPPED: input state lost')
            active|=kind in (1,3)
            if self.controls:
                from gamepad_rc import KEY_NAME,CODE_OF
                if kind==3 and code in CODE_OF.values():self.axes[code]=value
                if kind==1 and code in KEY_NAME:
                    if value not in (0,1,2):raise OSError('Invalid key event')
                    name=KEY_NAME[code]
                    if value==1:edges.append(name)
                    if value:self.buttons.add(name)
                    else:self.buttons.discard(name)
            if kind==1 and code==HOME:
                if value not in (0,1,2):raise OSError('Invalid HOME event')
                events.append(value!=0)
                if len(events)>8:raise OSError('HOME burst exceeds bounded turn')
        result=dict(pad_generation=self.generation,pad_present=True,pad_event=active,home_events=tuple(events))
        if self.controls:
            result['pilot_state']=dict(axes=dict(self.axes),buttons=tuple(self.buttons),edges=tuple(edges))
        return result

    def close(self):os.close(self.fd)


class RosLatestPose:
    topic='/vicon/MircoAir/MircoAir'
    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from geometry_msgs.msg import PoseStamped
        self.rclpy=rclpy;self.context=rclpy.context.Context();self.latest=None
        self.node=None;self.executor=None
        rclpy.init(args=[],context=self.context)
        try:
            self.node=Node('mircoair_ground_preparation',context=self.context)
            self.executor=SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.subscription=self.node.create_subscription(PoseStamped,self.topic,self.receive,
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE))
        except Exception:
            if self.executor is not None:self.executor.shutdown(timeout_sec=0.)
            if self.node is not None:self.node.destroy_node()
            self.context.shutdown();raise

    def receive(self,msg):
        p,q=msg.pose.position,msg.pose.orientation
        self.latest=(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,msg.header.frame_id,
                     [p.x,p.y,p.z],[q.x,q.y,q.z,q.w])

    def poll(self):
        for _ in range(4):self.executor.spin_once(timeout_sec=0.)
        sample,self.latest=self.latest,None
        return sample

    def now_ns(self):return self.node.get_clock().now().nanoseconds

    def close(self):
        self.executor.shutdown(timeout_sec=0.)
        self.node.destroy_node();self.context.shutdown()
