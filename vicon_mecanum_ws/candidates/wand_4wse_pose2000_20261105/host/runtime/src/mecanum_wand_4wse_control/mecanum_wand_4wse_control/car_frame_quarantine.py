"""Quarantine ONE implausible upside-down car frame, never repair its pose.

No look-ahead or motion smoothing. Healthy samples pass immediately, unchanged.
Only an abrupt >=150deg orientation change within50ms, after a fresh healthy
frame, with continuous position/height is eligible. A second distinct unsafe
frame, ordinary tilt, lift, invalid geometry or missing baseline goes through
the unchanged downstream guards. Replays cannot renew either timestamp.
"""
import copy
import math


def quaternion(roll,pitch,yaw):
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy)


class CarFrameQuarantine:
    def __init__(self,config):
        self.c=config
        self.last_good=None;self.last_seen=None;self.consecutive_unsafe=0
        self.quarantined=self.unsafe_forwarded=self.duplicate_or_old=self.stale=0
        self.first_quarantine=self.latest_decision=None

    def offer(self,car,now,ros):
        if car is None:return None
        try:
            p=car['pose'];stamp=car['stamp'];received=car['received']
            values=(p['x'],p['y'],p['z'],p['yaw'],car['roll'],car['pitch'],stamp,received,now,ros)
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in values):
                return car  # Never manufacture a plausible pose from malformed input.
            if not (-self.c.future_tolerance_s<=ros-stamp<=self.c.pose_timeout_s and 0<=now-received<=self.c.pose_timeout_s):
                self.stale+=1;return None
            if self.last_seen is not None and stamp<=self.last_seen:
                self.duplicate_or_old+=1;return None
            self.last_seen=stamp
            lift=abs(p['z']-self.c.ground_z_m)>self.c.lift_tolerance_m
            tilted=max(abs(car['roll']),abs(car['pitch']))>self.c.tilt_limit_rad
            if not lift and not tilted:
                self.last_good=copy.deepcopy(car);self.consecutive_unsafe=0
                self.latest_decision=dict(kind='ACCEPT_UNCHANGED',source_ros=stamp)
                return car
            self.consecutive_unsafe+=1
            old=self.last_good;eligible=False;step=None;dt=None
            if old is not None and not lift and tilted and self.consecutive_unsafe==1:
                dt=stamp-old['stamp']
                old_fresh=(0<=now-old['received']<=self.c.pose_timeout_s
                    and -self.c.future_tolerance_s<=ros-old['stamp']<=self.c.pose_timeout_s)
                if old_fresh and 0<dt<=.050:
                    a=quaternion(old['roll'],old['pitch'],old['pose']['yaw'])
                    b=quaternion(car['roll'],car['pitch'],p['yaw'])
                    dot=min(1.,abs(sum(x*y for x,y in zip(a,b))))
                    step=2*math.acos(dot)
                    xy=math.hypot(p['x']-old['pose']['x'],p['y']-old['pose']['y'])
                    eligible=(step>=math.radians(150.) and xy<=self.c.max_speed_mps*dt+.005
                        and abs(p['z']-old['pose']['z'])<=.010)
            if eligible:
                self.quarantined+=1
                detail=dict(kind='QUARANTINE_SINGLE_ORIENTATION_FLIP',source_ros=stamp,
                    original_received_steady=received,orientation_step_deg=math.degrees(step),source_step_s=dt,
                    raw=copy.deepcopy(car),last_good_source_ros=old['stamp'])
                self.latest_decision=detail
                if self.first_quarantine is None:self.first_quarantine=copy.deepcopy(detail)
                return None  # No output, cached substitute, new stamp, health renewal or permission.
            self.unsafe_forwarded+=1
            self.latest_decision=dict(kind='PASS_TO_EXISTING_GUARDS',source_ros=stamp,consecutive_unsafe=self.consecutive_unsafe)
            return car
        except (KeyError,TypeError,ValueError):
            return car  # Original decoding/guards remain responsible for malformed data.

    def snapshot(self):
        return copy.deepcopy(dict(quarantined=self.quarantined,unsafe_forwarded=self.unsafe_forwarded,
            duplicate_or_old=self.duplicate_or_old,stale=self.stale,consecutive_unsafe=self.consecutive_unsafe,
            first_quarantine=self.first_quarantine,latest_decision=self.latest_decision,
            source_timestamps_unchanged=True,filter_adds_lookahead=False))
