"""Static, arrival-paired attitude consistency observation; no I/O/control.

Yaw is deliberately NOT aligned or validated here. Quaternion sign changes
are equivalent rotations; a large source step during this stationary check
is latched for review, never repaired by an automatic 180-degree rotation.
"""
from collections import deque
import math


def unit(q):
    if len(q)!=4 or not all(math.isfinite(x) for x in q):
        raise ValueError('invalid_quaternion')
    norm=math.sqrt(sum(x*x for x in q))
    if not .95<=norm<=1.05:raise ValueError('invalid_quaternion_norm')
    return tuple(x/norm for x in q)


def gravity(q):
    w,x,y,z=unit(q)
    return (2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y))


def tilt_difference(source,fc):
    a,b=gravity(source),gravity(fc)
    return math.degrees(math.acos(max(-1.,min(1.,sum(x*y for x,y in zip(a,b))))))


class LiveAttitudeCheck:
    def __init__(self):
        self.pairs=deque(maxlen=40)
        self.fault=None
        self.last_boot=None
        self.duplicates=0

    def observe(self,source,fc,now,boot_ms):
        if self.fault:return
        try:
            if not math.isfinite(now) or now<0 or not isinstance(boot_ms,int) or boot_ms<0:
                raise ValueError('invalid_attitude_clock')
            if self.last_boot is not None and boot_ms<self.last_boot:
                raise ValueError('attitude_boot_clock_regressed')
            source,fc=unit(source),unit(fc)
            if boot_ms==self.last_boot:
                self.duplicates+=1
                return  # A repeat is neither a new pair nor new freshness.
            if self.pairs:
                dt=now-self.pairs[-1]['at']
                if dt<=0:raise ValueError('attitude_arrival_clock_not_increasing')
                step=math.degrees(2*math.acos(min(1.,abs(sum(
                    a*b for a,b in zip(source,self.pairs[-1]['source']))))))
                if dt<=.5 and step>15:raise ValueError('source_attitude_step_during_static_check')
            difference=tilt_difference(source,fc)
            if difference>5:raise ValueError('source_fc_gravity_disagreement_over_5_deg')
            self.last_boot=boot_ms
            self.pairs.append({'at':now,'source':source,'difference_deg':difference})
        except (ValueError,TypeError,OverflowError) as exc:self.fault=str(exc)

    def summary(self,now):
        gaps=[b['at']-a['at'] for a,b in zip(self.pairs,list(self.pairs)[1:])]
        duration=self.pairs[-1]['at']-self.pairs[0]['at'] if len(self.pairs)>1 else 0
        rate=(len(self.pairs)-1)/duration if duration>0 else None
        age=now-self.pairs[-1]['at'] if self.pairs else None
        valid=bool(not self.fault and len(self.pairs)>=20 and rate is not None and rate>=3
                   and gaps and max(gaps)<=.5 and age is not None and 0<=age<=.5)
        return {'observed':valid,'pairs':len(self.pairs),'pair_rate_hz':rate,
                'duplicate_fc_samples_ignored':self.duplicates,
                'maximum_pair_gap_s':max(gaps) if gaps else None,'latest_pair_age_s':age,
                'maximum_tilt_difference_deg':max((p['difference_deg'] for p in self.pairs),default=None),
                'fault':self.fault,'yaw_alignment_verified':False,
                'dynamic_extrinsics_verified':False,'imu_origin_verified':False,
                'scope':'Fresh stationary gravity-direction comparison at message arrival; not exposure synchronization'}
