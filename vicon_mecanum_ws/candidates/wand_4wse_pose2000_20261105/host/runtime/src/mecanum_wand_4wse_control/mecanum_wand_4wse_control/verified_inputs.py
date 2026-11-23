"""Relax hand-only proxy limits ONLY for original five-marker evidence.

No speed/accel/age/geometry/car guard changes. Ambiguous ~180deg discontinuity
still stops: yaw-only samples cannot determine the direction of a half turn.
"""
import math
from .rotation_servo import RotationInputs
from .stop_feedback import StopFeedbackInputs

class EvidenceMixin:
    def observe_wand_packet(self,p,now,ros):
        pose=p.pose(now,ros)  # independently rechecks original geometry and age
        self._verified_wand=(pose,p.stamp,p.received)
        try:
            accepted=self.observe_pose('wand',pose,p.stamp,p.received,ros)
            if accepted:self.verified_last_wand=(pose,p.received)
            return accepted
        finally:self._verified_wand=None
    def pose_step_issue(self,source,pose,old):
        if source=='wand' and getattr(self,'_verified_wand',None):return ''
        return super().pose_step_issue(source,pose,old)
    def wand_angle_step_issue(self,old,pose,stamp):
        if getattr(self,'_verified_wand',None):
            delta=abs(math.atan2(math.sin(pose.yaw-old['pose'].yaw),math.cos(pose.yaw-old['pose'].yaw)))
            return 'WAND_ANGLE_AMBIGUOUS' if delta>=math.pi-.02 else ''
        return super().wand_angle_step_issue(old,pose,stamp)

class VerifiedInputs(EvidenceMixin,StopFeedbackInputs):pass
class VerifiedGateInputs(EvidenceMixin,RotationInputs):pass
