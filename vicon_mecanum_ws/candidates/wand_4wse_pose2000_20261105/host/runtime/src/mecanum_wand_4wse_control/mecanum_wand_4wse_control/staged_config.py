"""Explicit angular AND XY tiers; every non-motion guard stays pinned."""
from dataclasses import dataclass,fields,replace
from .servo_config import ServoSafetyConfig
from .stages import STAGES,get_stage
from .feedback_policy import COMBINED_FEEDBACK_TIMEOUT_S
from .pose_policy import POSE_TIMEOUT_S

@dataclass(frozen=True)
class StagedSafetyConfig(ServoSafetyConfig):
    pose_timeout_s: float = POSE_TIMEOUT_S
    def __post_init__(self):
        if self.pose_timeout_s != POSE_TIMEOUT_S:
            raise ValueError('explicit 2000ms source-age policy required')
        if self.max_session_s != 600. or self.feedback_timeout_s != COMBINED_FEEDBACK_TIMEOUT_S:
            raise ValueError('Exact long600 session and diagnostic500ms/wheel500ms feedback policy required')
        choices=[i for i,s in enumerate(STAGES) if (s.speed,s.accel,s.yaw_rate,s.yaw_accel)==(
            self.max_speed_mps,self.max_accel_mps2,self.max_yaw_rate_radps,self.max_yaw_accel_radps2)]
        if choices!=[6]:raise ValueError('Exact 4WSE XY+yaw tier required; no mixed/simulation limits')
        # Use the original strict validator for arena, body, timers, source,
        # tilt, stationary, clock and source-age values. No blanket exemption.
        values={f.name:getattr(self,f.name) for f in fields(ServoSafetyConfig)}
        # Only this named candidate overrides source age. Validate every other
        # field with the unchanged historical parent, not a blanket bypass.
        values.update(max_speed_mps=.20,max_accel_mps2=.08,max_yaw_rate_radps=.10,max_yaw_accel_radps2=.40,max_session_s=120.,feedback_timeout_s=.20,pose_timeout_s=.20)
        ServoSafetyConfig(**values)
    @property
    def stage(self):
        return next(i for i,s in enumerate(STAGES) if (s.speed,s.accel,s.yaw_rate,s.yaw_accel)==(
            self.max_speed_mps,self.max_accel_mps2,self.max_yaw_rate_radps,self.max_yaw_accel_radps2))
    @property
    def limits(self):return get_stage(self.stage)
    def follower_config(self):return replace(super().follower_config(),max_wheel_rpm=self.limits.target_rpm)
