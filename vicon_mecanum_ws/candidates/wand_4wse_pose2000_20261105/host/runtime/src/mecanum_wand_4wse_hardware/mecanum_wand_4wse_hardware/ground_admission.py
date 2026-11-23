"""New-image contract. Historical 4WSC observations are not 4WSE approval."""
from .authorization import require,endpoint
import math
import time
from mecanum_wand_4wse_control.tracking_config import TrackingSafetyConfig
from .feedback_timeout import COMBINED_FEEDBACK_TIMEOUT_S
from mecanum_wand_4wse_control.pose_policy import POSE_TIMEOUT_S

CONSENT='REVFLOW_4WSE_STAGE6_NEW_IMAGE_SITE_READY'
KIND='4WSE_STAGE6_LONG600_OPERATOR_UNQUALIFIED_TRIAL_V1'

def common(p):
    require(isinstance(p,dict) and p.get('identity')=='4WSE' and p.get('purpose')=='sign'
        and p.get('trial_kind')==KIND and type(p.get('stage')) is int and p['stage']==6,
        'independent 4WSE stage6 Sign profile required')
    require(p.get('mapping_scale')==1. and p.get('max_trial_s')==600.
        and p.get('physical_qualified') is False and p.get('long_session_qualified') is False,
        '1:1 unqualified 600s software contract required')
    require(p.get('operator_input_mode')=='SPACE_TOGGLE_STAGE6_LONG600_V1'
        and p.get('arena_revision')=='tracking_only_stage6_long600_4wse_v8'
        and p['safety'].get('goal_boundary_policy')=='tracking',
        'single-click direct tracking without a fixed fence required')
    require(p.get('baseline_observation')=={} and p.get('operator_duration_trial')=={}
        and p.get('trial_review')=={} and p.get('long_duration_review')=={},'old motion approval or manufactured qualification forbidden')
    unknown=('stop_envelope_m','battery_current_a','driver_temperature_c','motor_temperature_c')
    require(set(p.get('measurements',{}))==set(unknown) and all(p['measurements'][k] is None for k in unknown),
        'keep unmeasured physical limits explicitly unknown')
    c=TrackingSafetyConfig(**p['safety']);h=p['hardware']
    require((c.max_speed_mps,c.max_accel_mps2,c.max_yaw_rate_radps,c.max_yaw_accel_radps2)==(.7,2.,2.,12.)
        and c.max_session_s==600. and c.pose_timeout_s==POSE_TIMEOUT_S and c.feedback_timeout_s==COMBINED_FEEDBACK_TIMEOUT_S
        and c.command_timeout_s==.10 and c.max_control_gap_s==.10,'existing limits/timeouts required')
    require(h.get('required_motion_tag')=='4WSE' and h.get('diagnostic_payload_bytes')==240
        and h.get('output_authority_version')==8 and h.get('coupled_shape_revision')==3 and h.get('wire_timeout_ms')==600
        and h.get('pwm_cap_permille')==800 and h.get('motor_target_rpm_cap')==400.
        and h.get('wheel_slew_guard_cps_s')==26000.,'4WSE wheel/transport envelope required')
    return c

def verify_runtime_profile(p,purpose,now=None):
    common(p);endpoint(p)
    from .deployment_record import verify_migration
    verify_migration(p)
    r=p.get('commissioning_review',{});issued=r.get('issued_utc_s');now=time.time() if now is None else now
    require(purpose=='sign' and r.get('kind')==KIND and r.get('human_authorized_stage')==6
        and type(issued) in (int,float) and math.isfinite(issued) and 0<=now-issued<=600,
        'fresh exact stage6 site declaration required')
    require(r.get('operator_declared_site_ready_now') is True
        and r.get('controlled_new_image_test') is True and r.get('measurements_still_unknown') is True
        and r.get('no_physical_qualification_claim') is True,'explicit controlled-test declaration required')
    duration=r.get('max_active_s')
    require(type(duration) in (int,float) and math.isfinite(duration) and 1<=duration<=600,
        'explicit bounded 1..600s session required')
    require(p.get('confirmations',{}).get('wiring_secured') is True
        and p['confirmations'].get('power_cut_ready') is True,'current wiring and power-cut condition required')
    return True
