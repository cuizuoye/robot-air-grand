"""Transport-free status classification for the DISARMED monitor only.

This explains missing observations; it never authorizes or starts fusion,
arming, targets or a mission. A diagnostic fault may degrade readiness while
the separate HOME input/stop path remains active.
"""

CHECKS = (
    ('identity_fresh_disarmed', 'WAIT_LINK', '等待同一台未解锁飞控的新鲜身份和心跳'),
    ('actuator_baseline_seen', 'WAIT_BASELINE', '等待完整的飞控执行器状态回读'),
    ('parameters_match_maintenance_baseline', 'WAIT_PARAMETERS', '等待本会话的固定参数读取与核对'),
    ('vicon_fresh', 'WAIT_VICON', '等待实时 MircoAir 位姿'),
    ('calibration_current', 'WAIT_CALIBRATION', '等待当前刚体版本与标定证据一致；不会自动采用新标定'),
    ('attitude_consistency_observed', 'WAIT_ATTITUDE', '等待至少20组新鲜Vicon/飞控倾斜方向一致的静止观测'),
    ('home_input_monitor_running', 'WAIT_INPUT', '等待手柄监听；设备存在不等于已唤醒'),
    ('home_release_lock_verified', 'WAIT_HOME_CHECK', '等待本会话 HOME 按下松开、接受回执及锁定回读'),
    ('live_lockdown_confirmed', 'WAIT_CURRENT_LOCK', '等待当前新鲜的输出锁定状态'),
)


def describe(checks, *, fault=None, running=True):
    # Only explicit True is evidence; missing/None/truthy strings are not.
    observed = {name: checks.get(name) is True for name, _, _ in CHECKS}
    missing = [{'check': name, 'phase': phase, 'hint': hint}
               for name, phase, hint in CHECKS if not observed[name]]
    phase = missing[0]['phase'] if missing else 'STATIC_BENCH_INPUTS_OBSERVED'
    if fault is not None:
        phase = 'DEGRADED'
    if not running:
        phase = 'CLOSED'
    return {'phase': phase, 'checks': observed, 'missing_observations': missing,
            'latched_fault': fault,
            'static_bench_inputs_observed': bool(running and fault is None and not missing),
            'next_step_requires_explicit_separate_bench': True,
            'fusion_started': False, 'flight_authorized': False,
            'hardware_mission_adapter_present': False,
            'flight_blockers': ['no_flight_adapter', 'powered_dynamic_control_not_verified',
                                'independent_emergency_path_not_verified'],
            'scope': 'Read-only disarmed session status, never a flight gate or automatic transition'}
