"""Offline target bounds/replay only. No ROS graph, serial or hardware tests."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import ast
import math
import sys

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1]
BRIDGE = TOOLS.parents[1]/'ros2_ws/src/uav_coord_bridge'
sys.path[:0] = [str(TOOLS), str(BRIDGE)]
from follow_target_generator import FollowTargetGenerator, Limits, Mode, TargetPose
from uav_coord_bridge.target_mapping import make_alignment, map_target_to_controller
from replay_follow_targets import replay


def generator(**kwargs):
    config = dict(initial_xy=(0., 0.), ground_z_map_m=.3, height_m=2., yaw_map_rad=.2,
        limits=Limits(-2, 2, -2, 2, .4, .3), wand_source='wand01', car_source='car01')
    config.update(kwargs)
    return FollowTargetGenerator(**config)


def sample(t=1_000_000_000, **kwargs):
    row = dict(source='car01', frame_id='map', stamp_ns=t, received_ns=t,
               position=(1., 0., 50.), quaternion_xyzw=(0., 0., 0., 1.), tracked=True, occluded=False)
    row.update(kwargs)
    return TargetPose(**row)


def test_measured_velocity_reduces_reference_lag_and_loss_still_brakes():
    old,new=generator(),generator()
    for g in (old,new):g.set_mode(Mode.CAR_FOLLOW_ABOVE)
    errors=[[],[]]
    for i in range(351):
        now=10**9+i*20_000_000;car=sample(now,position=(.2*i*.02,0.,.1))
        for j,g in enumerate((old,new)):
            out=g.step(now_ns=now,now_source_ns=now,target=car,
                       target_velocity_xy=(.2,0.) if j else (0.,0.))
            if i>200:errors[j].append(abs(out['position_map_m'][0]-car.position[0]))
            assert math.hypot(*out['acceleration_xy_mps2'])<=g.limits.acceleration_mps2+1e-9
    assert sum(errors[1])/len(errors[1]) < .02
    assert sum(errors[0])/len(errors[0]) > .09
    previous=math.hypot(*new.velocity)
    for i in range(30):
        now+=20_000_000
        out=new.step(now_ns=now,now_source_ns=now,target=replace(car,tracked=False),
                     target_velocity_xy=(100.,100.))
        speed=math.hypot(*out['velocity_xy_mps'])
        assert speed<=previous+1e-9 and out['reason']
        previous=speed


@pytest.mark.parametrize('mode', list(Mode))
def test_four_modes_bound_speed_acceleration_and_height(mode):
    g = generator()
    g.set_mode(mode, hold_xy=(.5, .5), offset_xy=(.2, .1))
    previous = None
    for i in range(1001):
        now = 10**9+i*20_000_000
        s = sample(now, source='wand01' if mode is Mode.WAND_FOLLOW else 'car01',
                   position=(math.sin(i*.01), .5*math.cos(i*.02), 100.+i))
        out = g.step(now_ns=now, now_source_ns=now, target=s)
        assert out['position_map_m'][2] == 2.3
        assert g.limits.contains(out['position_map_m'][:2])
        assert math.hypot(*out['velocity_xy_mps']) <= .4+1e-9
        assert math.hypot(*out['acceleration_xy_mps2']) <= .3+1e-9
        if previous:
            assert math.dist(out['position_map_m'][:2], previous['position_map_m'][:2]) <= .4*.02+1e-9
        previous = out
        assert out['flight_authorized'] is False and out['actual_sent_to_fc'] == 0


@pytest.mark.parametrize('yaw,expected', [(0., (1., .25)), (math.pi/2, (-.25, 1.)),
                                        (math.pi, (-1., -.25)), (-math.pi/2, (.25, -1.))])
def test_car_body_offset_rotates_by_car_not_uav_yaw(yaw, expected):
    g = generator(yaw_map_rad=-.8)
    g.set_mode(Mode.CAR_FOLLOW_OFFSET, offset_xy=(1., .25))
    s = sample(position=(0., 0., 100.), quaternion_xyzw=(0., 0., math.sin(yaw/2), math.cos(yaw/2)))
    out = g.step(now_ns=10**9, now_source_ns=10**9, target=s)
    assert out['desired_xy_m'] == pytest.approx(expected)
    assert out['yaw_map_rad'] == -.8 and out['position_map_m'][2] == 2.3


@pytest.mark.parametrize('change,reason', [
    ({'source': 'MircoAir'}, 'wrong_identity'), ({'source': 'wand01'}, 'wrong_identity'),
    ({'frame_id': 'NED'}, 'frame_mismatch'), ({'tracked': False}, 'untracked'),
    ({'occluded': True}, 'occluded'), ({'stamp_ns': 1}, 'stale'),
    ({'received_ns': 1}, 'stale'), ({'stamp_ns': 10**9+1}, 'future'),
    ({'received_ns': 10**9+1}, 'future'), ({'position': (math.nan, 0., 0.)}, 'finite'),
    ({'quaternion_xyzw': (0., 0., 0., 0.)}, 'quaternion'),
    ({'position': (3., 0., 0.)}, 'outside_area')])
def test_invalid_target_brakes_not_authorizes(change, reason):
    g = generator(); g.set_mode(Mode.CAR_FOLLOW_ABOVE)
    out = g.step(now_ns=10**9, now_source_ns=10**9, target=sample(**change))
    assert reason in out['reason'] and out['status'] == 'TARGET_BRAKING'
    assert out['flight_authorized'] is False


def test_target_loss_brakes_and_latches_without_land_command_or_resume():
    g = generator(); g.set_mode(Mode.CAR_FOLLOW_ABOVE)
    for i in range(50):
        t = 10**9+i*20_000_000
        g.step(now_ns=t, now_source_ns=t, target=sample(t))
    velocities = []
    for i in range(51):
        t += 20_000_000
        out = g.step(now_ns=t, now_source_ns=t)
        velocities.append(math.hypot(*out['velocity_xy_mps']))
    assert velocities[-1] == pytest.approx(0.) and g.fault.startswith('target_loss_timeout')
    assert out['actual_sent_to_fc'] == 0 and out['flight_authorized'] is False
    with pytest.raises(ValueError, match='latched'):
        g.step(now_ns=t+20_000_000, now_source_ns=t+20_000_000, target=sample(t+20_000_000))


def test_mode_and_offset_switch_preserve_motion_state():
    g = generator()
    for i in range(500):
        if i % 25 == 0:
            g.set_mode(list(Mode)[(i//25)%4], hold_xy=(-1., -1.), offset_xy=(-.2, .2))
        t = 10**9+i*20_000_000
        out = g.step(now_ns=t, now_source_ns=t, target=sample(t,
            source='wand01' if g.mode is Mode.WAND_FOLLOW else 'car01'))
        assert math.hypot(*out['acceleration_xy_mps2']) <= .3+1e-9
        assert g.limits.contains(out['position_map_m'][:2])


@pytest.mark.parametrize('xy', [(1.999, 1.999), (-1.999, -1.999), (1.999, -1.999)])
def test_geofence_reference_stopping_envelope(xy):
    g = generator(initial_xy=xy); g.set_mode(Mode.CAR_FOLLOW_ABOVE)
    for i in range(500):
        t = 10**9+i*20_000_000
        out = g.step(now_ns=t, now_source_ns=t, target=sample(t, position=(*xy, 0.)))
        assert g.limits.contains(out['position_map_m'][:2])


@pytest.mark.parametrize('clock', ['backwards', 'gap', 'source_backwards'])
def test_clock_gap_and_regression_are_not_hidden(clock):
    g = generator(); g.step(now_ns=10**9, now_source_ns=10**9)
    t = 999_000_000 if clock == 'backwards' else 1_200_000_000 if clock == 'gap' else 1_020_000_000
    with pytest.raises(ValueError, match='clock'):
        g.step(now_ns=t, now_source_ns=999_000_000 if clock == 'source_backwards' else t)


def test_source_duplicates_not_retimestamped_and_regression_rejected():
    g = generator(); g.set_mode(Mode.CAR_FOLLOW_ABOVE)
    g.step(now_ns=10**9, now_source_ns=10**9, target=sample())
    out = g.step(now_ns=1_020_000_000, now_source_ns=1_020_000_000, target=sample())
    assert out['source']['stamp_ns'] == 10**9 and out['source']['source_age_ns'] == 20_000_000
    out = g.step(now_ns=1_040_000_000, now_source_ns=1_040_000_000,
                 target=sample(999_000_000))
    assert 'regression' in out['reason']


def test_unknown_visibility_defaults_to_invalid():
    g = generator(); g.set_mode(Mode.CAR_FOLLOW_ABOVE)
    s = TargetPose('car01', 'map', 10**9, 10**9, (0., 0., 0.), (0., 0., 0., 1.))
    out = g.step(now_ns=10**9, now_source_ns=10**9, target=s)
    assert out['reason'] == 'target_untracked_or_occluded'


def test_ros_and_monotonic_clocks_are_not_subtracted_from_each_other():
    g = generator(); g.set_mode(Mode.CAR_FOLLOW_ABOVE)
    ros = 1_792_567_855_000_000_000
    s = sample(ros, received_ns=10**9)
    out = g.step(now_ns=10**9, now_source_ns=ros, target=s)
    assert out['source']['source_age_ns'] == out['source']['receive_age_ns'] == 0


def test_same_stamp_with_changed_pose_is_not_new_measurement():
    g = generator(); g.set_mode(Mode.CAR_FOLLOW_ABOVE)
    g.step(now_ns=10**9, now_source_ns=10**9, target=sample())
    out = g.step(now_ns=1_020_000_000, now_source_ns=1_020_000_000,
                 target=sample(position=(.9, 0., 0.)))
    assert out['reason'] == 'conflicting_duplicate_target'


@pytest.mark.parametrize('value', [-1., 0., math.nan, math.inf])
def test_height_must_be_explicit_positive_finite(value):
    with pytest.raises(ValueError): generator(height_m=value)


def test_ground_world_height_mapping_not_negative_two_or_range():
    from uav_coord_bridge.world_reference import freeze_reference
    from uav_coord_bridge.target_mapping import height_target_map
    ref = freeze_reference([0, 0, 0, 1], [0, 0, .3])
    out = generator().step(now_ns=10**9, now_source_ns=10**9)
    assert out['position_map_m'] == pytest.approx(height_target_map(ref, [0, 0], 2.))
    alignment = make_alignment([0, 0, .3], [5, 6, -10], 0, 0)
    assert map_target_to_controller(alignment, out['position_map_m'])[2] == pytest.approx(-12.)
    assert 'range' not in FollowTargetGenerator.__init__.__code__.co_varnames


def config():
    return dict(mode='CAR_FOLLOW_ABOVE', initial_xy=[0., 0.], ground_z_map_m=.3, height_m=2.,
        yaw_map_rad=0., limits=Limits(-2, 2, -2, 2, .4, .3).__dict__,
        wand_source='wand01', car_source='car01')


def records(n=751):
    return [dict(source='car01', frame_id='map', stamp_ns=10**12+i*20_000_000,
        received_source_ns=10**12+i*20_000_000, position=[1., 0., 99.],
        quaternion_xyzw=[0., 0., 0., 1.], tracked=True, occluded=False) for i in range(n)]


@pytest.mark.parametrize('mode', list(Mode))
def test_fifteen_second_replay_has_no_hardware_authority(mode):
    c = config(); c['mode'] = mode.value
    rows = records()
    if mode is Mode.WAND_FOLLOW:
        for r in rows: r['source'] = 'wand01'
    out = replay(rows, c)
    assert out['replay_completed'] and out['reference_bounds_passed']
    assert out['recorded_duration_s'] == pytest.approx(15.)
    assert out['flight_authorized'] is False and out['actual_sent_to_fc'] == 0
    assert out['ros_publishers_created'] == out['devices_opened'] == 0


def test_replay_dropout_not_filled_and_fault_is_bounded():
    rows = records(4); rows[-1]['received_source_ns'] += 2_000_000_000
    result = replay(rows, config())
    assert not result['replay_completed'] and result['fault'].startswith('target_loss_timeout')
    assert result['consumed_rows'] == 3 and not result['flight_authorized']


def test_unordered_replay_not_sorted_away():
    rows = records(); rows[4], rows[5] = rows[5], rows[4]
    with pytest.raises(ValueError, match='regression'): replay(rows, config())


def test_resampling_cannot_hide_a_source_clock_regression():
    rows = records()
    rows[4]['stamp_ns'] = rows[3]['stamp_ns']-1
    with pytest.raises(ValueError, match='source_clock_regression'): replay(rows, config())


def test_replay_includes_final_non_grid_sample():
    rows = records(20)
    rows[-1]['received_source_ns'] += 5_000_000
    out = replay(rows, config())
    assert out['consumed_rows'] == len(rows)
    assert out['outputs'][-1]['source']['stamp_ns'] == rows[-1]['stamp_ns']


@pytest.mark.parametrize('roof_z', [.1, .5, 1., 2., 10.])
def test_moving_roof_or_wand_z_never_changes_world_altitude(roof_z):
    g = generator(); g.set_mode(Mode.CAR_FOLLOW_ABOVE)
    out = g.step(now_ns=10**9, now_source_ns=10**9,
                 target=sample(position=(.1, .1, roof_z)))
    assert out['position_map_m'][2] == 2.3
    assert not out['flight_authorized']


def test_target_loss_does_not_claim_self_localization_or_link_backup():
    g = generator(); g.set_mode(Mode.CAR_FOLLOW_ABOVE)
    out = g.step(now_ns=10**9, now_source_ns=10**9)
    assert 'target' in out['reason'] and out['actual_sent_to_fc'] == 0
    assert not hasattr(g, 'optical_flow') and not hasattr(g, 'range_sensor')
    assert 'self_source' not in out  # No fake UAV pose synthesized from target.


@pytest.mark.parametrize('axis', [0, 1])
def test_reference_brakes_before_area_edge_after_target_reversal(axis):
    g = generator(); g.set_mode(Mode.CAR_FOLLOW_ABOVE)
    for i in range(3000):
        p = [0., 0., 0.]; p[axis] = 1.999 if (i//300)%2 == 0 else -1.999
        t = 10**9+i*20_000_000
        out = g.step(now_ns=t, now_source_ns=t, target=sample(t, position=tuple(p)))
        assert g.limits.contains(out['position_map_m'][:2])
        assert math.hypot(*out['acceleration_xy_mps2']) <= .3+1e-9


@pytest.mark.parametrize('axis', [0, 1])
@pytest.mark.parametrize('direction', [-1, 1])
def test_return_to_fixed_landing_point_brakes_without_reference_overshoot(axis,direction):
    # Recorded return was about 0.66 m with 0.4 m/s, 0.2 m/s² limits.
    # Old proportional pursuit produced ~14 cm reference overshoot before
    # the aircraft crossed the old boundary. Test the reference, not dynamics.
    g=generator(limits=Limits(-2.,2.,-2.,2.,.4,.2))
    goal=[0.,0.];goal[axis]=direction*.66
    g.set_mode(Mode.HOVER,hold_xy=tuple(goal))
    for i in range(600):
        t=10**9+i*20_000_000
        out=g.step(now_ns=t,now_source_ns=t)
        assert -1e-10<=direction*out['position_map_m'][axis]<=.66+1e-8
        assert math.hypot(*out['acceleration_xy_mps2'])<=.2+1e-9
    assert out['position_map_m'][axis]==pytest.approx(goal[axis],abs=.001)
    assert math.hypot(*out['velocity_xy_mps'])<.002






@pytest.mark.parametrize('file', [TOOLS/'follow_target_generator.py',
                                 TOOLS/'replay_follow_targets.py'])
def test_new_modules_have_no_live_control_or_hardware_api(file):
    source = file.read_text(); tree = ast.parse(source)
    banned = {'serial', 'socket', 'subprocess', 'pymavlink', 'mavutil'}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import): assert not banned.intersection(x.name.split('.')[0] for x in node.names)
        if isinstance(node, ast.ImportFrom): assert node.module.split('.')[0] not in banned
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            assert node.func.attr not in {'create_publisher', 'init', 'mavlink_connection', 'set_mode_send', 'param_set_send'}
    assert 'PARAM_SET' not in source and 'PARAM_EXT_SET' not in source
