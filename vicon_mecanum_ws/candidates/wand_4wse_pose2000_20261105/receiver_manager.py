"""Passive Vicon dependency; own only a receiver created by this launcher."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

PROVENANCE = Path('/home/czy/vicon_receiver_ws/drone_sim/tools/gamepad_rc')
PARAMS = dict(hostname='10.10.10.2', buffer_size='200', namespace='vicon',
    world_frame='map', vicon_frame='vicon', map_xyz='[0.0,0.0,0.0]',
    map_rpy='[0.0,0.0,0.0]', map_rpy_in_degrees='false', stream_mode='ServerPush',
    enable_marker_ray_data='false', enable_debug_data='false', enable_device_data='false',
    publish_latency='true', capture_frame_samples='false')


def receiver_pids():
    from mecanum_wand_4wse_control.rotation_receiver_guard import receiver_pids as domain_pids
    return domain_pids(domain=0)


def verify_existing_domain0():
    from mecanum_wand_4wse_control.rotation_receiver_guard import verify_existing
    previous = os.environ.get('ROS_DOMAIN_ID')
    try:
        os.environ['ROS_DOMAIN_ID'] = '0'
        return verify_existing()
    finally:
        if previous is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = previous


class PassiveReceiver:
    def __init__(self, folder, env, *, discover=receiver_pids,
                 verify=verify_existing_domain0, popen=subprocess.Popen,
                 clock=time.monotonic, sleep=time.sleep):
        self.folder, self.env = Path(folder), dict(env)
        self.discover, self.verify, self.popen = discover, verify, popen
        self.clock, self.sleep = clock, sleep
        self.child = self.log = None
        self.proof = None

    def spawn(self):
        sys.path.insert(0, str(PROVENANCE))
        from vicon_receiver_provenance import verify_current, child_environment, recheck_before_spawn
        proof = verify_current()
        env = child_environment(self.env, proof['policy'])
        env.update(ROS_DOMAIN_ID='0', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
            ROS_LOG_DIR=str(self.folder / 'receiver_ros_logs'))
        args = [proof['installed'], '--ros-args']
        for key, value in PARAMS.items():
            args += ['-p', key + ':=' + value]
        recheck_before_spawn(proof)
        self.log = (self.folder / 'receiver.log').open('xb')
        self.child = self.popen(args, env=env, stdout=self.log,
            stderr=subprocess.STDOUT, start_new_session=True)

    def start(self):
        found = self.discover()
        if len(found) > 1:
            raise ValueError('域0内多个Vicon接收器；不停止或替换外部进程')
        if found:
            self.proof = self.verify()  # Reject wrong provenance/domain/coordinates.
        else:
            self.spawn()
            end = self.clock() + 8.
            while self.clock() < end:
                if self.child.poll() is not None:
                    raise ValueError('只读Vicon接收器提前退出，见receiver.log')
                found = self.discover()
                if found:
                    if found != [self.child.pid]:
                        raise ValueError('Vicon接收器启动竞争；不接管外部进程')
                    self.proof = self.verify()
                    break
                self.sleep(.025)
            if self.proof is None:
                raise ValueError('只读Vicon接收器启动超时')
        result = dict(self.proof, owned=self.child is not None,
            device_control=False, network_configuration_changed=False,
            fresh_pose_validated=False)  # The unchanged backend checks fresh frames.
        (self.folder / 'RECEIVER.json').write_text(json.dumps(result, indent=2) + '\n')
        return result

    def check_alive(self):
        if self.child is not None and self.child.poll() is not None:
            raise ValueError('本启动器的只读Vicon接收器退出；停止会话，不自动重启')

    def close(self):
        if self.child is not None and self.child.poll() is None:
            self.child.terminate()
            try:
                self.child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                # Only the passive reader we created; never a hardware owner.
                self.child.kill()
                self.child.wait(timeout=2)
        if self.log is not None:
            self.log.close()
