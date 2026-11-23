"""Independent startup V2. Reuse the sealed 4WSE runtime without editing it."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import selectors
import signal
import subprocess
import sys
import tempfile
import time
from urllib.parse import urlsplit
from urllib.request import build_opener, ProxyHandler, Request

HERE = Path(__file__).resolve().parent
PARENT = Path('/home/czy/vicon_mecanum_ws/candidates/wand_4wse_pose2000_20261105')
HOST = PARENT / 'host'
PARENT_MANIFEST_SHA = '8f294b1e59def6b6700e8c809228a5d639a3e5a5a35ff931e938f0ff8659aa04'
sys.path.insert(0, str(HOST))
import bootstrap
from release_binding import verify_host_snapshot, require, sha
from receiver_manager import PassiveReceiver


def atomic_json(path, value):
    fd, name = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(name, path)  # mkstemp is0600 before any publication.
    finally:
        if os.path.exists(name):
            os.unlink(name)


def verify_parent():
    require(sha(HOST / 'HOST_MANIFEST.json') == PARENT_MANIFEST_SHA, 'sealed parent manifest changed')
    return verify_host_snapshot()


def verify_release():
    verify_parent()
    m = json.loads((HERE / 'RELEASE.json').read_text())
    require(m.get('revision') == '4WSE_POSE2000_V1' and m.get('offline_passed') is True
        and m.get('physical_validated') is False and m.get('parent_manifest_sha256') == PARENT_MANIFEST_SHA,
        'offline-qualified launcher release required')
    for group in ('sources', 'evidence'):
        require(bool(m.get(group)), 'missing release hashes')
        for relative, digest in m[group].items():
            p = (HERE / relative).resolve()
            require(p.is_relative_to(HERE) and sha(p) == digest, 'launcher release changed: ' + relative)
    return m


def endpoint(info):
    u = urlsplit(info['url'])
    require(u.scheme == 'http' and u.hostname == '127.0.0.1'
        and u.port is not None and 1024 <= u.port <= 65535
        and re.fullmatch('[a-f0-9]{48}', u.fragment), 'invalid local stop endpoint')
    return u


def stop_info(info):
    u = endpoint(info)
    origin = f'http://127.0.0.1:{u.port}'
    request = Request(origin + '/operator-stop',
        data=json.dumps(dict(type='stop', token=u.fragment, reason='LOCAL_STOP')).encode(),
        headers={'Content-Type': 'application/json', 'Origin': origin}, method='POST')
    with build_opener(ProxyHandler({})).open(request, timeout=3) as response:
        require(response.status == 202, 'local stop was not accepted')


def stop_current():
    stop_info(json.loads((HERE / 'CURRENT.json').read_text()))
    print('已请求停止；请核对STOPPED_CONFIRMED及实车，必要时断电。', flush=True)


def ros_environment():
    p = subprocess.run(['bash', '-c', 'source /opt/ros/jazzy/setup.bash && env -0'],
        capture_output=True, check=True, timeout=10)
    return dict(item.decode().split('=', 1) for item in p.stdout.split(b'\0') if item)


def route_check():
    p = subprocess.run(['ip', '-j', 'route', 'get', '192.168.50.1', 'oif', 'wlx502b73a508c9'],
        capture_output=True, text=True, check=True, timeout=5)
    rows = json.loads(p.stdout)
    require(len(rows) == 1 and rows[0].get('dev') == 'wlx502b73a508c9'
        and rows[0].get('prefsrc', '').startswith('192.168.50.'),
        'Tenda未就绪；不自动更改网络配置')


def prepare_session(software, seconds, site_ready, folder):
    if not software:
        require(site_ready is True, 'current site confirmation required; no ROS/socket opened')
    from ground_entry import profile
    p, consent = profile(HOST, folder, software,
        site_ready=site_ready, operator_trial=not software, seconds=seconds)
    env = ros_environment()
    env.update(WAND_4WSE_MODE='rehearsal' if software else 'hardware',
        WAND_4WSE_PROFILE=str(p), WAND_PROFILE_APPROVAL=sha(p), WAND_HARDWARE_CONSENT=consent,
        WAND_4WSE_PURPOSE='sign', WAND_4WSE_MAX_ACTIVE_S=str(seconds), PYTHONDONTWRITEBYTECODE='1')
    return env


def publish_ready(info, software, seconds, folder, pid, receiver, no_browser):
    u = endpoint(info)
    session = Path(info['session']).resolve()
    require(session.is_relative_to(HOST / 'sessions') and info.get('hardware_entry') is (not software)
        and info.get('synthetic') is software and info.get('port') == u.port
        and info.get('mode') == ('rehearsal' if software else 'hardware'), 'backend readiness identity mismatch')
    current = dict(info, seconds=seconds, launcher_pid=os.getpid(), node_pid=pid,
        startup_folder=str(folder), receiver_owned=receiver is not None and receiver.child is not None,
        automatic_motion=False, created_utc_s=time.time())
    atomic_json(HERE / 'CURRENT.json', current)
    atomic_json(folder / 'READY.json', current)
    # Publication is completed BEFORE observers see BROWSER_READY.
    print('BROWSER_READY ' + json.dumps(current), flush=True)
    if not no_browser:
        subprocess.Popen(['xdg-open', info['url']], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return current


def launch(software, seconds, site_ready=False, no_browser=False):
    require(type(software) is bool and type(seconds) in (int, float) and 1 <= seconds <= 600,
        'explicit mode and bounded1..600s duration required')
    if not software:
        require(site_ready is True, 'current site confirmation required; no ROS/socket opened')
        verify_release()
    else:
        verify_parent()
    owner = open('/tmp/wand_4wse_browser_operator.lock', 'a')
    process = receiver = None
    info = None
    try:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if not software:
            route_check()
        (HERE / 'evidence').mkdir(exist_ok=True)
        folder = Path(tempfile.mkdtemp(prefix='startup_', dir=HERE / 'evidence'))
        env = prepare_session(software, seconds, site_ready, folder)
        if not software:
            receiver = PassiveReceiver(folder, env)
            receiver.start()  # Before the unchanged hardware backend is created.
        args = ['node', str(HOST / 'server.cjs'), '--rehearsal' if software else '--hardware',
            '--seconds', str(seconds)]
        if software:
            args += ['--synthetic']
        with (folder / 'backend.log').open('x') as log:
            process = subprocess.Popen(args, env=env, stdout=subprocess.PIPE,
                stderr=log, bufsize=0)
            selector = selectors.DefaultSelector()
            selector.register(process.stdout, selectors.EVENT_READ)
            pending = b''
            deadline = time.monotonic() + 45.
            try:
                eof = False
                while not eof:
                    if receiver is not None:
                        receiver.check_alive()
                    if info is None and time.monotonic() > deadline:
                        raise ValueError('启动自检超时；没有打开页面，不自动重试')
                    for key, _ in selector.select(.1):
                        data = os.read(key.fileobj.fileno(), 65536)
                        if not data:
                            eof = True
                            break
                        pending += data
                        require(len(pending) < 1048576, 'backend output is unbounded')
                        while b'\n' in pending:
                            line, pending = pending.split(b'\n', 1)
                            line = line.decode()
                            log.write(line + '\n'); log.flush()
                            if line.startswith('BROWSER_READY '):
                                require(info is None, 'duplicate backend readiness')
                                info = publish_ready(json.loads(line[len('BROWSER_READY '):]),
                                    software, seconds, folder, process.pid, receiver, no_browser)
                            elif line.startswith('BROWSER_RESULT '):
                                print(line, flush=True)
                    if process.poll() is not None and not eof:
                        continue  # Drain the final pipe contents before exit.
                code = process.wait(timeout=5)
                require(code == 0 and info is not None,
                    '启动未完成或后端退出失败；检查 ' + str(folder / 'backend.log'))
            finally:
                selector.close()
    finally:
        if process is not None and process.poll() is None:
            if info is not None:
                try:
                    stop_info(info)
                except (OSError, ValueError):
                    pass
            process.terminate()
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                # Do not SIGKILL a hardware owner during its stop confirmation.
                print('后端未退出，请立即确认停车，必要时断开动力电源。', file=sys.stderr)
        if receiver is not None:
            receiver.close()
        owner.close()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    m = p.add_mutually_exclusive_group()
    m.add_argument('--launch', action='store_true')
    m.add_argument('--rehearsal', action='store_true')
    m.add_argument('--stop', action='store_true')
    m.add_argument('--check', action='store_true')
    p.add_argument('--site-ready', action='store_true')
    p.add_argument('--seconds', type=float, default=60.)
    p.add_argument('--no-browser', action='store_true')
    a = p.parse_args(argv)
    if a.stop:
        stop_current(); return
    if a.launch or a.rehearsal:
        launch(a.rehearsal, a.seconds, a.site_ready, a.no_browser)
    else:
        verify_release()
        print(json.dumps(dict(passed=True, revision='4WSE_POSE2000_V1', offline_only=True,
            ROS_initialized=False, hardware_socket_opened=False, physical_validated=False,
            motion_parameters_unchanged=True, pose_timeout_ms=2000,
            feedback_timeout_ms=500, control_timeout_ms=100)))


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        main()
    except (ValueError, OSError, KeyError, subprocess.SubprocessError) as error:
        raise SystemExit('BLOCKED: ' + str(error))
    except KeyboardInterrupt:
        raise SystemExit('已请求停止；请确认小车停稳。')
