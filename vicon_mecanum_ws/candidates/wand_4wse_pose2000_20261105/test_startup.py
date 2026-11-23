"""Offline dependency and launch lifecycle injection; no real Vicon or robot IO."""
import copy
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import types
from unittest.mock import Mock, patch

import entry
import receiver_manager as rm


class Child:
    pid = 123456
    def __init__(self, code=None):
        self.code = code
        self.actions = []
    def poll(self):
        return self.code
    def terminate(self):
        self.actions.append('terminate')
    def kill(self):
        self.actions.append('kill')
    def wait(self, timeout):
        self.actions.append(('wait', timeout)); self.code = 0
        return 0


class Startup(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)
        self.info = dict(mode='rehearsal', synthetic=True, hardware_entry=False, port=33033,
            url='http://127.0.0.1:33033/#' + 'a' * 48,
            session=str(entry.HOST / 'sessions' / 'offline_fixture'))
    def tearDown(self):
        self.temp.cleanup()

    def receiver(self, discover, verify=None):
        return rm.PassiveReceiver(self.folder, {}, discover=discover,
            verify=verify or Mock(return_value={'pid': 123456}),
            popen=Mock(side_effect=AssertionError('real process forbidden')))

    def test_01_parent_seal_and_flash_binding_still_valid(self):
        self.assertEqual(entry.verify_parent()['identity'], '4WSE')

    def test_02_existing_receiver_reused_without_spawn(self):
        r = self.receiver(lambda: [123456])
        with patch.object(r, 'spawn', side_effect=AssertionError('must reuse')):
            value = r.start(); r.close()
        self.assertFalse(value['owned']); self.assertFalse(value['fresh_pose_validated'])

    def test_03_multiple_existing_receivers_rejected_not_killed(self):
        r = self.receiver(lambda: [111, 222])
        with self.assertRaisesRegex(ValueError, '多个'):
            r.start()
        r.close(); self.assertIsNone(r.child)

    def test_04_wrong_existing_receiver_provenance_not_replaced(self):
        check = Mock(side_effect=ValueError('wrong provenance'))
        r = self.receiver(lambda: [123456], check)
        with patch.object(r, 'spawn', side_effect=AssertionError('must not replace')):
            with self.assertRaisesRegex(ValueError, 'provenance'):
                r.start()

    def test_05_cold_start_waits_for_owned_pid_before_verifying(self):
        values = iter([[], [], [123456]])
        r = self.receiver(lambda: next(values)); r.sleep = lambda _: None
        with patch.object(r, 'spawn', side_effect=lambda: setattr(r, 'child', Child())):
            value = r.start(); r.close()
        self.assertTrue(value['owned']); self.assertEqual(r.child.actions[0], 'terminate')

    def test_06_cold_receiver_exit_aborts(self):
        r = self.receiver(lambda: [])
        with patch.object(r, 'spawn', side_effect=lambda: setattr(r, 'child', Child(1))):
            with self.assertRaisesRegex(ValueError, '提前退出'):
                r.start()

    def test_07_cold_receiver_timeout_aborts_without_retry(self):
        r = self.receiver(lambda: []); r.clock = Mock(side_effect=[0., 9.])
        with patch.object(r, 'spawn', side_effect=lambda: setattr(r, 'child', Child())) as spawn:
            with self.assertRaisesRegex(ValueError, '超时'):
                r.start()
            self.assertEqual(spawn.call_count, 1)
        r.close()

    def test_08_concurrent_foreign_receiver_is_not_adopted(self):
        values = iter([[], [555]])
        r = self.receiver(lambda: next(values))
        with patch.object(r, 'spawn', side_effect=lambda: setattr(r, 'child', Child())):
            with self.assertRaisesRegex(ValueError, '启动竞争'):
                r.start()
        r.close(); self.assertEqual(r.child.pid, 123456)

    def test_09_owned_receiver_loss_is_failure_not_autorestart(self):
        r = self.receiver(lambda: []); r.child = Child(1)
        with patch.object(r, 'spawn') as spawn:
            with self.assertRaisesRegex(ValueError, '不自动重启'):
                r.check_alive()
            spawn.assert_not_called()

    def test_10_reused_receiver_never_terminated(self):
        r = self.receiver(lambda: [123456]); r.start(); r.close()
        self.assertIsNone(r.child)

    def test_11_only_owned_passive_receiver_kill_fallback(self):
        r = self.receiver(lambda: []); r.child = Child()
        r.child.wait = Mock(side_effect=[subprocess.TimeoutExpired('passive', 3), 0])
        r.close(); self.assertEqual(r.child.actions, ['terminate', 'kill'])

    def test_12_receiver_start_failure_precedes_backend(self):
        with patch.object(entry, 'verify_release'), patch.object(entry, 'route_check'), \
             patch.object(entry, 'HERE', self.folder), patch.object(entry, 'prepare_session', return_value={}), \
             patch.object(entry, 'PassiveReceiver') as receiver, patch.object(entry.subprocess, 'Popen') as spawn:
            receiver.return_value.start.side_effect = ValueError('receiver failure')
            with self.assertRaisesRegex(ValueError, 'receiver failure'):
                entry.launch(False, 60, True, True)
            spawn.assert_not_called(); receiver.return_value.close.assert_called_once()

    def test_13_no_site_authority_means_no_io(self):
        with patch.object(entry, 'verify_release') as verify, \
             patch.object(entry, 'route_check') as route, patch.object(entry, 'PassiveReceiver') as receiver, \
             patch.object(entry, 'ros_environment') as ros:
            with self.assertRaisesRegex(ValueError, 'no ROS/socket'):
                entry.launch(False, 60)
            verify.assert_not_called(); route.assert_not_called(); receiver.assert_not_called(); ros.assert_not_called()

    def test_14_duration_bounds_and_nan_fail_before_io(self):
        for seconds in [0, 601, True, float('nan'), float('inf')]:
            with self.subTest(seconds=seconds), patch.object(entry, 'verify_release') as check:
                with self.assertRaises(ValueError):
                    entry.launch(False, seconds, True)
                check.assert_not_called()

    def test_15_file_only_check_does_not_load_ros_environment_or_socket(self):
        with patch.object(entry, 'verify_release', return_value={}), \
             patch.object(entry, 'ros_environment', side_effect=AssertionError('ROS forbidden')), \
             patch.object(entry, 'PassiveReceiver', side_effect=AssertionError('receiver forbidden')), \
             patch.object(entry, 'launch', side_effect=AssertionError('launch forbidden')):
            with patch('sys.stdout', new=io.StringIO()):
                entry.main(['--check'])

    def test_16_stop_not_blocked_by_file_qualification(self):
        with patch.object(entry, 'verify_release', side_effect=AssertionError('must not block STOP')), \
             patch.object(entry, 'stop_current') as stop:
            entry.main(['--stop']); stop.assert_called_once()

    def test_17_stop_rejects_nonlocal_or_malformed_url_before_network(self):
        for url in ['http://example.com:33033/#'+'a'*48, 'https://127.0.0.1:33033/#'+'a'*48,
                    'http://127.0.0.1:33033/#x', 'http://127.0.0.1:80/#'+'a'*48]:
            with self.subTest(url=url), patch.object(entry, 'build_opener') as opener:
                with self.assertRaises(ValueError):
                    entry.stop_info({'url': url})
                opener.assert_not_called()

    def test_18_route_uses_explicit_tenda_interface_read_only(self):
        result = Mock(stdout=json.dumps([dict(dev='wlx502b73a508c9', prefsrc='192.168.50.2')]))
        with patch.object(entry.subprocess, 'run', return_value=result) as run:
            entry.route_check()
        self.assertEqual(run.call_args.args[0], ['ip','-j','route','get','192.168.50.1','oif','wlx502b73a508c9'])

    def test_19_wrong_route_rejected_not_reconfigured(self):
        with patch.object(entry.subprocess, 'run', return_value=Mock(stdout='[{"dev":"wlp114s0"}]')) as run:
            with self.assertRaisesRegex(ValueError, '不自动更改'):
                entry.route_check()
            self.assertEqual(run.call_count, 1)

    def test_20_ready_metadata_exists_before_publication(self):
        (self.folder / 'evidence').mkdir()
        folder = self.folder / 'evidence'
        def observe(*args, **kwargs):
            if args and str(args[0]).startswith('BROWSER_READY'):
                current = json.loads((self.folder / 'CURRENT.json').read_text())
                self.assertEqual(current['url'], self.info['url'])
                self.assertEqual((self.folder / 'CURRENT.json').stat().st_mode & 0o777, 0o600)
                self.assertTrue((folder / 'READY.json').exists())
        with patch.object(entry, 'HERE', self.folder), patch('builtins.print', side_effect=observe), \
             patch.object(entry.subprocess, 'Popen') as opener:
            result = entry.publish_ready(self.info, True, 60, folder, 444, None, True)
            opener.assert_not_called(); self.assertFalse(result['automatic_motion'])

    def test_21_ready_mismatched_mode_rejected_before_browser_or_metadata(self):
        for changes in [dict(hardware_entry=True), dict(synthetic=False), dict(mode='hardware'),
                        dict(port=33034), dict(session='/tmp/not-the-sealed-host')]:
            info = dict(self.info, **changes)
            with self.subTest(changes=changes), patch.object(entry, 'HERE', self.folder), \
                 patch.object(entry.subprocess, 'Popen') as opener:
                with self.assertRaises(ValueError):
                    entry.publish_ready(info, True, 60, self.folder, 444, None, False)
                opener.assert_not_called(); self.assertFalse((self.folder / 'CURRENT.json').exists())

    def test_22_open_browser_is_url_only_no_input_injection(self):
        with patch.object(entry, 'HERE', self.folder), patch('builtins.print'), \
             patch.object(entry.subprocess, 'Popen') as opener:
            entry.publish_ready(self.info, True, 60, self.folder, 444, None, False)
            self.assertEqual(opener.call_args.args[0], ['xdg-open', self.info['url']])

    def test_23_shared_owner_lock_conflict_does_not_start_receiver(self):
        import fcntl
        with open('/tmp/wand_4wse_browser_operator.lock', 'a') as owner:
            fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with patch.object(entry, 'verify_release'), patch.object(entry, 'PassiveReceiver') as receiver, \
                 patch.object(entry, 'route_check') as route:
                with self.assertRaises(BlockingIOError):
                    entry.launch(False, 60, True, True)
                receiver.assert_not_called(); route.assert_not_called()

    def test_24_parent_and_current_motion_configuration_not_edited(self):
        from ground_entry import profile
        p, _ = profile(entry.HOST, self.folder, True, seconds=60.)
        value = json.loads(p.read_text()); s = value['safety']
        self.assertEqual((s['max_speed_mps'], s['max_accel_mps2']), (.70, 2.))
        self.assertEqual((s['max_yaw_rate_radps'], s['max_yaw_accel_radps2']), (2., 12.))
        from mecanum_wand_4wse_control.tracking_config import TrackingSafetyConfig as SafetyConfig
        config = SafetyConfig(**s)
        self.assertEqual((config.pose_timeout_s, config.feedback_timeout_s, config.max_control_gap_s), (2., .5, .1))
        from mecanum_wand_4wse_control.feedback_policy import DIAGNOSTIC_TIMEOUT_S,WHEEL_TIMEOUT_S
        self.assertEqual((DIAGNOSTIC_TIMEOUT_S,WHEEL_TIMEOUT_S),(.5,.5))
        self.assertEqual(value['hardware']['esp_ip'], '127.0.0.1')
        self.assertNotIn(value['hardware']['command_port'], [46000, 46001])
        self.assertNotIn(value['hardware']['telemetry_port'], [46000, 46001])

    def test_25_actual_spawn_builder_is_passive_domain0_and_verified_before_spawn(self):
        module = types.ModuleType('vicon_receiver_provenance')
        proof = {'installed': '/offline/verified/vicon_client', 'policy': {}}
        module.verify_current = Mock(return_value=proof)
        module.child_environment = Mock(return_value={'ROS_DOMAIN_ID': '83'})
        module.recheck_before_spawn = Mock()
        r = self.receiver(lambda: [])
        def spawn(args, **kw):
            module.recheck_before_spawn.assert_called_once_with(proof)
            self.assertEqual(args[:2], ['/offline/verified/vicon_client', '--ros-args'])
            self.assertEqual(args[2:], [x for k, v in rm.PARAMS.items() for x in ('-p', k + ':=' + v)])
            self.assertEqual(kw['env']['ROS_DOMAIN_ID'], '0')
            self.assertEqual(kw['env']['ROS_AUTOMATIC_DISCOVERY_RANGE'], 'LOCALHOST')
            self.assertFalse('clear' in str(args) or 'enable_motion' in str(args))
            return Child()
        r.popen = spawn
        with patch.dict('sys.modules', {'vicon_receiver_provenance': module}):
            r.spawn(); r.close()

    def test_26_existing_receiver_check_restores_parent_domain_on_failure(self):
        from mecanum_wand_4wse_control import rotation_receiver_guard
        def verify():
            self.assertEqual(os.environ['ROS_DOMAIN_ID'], '0')
            raise ValueError('wrong source')
        with patch.dict(os.environ, {'ROS_DOMAIN_ID': '197'}), \
             patch.object(rotation_receiver_guard, 'verify_existing', side_effect=verify):
            with self.assertRaisesRegex(ValueError, 'wrong source'):
                rm.verify_existing_domain0()
            self.assertEqual(os.environ['ROS_DOMAIN_ID'], '197')


if __name__ == '__main__':
    unittest.main(verbosity=2)
