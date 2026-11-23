"""Injected-clock source-age and independent-protection regression."""
from dataclasses import replace
import json
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / 'host'))
import bootstrap
bootstrap.install()
from geometry import synthetic_packet
from mecanum_wand_4wse_control.tracking_config import TrackingSafetyConfig
from mecanum_wand_4wse_control.rotation_source import Packet, decode_packet
from mecanum_wand_4wse_control.cached_packet import decode_packet as cached_decode
from mecanum_wand_4wse_control.source_bundle import BundleCache, make_bundle, decode_bundle
from mecanum_wand_4wse_control.safety_core import Inputs, Lease
from mecanum_wand_4wse_control.core import Pose
from mecanum_wand_4wse_hardware.commissioning import AuthoritySigner, TransportPolicy
from mecanum_wand_4wse_control.safety_core import ZERO


def config():
    return TrackingSafetyConfig(**json.loads((HERE / 'host/profiles/stage6.LONG600_NOT_APPROVED.json').read_text())['safety'])


def car(age=0.):
    return dict(pose=dict(x=0., y=0., z=config().ground_z_m, yaw=0.),
                stamp=1000.-age, received=100.-age, roll=0., pitch=0.)


class PosePolicy(unittest.TestCase):
    def test_01_config_follower_and_other_limits(self):
        c = config()
        self.assertEqual(c.pose_timeout_s, 2.)
        self.assertEqual(c.follower_config().input_timeout_s, 2.)
        self.assertEqual((c.feedback_timeout_s, c.command_timeout_s, c.max_control_gap_s), (.5, .1, .1))
        self.assertEqual((c.max_speed_mps, c.max_accel_mps2, c.max_yaw_rate_radps, c.max_yaw_accel_radps2), (.7, 2., 2., 12.))

    def test_02_exact_named_policy_not_unbounded(self):
        for value in (.2, 1., 2.001, float('inf'), float('nan')):
            with self.subTest(value=value), self.assertRaises(ValueError):
                replace(config(), pose_timeout_s=value)

    def test_03_packet_age_within2000(self):
        for age in (.201, .5, 1.5, 1.999, 2.):
            packet = synthetic_packet(1., 1., 0., 100.-age, 1000.-age)
            with self.subTest(age=age):
                self.assertIsNotNone(packet.pose(100., 1000.))
                self.assertIsNotNone(Packet(packet.samples).pose(100., 1000.))
                self.assertIsNotNone(decode_packet(packet.payload(), 100., 1000.))
                self.assertIsNotNone(cached_decode(packet.payload(), 100., 1000.))

    def test_04_packet_expires_strictly_after2000(self):
        packet = synthetic_packet(1., 1., 0., 97.999, 997.999)
        for fn in (lambda: packet.pose(100., 1000.), lambda: cached_decode(packet.payload(), 100., 1000.)):
            with self.assertRaisesRegex(ValueError, 'AGE_OR_MIXED_FRAME'):
                fn()

    def test_05_source_age_cannot_be_refreshed_by_recent_receipt(self):
        packet = synthetic_packet(1., 1., 0., 100., 997.9)
        with self.assertRaises(ValueError):
            packet.pose(100., 1000.)

    def test_06_old_receipt_cannot_be_refreshed_by_source_stamp(self):
        packet = synthetic_packet(1., 1., 0., 97.9, 1000.)
        with self.assertRaises(ValueError):
            packet.pose(100., 1000.)

    def test_07_future_stamp_tolerance_unchanged(self):
        packet = synthetic_packet(1., 1., 0., 100., 1000.006)
        with self.assertRaises(ValueError):
            packet.pose(100., 1000.)

    def test_08_mixed_frames_still_rejected(self):
        p = synthetic_packet(1., 1., 0., 100., 1000.)
        samples = list(p.samples)
        samples[-1] = replace(samples[-1], stamp=999.999)
        with self.assertRaises(ValueError):
            Packet(tuple(samples)).pose(100., 1000.)

    def test_09_exact_bundle_cache_does_not_evict_at200(self):
        p = synthetic_packet(1., 1., 0., 100., 1000.)
        wire = make_bundle(p, car())
        cache = BundleCache()
        cache.add(wire, 100., 1000.)
        for age in (.201, .5, 1.5, 1.999, 2.):
            self.assertIsNotNone(cache.exact(wire['digest'], 100.+age, 1000.+age))
        with self.assertRaises(ValueError):
            cache.exact(wire['digest'], 102.001, 1002.001)

    def test_10_car_and_sign_independent_age_checks(self):
        p = synthetic_packet(1., 1., 0., 100., 1000.)
        self.assertIsNotNone(decode_bundle(make_bundle(p, car(1.5)), 100., 1000.))
        with self.assertRaisesRegex(ValueError, 'SOURCE_CAR_AGE'):
            decode_bundle(make_bundle(p, car(2.001)), 100., 1000.)

    def test_11_original_pose_receipt_cannot_be_renewed_by_duplicate(self):
        inputs = Inputs(config())
        p = Pose(0., 0., config().ground_z_m, 0.)
        self.assertTrue(inputs.observe_pose('car', p, 1000., 100., 1000.))
        self.assertFalse(inputs.observe_pose('car', p, 1000., 101.5, 1001.5))
        self.assertEqual(inputs.poses['car']['received'], 100.)
        self.assertTrue(inputs.pose_fresh('car', 102., 1002.))
        self.assertFalse(inputs.pose_fresh('car', 102.001, 1002.001))

    def test_12_control_and_signed_authority_timeout_unchanged(self):
        key = '12'*32
        signer, transport = AuthoritySigner(key, 6), TransportPolicy(key, 6)
        transport.receive(signer.encode(ZERO, 100., 1000.), 100., 1000.)
        transport.receive(signer.encode(ZERO, 100.01, 1000.01, epoch='test', state='ARMED', permit=True), 100.01, 1000.01)
        transport.active(100.111)
        self.assertEqual(transport.fault, 'SIGNED_AUTHORITY_TIMEOUT')

    def test_13_replay_today_pose_timeout_records(self):
        report = json.loads((HERE / 'evidence/RECENT_FAILURES.json').read_text())
        count = 0
        for row in report['failed_sessions']:
            if not row['addressed_by_pose2000']:
                continue
            count += 1
            for age_ms in row['pose_source_ages_ms'].values():
                self.assertGreaterEqual(age_ms, 0.)
                self.assertLess(age_ms, 2000.)
                packet = synthetic_packet(1., 1., 0., 100.-age_ms/1000., 1000.-age_ms/1000.)
                self.assertIsNotNone(packet.pose(100., 1000.))
        self.assertEqual(count, 4)

    def test_14_geometry_checks_not_relaxed_by_longer_age(self):
        p = synthetic_packet(1., 1., 0., 99., 999.)
        samples = list(p.samples)
        original = samples[-1]
        samples[-1] = replace(original, position=(original.position[0]+.02, *original.position[1:]))
        with self.assertRaisesRegex(ValueError, 'GEOMETRY_OR_ATTITUDE_FLIP'):
            Packet(tuple(samples)).pose(100., 1000.)


if __name__ == '__main__':
    unittest.main(verbosity=2)
