"""Offline display-context validation; no source restamping or ROS access."""
import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent / 'host'))
from diagnostic_view import display_diagnostics


class DisplayDiagnostics(unittest.TestCase):
    def test_real_trace_age_and_zero_fault_preserved(self):
        value = display_diagnostics(dict(pose_source_ages_s={'car': .188592, 'wand': .200504}),
                                    dict(native_fault_bits=0), {})
        self.assertEqual(value['pose_source_ages_s']['wand'], .200504)
        self.assertEqual(value['native_fault_bits'], 0)

    def test_no_mutation_or_restamping(self):
        status = dict(pose_source_ages_s={'car': .235, 'wand': .214}, input_health='WAND_POSE_TIMEOUT')
        before = copy.deepcopy(status)
        display_diagnostics(status, {}, {})
        self.assertEqual(status, before)

    def test_unknown_missing_not_reported_as_zero(self):
        self.assertIsNone(display_diagnostics({}, {}, {})['native_fault_bits'])
        self.assertEqual(display_diagnostics({}, {}, {})['pose_source_ages_s'], {})

    def test_nonfinite_and_boolean_not_sent(self):
        result = display_diagnostics(dict(pose_source_ages_s={'car': float('nan'), 'wand': True}),
                                     dict(native_fault_bits=True), {})
        self.assertEqual(result['pose_source_ages_s'], {})
        self.assertIsNone(result['native_fault_bits'])

    def test_only_bounded_diagnostic_fields(self):
        result = display_diagnostics(dict(input_health='X'*1000), {}, dict(error='Y'*1000))
        self.assertEqual(len(result['input_health']), 200)
        self.assertEqual(len(result['source_issue']), 200)


if __name__ == '__main__':
    unittest.main()
