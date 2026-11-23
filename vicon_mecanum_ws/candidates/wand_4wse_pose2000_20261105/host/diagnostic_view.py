"""Small observation-only context; never alters timestamps or control state."""
import math


def display_diagnostics(status, transport, source):
    ages = status.get('pose_source_ages_s', {})
    clean = {key: value for key in ('car', 'wand')
             if type(value := ages.get(key)) in (int, float) and math.isfinite(value)}
    bits = transport.get('native_fault_bits')
    return dict(pose_source_ages_s=clean,
                native_fault_bits=bits if type(bits) is int and bits >= 0 else None,
                input_health=str(status.get('input_health') or '')[:200],
                source_issue=str(source.get('last_error') or source.get('error') or '')[:200])
