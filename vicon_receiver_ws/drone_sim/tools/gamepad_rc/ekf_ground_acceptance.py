"""Phase 2 OFFLINE evidence review. No transport, writer or authorization API.

Hashes bind supplied artifacts, not their provenance. Even a successful review
of real records is NOT a live lock proof or permission to operate hardware.
"""
import hashlib
import math
from pathlib import Path
import re
import struct

import ekf_static_profile as profile

SOURCE_PARAMS = ('src/modules/ekf2/ekf2_params.c',
                 'src/modules/commander/commander_params.c')
NO_GRANT = dict(physical_authorization=False, flight_authorized=False,
                control_ready=False, live_enabled=False, devices_opened=0)


def result(**fields):
    return dict(schema=1, scope='plan-b-phase2-offline-review', **fields, **NO_GRANT)


def dependency_issues(values):
    """Pinned EKF2::VerifyParams rules; report, NEVER apply its corrections."""
    issues = []
    height = values.get('EKF2_HGT_REF')
    if height == 0 and values.get('EKF2_BARO_CTRL') == 0:
        issues.append('height_reference_would_enable_baro')
    if height == 1 and not values.get('EKF2_GPS_CTRL', 0) & 2:
        issues.append('height_reference_would_enable_gps')
    if height == 2 and values.get('EKF2_RNG_CTRL') == 0:
        issues.append('height_reference_would_enable_range')
    if height == 3 and not values.get('EKF2_EV_CTRL', 0) & 2:
        issues.append('height_reference_would_enable_ev_vertical')
    gps = values.get('EKF2_GPS_CTRL', 0)
    if gps & 2 and not gps & 1:
        issues.append('gps_vertical_without_horizontal_would_be_cleared')
    # EKF2.hpp SensorFusionMask: deprecated GPS/flow/bias/EV/drag/GPSyaw bits.
    if values.get('EKF2_AID_MASK', 0) & 0b110111111:
        issues.append('deprecated_aid_mask_would_migrate_and_commit')
    return issues


def inventory(document, source_root):
    root = Path(source_root)
    declarations = {}
    hashes = {}
    for relative in SOURCE_PARAMS:
        file = (root / relative).resolve(strict=True)
        if (file.parts[1] in ('dev','proc','sys') or not file.is_file()
                or file.stat().st_size > 8*1024*1024):
            raise ValueError('Source must be a bounded regular local file')
        data = file.read_bytes()
        hashes[relative] = hashlib.sha256(data).hexdigest()
        text = data.decode()
        pattern = r'/\*\*(?P<doc>(?:(?!\*/).)*)\*/\s*PARAM_DEFINE_(?P<kind>INT32|FLOAT)\((?P<name>\w+)\s*,[^;]*;'
        for match in re.finditer(pattern, text, re.S):
            name = match['name']
            if name not in profile.PARAMETER_TYPES:
                continue
            if name in declarations:
                raise ValueError('Duplicate source declaration: ' + name)
            declarations[name] = dict(source=relative,
                line=text.count('\n', 0, match.start('kind')) + 1,
                type=6 if match['kind'] == 'INT32' else 9,
                reboot_required='@reboot_required true' in match['doc'])
    rows = []
    for name, kind in profile.PARAMETER_TYPES.items():
        source = declarations.get(name)
        if source is None or source['type'] != kind:
            raise ValueError('Pinned source missing or type differs: ' + name)
        parameters = document.get('parameters', {})
        spec = parameters.get(name, {}) if isinstance(parameters,dict) else {}
        spec = spec if isinstance(spec,dict) else {}
        rows.append(dict(name=name, **source, type_name='INT32' if kind == 6 else 'REAL32',
            value=spec.get('value'), approval=spec.get('status', 'missing'),
            evidence=spec.get('evidence', ''), value_missing=spec.get('value') is None))
    values = {r['name']: r['value'] for r in rows if r['value'] is not None}
    # Type validation happens before integer dependency operations.
    errors = profile.profile_errors(document)
    return result(parameters=rows, parameter_count=len(rows), source_sha256=hashes,
        profile_sha256=profile.fingerprint(document), profile_errors=errors,
        dependency_issues=dependency_issues(values) if not errors else ['invalid_profile'],
        approved_count=sum(r['approval'] == 'approved' for r in rows),
        note='Source metadata and approval declarations only; not hardware validation')


def full_snapshot(snapshot):
    """All parameters, indices and typed wire values; never ignore non-EKF drift."""
    if not isinstance(snapshot,dict) or not isinstance(snapshot.get('firmware'),dict):
        raise ValueError('Full snapshot with firmware metadata required')
    count = snapshot.get('expected_count')
    rows = snapshot.get('records')
    if (snapshot.get('system_id'),snapshot.get('component_id')) != (1,1) or snapshot.get('firmware',{}).get('version') != profile.VERSION:
        raise ValueError('Snapshot identity/version metadata mismatch')
    if (snapshot.get('complete') is not True or type(count) is not int or count <= 0
            or snapshot.get('received_count') != count or snapshot.get('missing_indices') != []
            or not isinstance(rows, list) or len(rows) != count):
        raise ValueError('Incomplete full parameter snapshot')
    names, indices = {}, set()
    for row in rows:
        if not isinstance(row,dict):
            raise ValueError('Full snapshot record must be an object')
        name, index, kind = row.get('name'), row.get('index'), row.get('type')
        if (not isinstance(name, str) or not name or name in names or type(index) is not int
                or index in indices or row.get('param_count') != count or type(kind) is not int):
            raise ValueError('Duplicate/invalid full snapshot record')
        raw = bytes.fromhex(row.get('wire_value_hex', ''))
        if kind not in (6,9) or len(raw) != 4:
            raise ValueError('Full snapshot requires consistent typed wire bytes')
        value = profile.typed_value(struct.unpack('<i' if kind == 6 else '<f',raw)[0],kind)
        # Historical snapshot serializer stored INT32 text as 6.0. Accept only
        # exact equality with genuine typed four-byte data, never round/coerce
        # a fractional JSON value or infer the MAV type from its spelling.
        text_value = row.get('value')
        if type(text_value) not in (int,float) or not math.isfinite(text_value) or text_value != value:
            raise ValueError('Full snapshot requires consistent typed wire bytes')
        indices.add(index)
        names[name] = dict(type=kind, value=value)
    if indices != set(range(count)):
        raise ValueError('Missing full snapshot indices')
    return names


def persistence_review(document, before, after, evidence):
    """Review claimed save + reboot + FULL readback. No fixed-wait heuristic.

    Requires a synchronous maintenance save result, NOT a PARAM_VALUE/storage
    ACK. Operator evidence remains untrusted until separately reviewed.
    """
    problems = []
    if profile.profile_errors(document):
        return result(review_complete=False, reasons=profile.profile_errors(document))
    try:
        old, new = full_snapshot(before), full_snapshot(after)
    except (ValueError, TypeError, KeyError, struct.error) as exc:
        return result(review_complete=False, reasons=[str(exc)])
    comparison = profile.compare(document, {n: dict(name=n, **r) for n, r in new.items()})
    changed = sorted(n for n in old.keys() | new.keys() if old.get(n) != new.get(n))
    unexpected = [n for n in changed if n not in profile.PARAMETER_TYPES]
    if old.keys() != new.keys():
        problems.append('parameter_set_changed')
    if unexpected:
        problems.append('unrelated_parameters_changed')
    if not comparison['configuration_match']:
        problems.append('post_reboot_profile_mismatch')
    # Approval is independent of persistence: do not require a circular, already
    # verified persistence claim to review the first save/reboot evidence.
    if (comparison['unapproved'] or comparison['pending_calibration']
            or not comparison['firmware_binding_reviewed']):
        problems.append('configuration_not_approved')
    values = {n: r['value'] for n, r in new.items()}
    problems.extend(dependency_issues(values))
    for key, data in (('before_sha256', before), ('after_sha256', after)):
        if evidence.get(key) != profile.fingerprint(data):
            problems.append(key + '_mismatch')
    identity_before, identity_after = evidence.get('identity_before'), evidence.get('identity_after')
    expected = document.get('firmware', {})
    for identity in (identity_before, identity_after):
        if (not isinstance(identity, dict) or identity.get('uid2') != expected.get('uid2')
                or identity.get('firmware') != profile.VERSION
                or identity.get('artifact_sha256') != expected.get('artifact_sha256')
                or identity.get('target') != profile.TARGET):
            problems.append('identity_or_installed_firmware_not_bound')
    boots = (evidence.get('boot_before'), evidence.get('boot_after'))
    if any(not isinstance(b, str) or not b for b in boots) or boots[0] == boots[1]:
        problems.append('new_boot_not_evidenced')
    keys = ('before_complete_ns', 'save_started_ns', 'save_completed_ns',
            'restart_observed_ns', 'after_complete_ns')
    stamps = [evidence.get(k) for k in keys]
    if (any(type(t) is not int or t < 0 for t in stamps)
            or any(a >= b for a, b in zip(stamps, stamps[1:]))):
        problems.append('maintenance_event_order_or_time_missing')
    if not evidence.get('host_clock_session'):
        problems.append('shared_monotonic_clock_session_missing')
    transcript = evidence.get('save_transcript_sha256')
    if (evidence.get('save_method') != 'nsh_param_save_default_sync'
            or type(evidence.get('save_exit_code')) is not int
            or evidence['save_exit_code'] != 0 or not isinstance(transcript,str)
            or re.fullmatch('[0-9a-f]{64}',transcript) is None):
        problems.append('synchronous_save_completion_not_evidenced')
    if evidence.get('operator_reviewed') is not True:
        problems.append('operator_evidence_review_missing')
    return result(review_complete=not problems, reasons=sorted(set(problems)),
        comparison=comparison, changed_parameters=changed, unexpected_changes=unexpected,
        protected_parameters_unchanged=not unexpected and old.keys() == new.keys(),
        evidence_sha256=profile.fingerprint(evidence),
        persistence_proven_by_this_program=False,
        note='Artifact consistency only; supplied claims/hashes are not authentic hardware proof')


def first_fusion_review(capture):
    """Offline decoded EVF2 evidence + LOCAL_POSITION_NED/ODOMETRY samples.

    Existing actual aid/innovation/selector/reset checks are reused. Review
    limits are explicit human inputs, never new live gate defaults. All phases
    use original source times plus host monotonic times; no invented stamps.
    """
    from fusion_binary_receiver import fusion_evidence
    from fusion_binary_protocol import RESET_FIELDS
    from compact_vision import ground_receipt_evidence
    failures = []
    limits = capture.get('limits', {})
    keys = ('position_step_m', 'height_step_m', 'yaw_step_rad', 'settled_position_span_m')
    if (not isinstance(limits,dict) or limits.get('reviewed') is not True or not limits.get('evidence')
            or any(type(limits.get(k)) not in (int, float)
                   or not math.isfinite(limits[k]) or limits[k] <= 0 for k in keys)):
        return result(review_complete=False, reasons=['jump_limits_not_reviewed'])
    rows = capture.get('samples', [])
    audits = capture.get('audits', [])
    yaws = capture.get('yaw_comparisons', [])
    contexts = capture.get('audit_context', [])
    start = capture.get('first_position_send_ns')
    if type(start) is not int or start < 0:
        return result(review_complete=False, reasons=['first_send_time_missing'])
    try:
        if len(rows) < 4 or len(audits) != 3 or len(audits) != len(yaws) or len(audits) != len(contexts):
            raise ValueError('baseline_first_and_settled_evidence_missing')
        coordinate = capture.get('coordinate_review',{})
        if (not isinstance(coordinate,dict) or coordinate.get('reviewed') is not True or coordinate.get('local_axes') != 'x_y_z_down_yaw_aligned'
                or any(not isinstance(coordinate.get(k),str)
                    or re.fullmatch('[0-9a-f]{64}',coordinate[k]) is None
                    for k in ('transform_sha256','rigid_body_revision_sha256'))):
            raise ValueError('coordinate_transform_review_missing')
        last_at = last_source = -1
        for row in rows:
            at, source = row['at_ns'], row['source_us']
            vec = row['position_local_m'] + [row['yaw_rad']]
            if (type(at) is not int or type(source) is not int or source <= 0 or at <= last_at or source <= last_source
                    or len(vec) != 4 or any(type(x) not in (int, float) or not math.isfinite(x) for x in vec)):
                raise ValueError('invalid_or_cached_estimator_samples')
            if row.get('identity') != capture.get('identity') or row.get('boot') != capture.get('boot'):
                raise ValueError('identity_or_boot_changed')
            if not capture.get('identity') or not capture.get('boot'):
                raise ValueError('session_binding_missing')
            if (not isinstance(row.get('reset_counters'),dict)
                    or any(type(row['reset_counters'].get(k)) is not int
                           or not 0 <= row['reset_counters'][k] < 256 for k in RESET_FIELDS)):
                raise ValueError('reset_counters_missing_or_invalid')
            last_at, last_source = at, source
        before = [r for r in rows if r['at_ns'] < start]
        after = [r for r in rows if r['at_ns'] >= start]
        if not before or len(after) < 3 or before[-1].get('ev_fused') is not False:
            raise ValueError('pre_first_fusion_baseline_missing')
        if not all(r.get('ev_fused') is True for r in after):
            failures.append('fusion_not_continuous')
        receipt = capture.get('receipt', {})
        if (not isinstance(receipt,dict) or receipt.get('identity') != capture['identity']
                or receipt.get('boot') != capture['boot']):
            raise ValueError('receipt_identity_or_boot_changed')
        receipt_review = ground_receipt_evidence(receipt['observations'],receipt['actual_sends'])
        origin = receipt['session_origin_ns']
        if type(origin) is not int or origin < 0:
            raise ValueError('receipt_session_origin_missing')
        times = [origin+int(s['elapsed_s']*1e9) for s in receipt['actual_sends']]
        received_times = [origin+int(s['at']*1e9) for s in receipt['observations']]
        if (times[0] != start or times[-1] > rows[-1]['at_ns']
                or any(not start <= at <= rows[-1]['at_ns'] for at in received_times)):
            raise ValueError('receipt_outside_current_position_window')
        last_audit = -1
        for context in contexts:
            at = context['at_ns']
            if (type(at) is not int or not start <= at <= rows[-1]['at_ns'] or at <= last_audit
                    or context.get('identity') != capture['identity'] or context.get('boot') != capture['boot']):
                raise ValueError('audit_not_bound_to_current_post_send_session')
            last_audit = at
        if not fusion_evidence(audits, yaws):
            failures.append('actual_ekf_fusion_not_evidenced')
        # Track all samples including transient first-reset spikes, not just endpoints.
        steps = []
        for a, b in zip([before[-1]] + after, after):
            delta = [y-x for x, y in zip(a['position_local_m'], b['position_local_m'])]
            steps.append(dict(at_ns=b['at_ns'], position_m=math.sqrt(sum(x*x for x in delta)),
                height_m=abs(delta[2]), yaw_rad=abs(math.remainder(b['yaw_rad']-a['yaw_rad'], 2*math.pi))))
        for row in steps:
            if (row['position_m'] > limits['position_step_m'] or row['height_m'] > limits['height_step_m']
                    or row['yaw_rad'] > limits['yaw_step_rad']):
                failures.append('first_fusion_or_later_jump')
        settled = after[-3:]
        span = max(math.dist(a['position_local_m'], b['position_local_m']) for a in settled for b in settled)
        if span > limits['settled_position_span_m']:
            failures.append('settled_position_unstable')
        for key in RESET_FIELDS:
            # First-fusion resets are reported, not silently rebased. Later resets fail.
            if any(r['reset_counters'][key] != after[0]['reset_counters'][key] for r in after[1:]):
                failures.append('reset_after_initial_fusion:' + key)
        resets = {k: [before[-1]['reset_counters'][k], after[0]['reset_counters'][k]] for k in RESET_FIELDS}
    except (KeyError, TypeError, ValueError, RuntimeError, IndexError) as exc:
        return result(review_complete=False, reasons=['incomplete_fusion_capture:' + str(exc)])
    return result(review_complete=not failures, reasons=sorted(set(failures)),
        steps=steps, settled_span_m=span, initial_reset_counters=resets,
        receipt_review=receipt_review,
        note='Existing fusion predicates + reviewed discontinuity limits; not a live authorization')
