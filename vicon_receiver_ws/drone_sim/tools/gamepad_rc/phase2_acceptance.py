#!/usr/bin/env python3
"""Plan B phase 2 OFFLINE artifact review. No connect/port/HOME/write options.

Reports are not accepted by any hardware permit consumer. Existing output files
are never overwritten. All inputs use the existing bounded regular-JSON loader.
"""
import argparse
import json
import time
from pathlib import Path

from ekf_static_profile import load_json
from ekf_ground_acceptance import inventory, persistence_review, first_fusion_review
from ground_link_diagnostics import summarize
from ev_delay_calibration import calibrate


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='Exclusive new offline report')
    sub = parser.add_subparsers(dest='operation', required=True)
    audit = sub.add_parser('inventory')
    audit.add_argument('--profile', required=True, type=Path)
    audit.add_argument('--source-root', required=True, type=Path)
    persist = sub.add_parser('persistence')
    for name in ('profile','before','after','evidence'):
        persist.add_argument('--'+name, required=True, type=Path)
    ulog = sub.add_parser('delay-ulog')
    ulog.add_argument('--capture', required=True, type=Path)
    ulog.add_argument('--ulog', required=True, type=Path)
    ulog.add_argument('--identity', required=True, type=Path, help='Reviewed sys_uuid/ver_sw/ver_hw JSON')
    ulog.add_argument('--imu-source', choices=('sensor_combined','vehicle_imu'), default='sensor_combined')
    ulog.add_argument('--timing-method',choices=('native-fc','reviewed-sdk'),default='native-fc',
                      help='Same-ULog sample times by default; no host TIMESYNC in the fit')
    ulog.add_argument('--sensor-selection-evidence', type=Path,
                      help='Optional reviewed same-boot read-only cache brackets; never inferred or collected')
    for name in ('fusion','delay','delay-capture','timing'):
        child = sub.add_parser(name)
        child.add_argument('--capture', required=True, type=Path)
        if name == 'timing':
            child.add_argument('--start-ns', required=True, type=int)
            child.add_argument('--end-ns', required=True, type=int)
    args = parser.parse_args(argv)
    try:
        if args.operation == 'inventory':
            report = inventory(load_json(args.profile), args.source_root)
        elif args.operation == 'persistence':
            report = persistence_review(*(load_json(getattr(args,n)) for n in ('profile','before','after','evidence')))
        elif args.operation == 'timing':
            report = summarize(load_json(args.capture), start_ns=args.start_ns, end_ns=args.end_ns)
        elif args.operation == 'delay-ulog':
            from ev_delay_sources import load_ulog_file, merge_ulog_capture
            from ev_delay_capture import analyze
            combined, imported = merge_ulog_capture(load_json(args.capture),load_ulog_file(args.ulog),
                expected_info=load_json(args.identity),import_ns=time.monotonic_ns(),source=args.imu_source,
                native_fc_time=args.timing_method=='native-fc',
                selection_evidence=(load_json(args.sensor_selection_evidence)
                                    if args.sensor_selection_evidence is not None else None))
            report = dict(**analyze(combined),ulog_import=imported,merged_capture=combined)
        elif args.operation == 'delay-capture':
            from ev_delay_capture import analyze
            report = analyze(load_json(args.capture))
        else:
            operation = {'fusion':first_fusion_review, 'delay':calibrate}[args.operation]
            report = operation(load_json(args.capture))
        rendered = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
        if args.output:
            with args.output.open('x') as stream:
                stream.write(rendered)
        else:
            print(rendered,end='')
        # Inventory success means the source was audited, not its values approved.
        if args.operation == 'inventory':
            return 0
        success = report.get('timing_window_clear') if args.operation == 'timing' else report.get('review_complete',report.get('candidate_identified'))
        return 0 if success else 2
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.exit(2,'Offline acceptance failed: '+str(exc)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
