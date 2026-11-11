"""Run sensor-feedback force control on a verified regional trajectory."""
import argparse
import json
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True,
                        help='Verified tracking directory produced by the current pipeline')
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--target-force-n', type=float, default=1.)
    parser.add_argument('--force-limit-n', type=float, default=3.)
    parser.add_argument('--mode', choices=('trajectory',), default='trajectory',
                        help='Trajectory execution is the default; the legacy probe entry point was retired')
    parser.add_argument('--max-duration-s', type=float, default=60.,
                        help='Full-path execution time budget; unmet contact/path requirements cause failure')
    parser.add_argument('--trace-speed-mm-s', type=float, default=2.,
                        help='Full-path nominal speed for allocating segment time, further slowed by contact feedback')
    parser.add_argument('--contact-schedule', choices=('auto', 'regional', 'segments'), default='auto',
                        help='auto: one continuous contact trace for regional sources; segments: legacy acquire/release per arc')
    parser.add_argument('--segment-speed-mm-s', action='append', default=[], metavar='SEGMENT:SPEED',
                        help='Override nominal speed for one retained segment (one-based); keeps all force/collision guards')
    parser.add_argument('--segment-roll-deg', action='append', default=[], metavar='SEGMENT:DEGREES',
                        help='Optional axial roll correction for one-based retained segment; +/-30 degrees maximum')
    parser.add_argument('--viewer', action='store_true',
                        help='Open MuJoCo and a live force-sensor panel on the right (trajectory mode)')
    parser.add_argument('--contact-margin-mm', type=float, default=.05,
                        help='MuJoCo contact activation margin, in mm; logged in report')
    args = parser.parse_args(argv)
    from .compliance import ForceControlConfig
    from .contact_path import ContactPathConfig
    from .force_trajectory import run as run_trajectory
    try:
        cfg = ForceControlConfig(target_force_N=args.target_force_n, force_stop_N=args.force_limit_n)
        path_cfg = ContactPathConfig(max_duration_s=args.max_duration_s,
                                     max_trace_speed_m_s=args.trace_speed_mm_s*.001)
        path_cfg.validate(cfg)
    except ValueError as error:
        parser.error(str(error))
    corrections = {}
    for item in args.segment_roll_deg:
        try:
            index, angle = item.split(':')
            index, angle = int(index)-1, float(angle)
        except ValueError:
            parser.error('--segment-roll-deg expects SEGMENT:DEGREES, e.g. 9:-10')
        if index in corrections: parser.error('Duplicate segment roll correction')
        corrections[index] = angle
    speeds = {}
    for item in args.segment_speed_mm_s:
        try:
            index, speed = item.split(':')
            index, speed = int(index)-1, float(speed)*.001
        except ValueError:
            parser.error('--segment-speed-mm-s expects SEGMENT:SPEED, e.g. 1:1')
        if index in speeds: parser.error('Duplicate segment speed override')
        speeds[index] = speed
    report = run_trajectory(args.source, args.out, cfg, path_cfg,
        args.contact_margin_mm, corrections, args.viewer, segment_speeds_m_s=speeds,
        contact_schedule=args.contact_schedule)
    print(json.dumps({key: report[key] for key in ('status', 'force_control_passed', 'stop_reason',
        'recorded_duration_s', 'retained_path_progress_fraction', 'peak_compensated_force_N',
        'force_tracking_passed', 'scanned_surface_contact_passed')}))
    return 0 if report['force_control_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
