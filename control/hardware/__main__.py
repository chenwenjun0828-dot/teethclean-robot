"""Explicit offline, read-only and motion entry points for the ROS 1 bridge."""
import argparse
import json
from pathlib import Path

from .trajectory_contract import read_json


def main(argv=None):
    parser = argparse.ArgumentParser(description='UR7e saved trajectory bridge (ROS 1)')
    commands = parser.add_subparsers(dest='command', required=True)
    export = commands.add_parser('export', help='Offline export; never connects to ROS')
    export.add_argument('--plan', required=True)
    export.add_argument('--out', required=True)
    export.add_argument('--mapping', help='JSON with mapping and joint_names; default is explicit reference mapping')
    export.add_argument('--step-s', type=float, default=.01)
    export.add_argument('--slowdown', type=float, default=1.)
    inspect_parser = commands.add_parser('inspect', help='Check bundle and list blockers without ROS')
    inspect_parser.add_argument('--trajectory', required=True)
    inspect_parser.add_argument('--deployment')
    for name in ('check', 'execute'):
        command = commands.add_parser(name, help='Read-only ROS state check' if name == 'check' else 'Explicit protected motion')
        command.add_argument('--trajectory', required=True)
        command.add_argument('--deployment', required=True)
        command.add_argument('--out', required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == 'export':
            from .ros_export import export_plan
            mapping = read_json(args.mapping) if args.mapping else {}
            result = export_plan(args.plan, args.out, mapping=mapping.get('mapping'),
                                 target_names=mapping.get('joint_names'), step_s=args.step_s,
                                 slowdown=args.slowdown)
            result = {key: result[key] for key in ('points', 'duration_s', 'execution_blockers')}
            print(json.dumps(result, ensure_ascii=False, indent=2))
            print(str(Path(args.out).resolve()/'trajectory.json'))
            return 0
        if args.command == 'inspect':
            from .ros_runner import inspect
            result = inspect(args.trajectory, args.deployment)
        else:
            from .ros_runner import run
            from .ros1_trajectory import Ros1TrajectoryBackend
            result = run(args.trajectory, args.deployment, args.out, Ros1TrajectoryBackend,
                         execute=args.command == 'execute')
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result['status'] in ('offline_checks_passed', 'read_only_checked', 'completed') else 2
    except (ValueError, RuntimeError, OSError, KeyError, ImportError) as error:
        print('Bridge refused: '+str(error))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
