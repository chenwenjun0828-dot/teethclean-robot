"""Render the selected six-region continuous reference."""
import argparse
import json
from pathlib import Path
from ..config import CONTINUOUS_PATH_KINDS


def render(plan_directory, output):
    source = Path(plan_directory)
    report = json.loads((source/'planning_report.json').read_text(encoding='utf-8'))
    if report.get('path_kind') not in CONTINUOUS_PATH_KINDS:
        raise ValueError('Only six-region continuous plans are supported; replan')
    from .sweep_preview import render as render_sweep
    return render_sweep(source, output)


def main(argv=None):
    from pipeline.parameters import add_parameter_file, parse_parameters
    parser = argparse.ArgumentParser(description=__doc__)
    add_parameter_file(parser)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--out', type=Path, default=Path('outputs/planning/segment_map'))
    args = parse_parameters(parser, argv)
    print(render(args.plan, args.out).resolve())


if __name__ == '__main__':
    main()
