"""Export a labelled tooth-cloud / landmarks / gingival-path diagnostic figure."""
import argparse
import json
from pathlib import Path


def render(directory):
    from ..config import CONTINUOUS_PATH_KINDS
    directory=Path(directory)
    report=json.loads((directory/'planning_report.json').read_text(encoding='utf-8'))
    if report.get('path_kind') in CONTINUOUS_PATH_KINDS:
        from .sweep_preview import render as render_sweep
        return render_sweep(directory)
    raise ValueError('Only six-region continuous plans are supported; replan')


def main(argv=None):
    from pipeline.parameters import add_parameter_file, parse_parameters
    parser=argparse.ArgumentParser(description=__doc__)
    add_parameter_file(parser)
    parser.add_argument('--plan',type=Path,required=True)
    args=parse_parameters(parser, argv)
    print(render(args.plan).resolve())


if __name__=='__main__':main()
