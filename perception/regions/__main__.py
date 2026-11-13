"""Export continuous gingival regions independently of robot planning."""
import argparse
from pathlib import Path
import numpy as np
from . import RegionConfig, SweepOptions, prepare_regions
from .io import Inputs
from .result import resolve_regional_dataset
from .config import REGIONS


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jaw', choices=('mandibular', 'maxillary'), default='mandibular')
    parser.add_argument('--scan', type=Path)
    parser.add_argument('--topology', type=Path)
    parser.add_argument('--prediction-dir', type=Path)
    parser.add_argument('--regional-dataset', type=Path)
    parser.add_argument('--region', choices=REGIONS, action='append', help='Export only selected regions; repeat to select several')
    parser.add_argument('--world-from-scan', type=Path, help='Registered rigid scan-to-world transform in metres')
    parser.add_argument('--trim-mm', type=float, default=1.)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.out.exists() and any(args.out.iterdir()):
        parser.error('Use a new empty perception output directory')
    inputs = Inputs.defaults(args.jaw)
    for name, value in (('scan', args.scan), ('topology', args.topology), ('prediction', args.prediction_dir)):
        if value is not None:
            setattr(inputs, name, value.resolve())
    try:
        dataset = resolve_regional_dataset(inputs, args.regional_dataset)
        world = None if args.world_from_scan is None else np.load(args.world_from_scan, allow_pickle=False)
        results = prepare_regions(inputs, RegionConfig(), SweepOptions(trim_mm=args.trim_mm), world_from_scan=world,
                                  regional_dataset=dataset, regions=args.region, output=args.out)
        for name, result in results.items():
            print(f'{name}: {result.reference.record["sweep_length_mm"]:.3f} mm -> {result.output_dir}')
        return 0
    except (ValueError, RuntimeError, OSError) as error:
        print(f'Perception regional preparation failed: {error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
