"""Oral-scan and RGB-D perception interface for one jaw."""
from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path

import mujoco

from .gingival_extraction import extract_gingival_line
from .gingival_line import RegisteredPaths, load_registered_paths
from .global_camera import capture_rgbd, save_capture
from .registration import register_capture, resolve_registration_method


@dataclass
class PerceptionResult:
    paths: RegisteredPaths
    registration_dir: Path
    topology_path: Path
    capture_dir: Path
    extraction_dir: Path
    regions: dict = field(default_factory=dict)
    regions_manifest: Path | None = None


def perceive_gingival_line(model, data, jaw, scan, config, output_dir,
                           frame=None, landmark_provider=None, on_stage=None,
                           registration_method=None, region_config=None, sweep_options=None,
                           prediction_dir=None, regional_dataset=None) -> PerceptionResult:
    """Return ordered gingival chains in world metres from a colored scan and RGB-D."""
    scan, output_dir = Path(scan).resolve(), Path(output_dir).resolve()
    method = resolve_registration_method(config, registration_method)
    if method == 'automatic' and landmark_provider is not None:
        raise ValueError('Automatic registration does not accept a landmark provider')

    def stage(name, details=None):
        if on_stage:
            on_stage(name, details)

    stage('extraction')
    extraction_dir = output_dir / 'extraction'
    line = extract_gingival_line(scan, extraction_dir, jaw)
    stage('extraction', {'points': len(line['points_mm']), 'edges': len(line['edges']),
                         'surface': line['surface']})
    run_config = dict(config)
    run_config['registration_method'] = method
    run_config[jaw + '_mesh'] = str(scan)
    run_config[jaw + '_line'] = str(line['line_csv'].resolve())
    config_path = output_dir / 'registration_config.json'
    config_path.write_text(json.dumps(run_config, ensure_ascii=False, indent=2), encoding='utf-8')

    stage('capture')
    mujoco.mj_forward(model, data)
    if frame is None:
        frame = capture_rgbd(model, data, run_config['camera'],
                             run_config['width'], run_config['height'])
    capture_dir = output_dir / 'capture'
    metadata = save_capture(capture_dir, frame, model, data, jaw, run_config)
    stage('capture', {'capture_id': metadata['capture_id'],
                      'valid_points': metadata['valid_points']})

    stage('registration')
    landmarks = (landmark_provider(model, data, metadata, scan, output_dir)
                 if landmark_provider else None)
    registration_dir = register_capture(capture_dir, config_path, jaw,
                                        output_dir / 'registration', landmarks=landmarks,
                                        registration_method=method)
    paths = load_registered_paths(registration_dir, jaw, line['topology_obj'])
    stage('registration', {'chains': len(paths.chains),
                           'registration_method': method,
                           'registration_dir': str(registration_dir)})
    from .regions import prepare_regions
    from .regions.io import Inputs
    from .regions.result import resolve_regional_dataset
    inputs = Inputs.defaults(jaw)
    inputs.scan, inputs.topology = scan, line['topology_obj']
    if prediction_dir is not None:
        inputs.prediction = Path(prediction_dir)
    dataset = resolve_regional_dataset(inputs, regional_dataset)
    stage('regional_paths')
    regions_dir = output_dir/'regions'
    regions = prepare_regions(inputs, region_config, sweep_options,
        world_from_scan=paths.world_from_scan, output=regions_dir, regional_dataset=dataset)
    stage('regional_paths', {'geometry_owner': 'perception', 'regions': list(regions),
          'manifest': str(regions_dir/'perception_regions.json'),
          'continuous_lengths_mm': {name: result.reference.record['sweep_length_mm']
                                    for name, result in regions.items()}})
    return PerceptionResult(paths, registration_dir, line['topology_obj'],
                            capture_dir, extraction_dir, regions,
                            regions_dir/'perception_regions.json')
