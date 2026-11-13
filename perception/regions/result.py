"""Explicit, replayable boundary between perception and robot planning."""
from __future__ import annotations

from dataclasses import dataclass, replace
from copy import deepcopy
import json
from pathlib import Path
import numpy as np

from .config import RegionConfig, SweepOptions, REGION_ORDER, continuous_path_kind
from .continuous import SweepPath
from .geometry import Geometry, Stroke
from .io import Inputs, dump, fingerprint, validate_transform, relative_path, manifest_path


def resolve_regional_dataset(inputs, dataset=None):
    """Require a matching regional source; never fall back to extracted topology."""
    from perception.regional_paths import load_regional_paths, RegionalPaths
    from .io import ROOT
    if not isinstance(dataset, RegionalPaths):
        folder = ROOT/'assets/regional_paths'/inputs.jaw if dataset is None else Path(dataset)
        if not folder.is_dir():
            raise FileNotFoundError(f'Matching six-region dataset is required: {folder}')
        dataset = load_regional_paths(folder)
    if dataset.jaw != inputs.jaw or fingerprint(inputs.scan)['sha256'] != dataset.metadata['mesh_sha256']:
        raise ValueError('Regional paths do not belong to the requested scan/jaw')
    return dataset


def validate_region_inputs(inputs, dataset=None):
    dataset = resolve_regional_dataset(inputs, dataset)
    for name in ('summary.json', 'segmentation.ply', 'prediction.npz'):
        path = Path(inputs.prediction)/name
        if not path.is_file():
            raise FileNotFoundError(path)
    summary = json.loads((Path(inputs.prediction)/'summary.json').read_text(encoding='utf-8'))
    if summary['input_sha256'] != fingerprint(inputs.scan)['sha256']:
        raise ValueError('CrossTooth input SHA256 differs from the current scan')
    return dataset


@dataclass
class PreparedRegion:
    inputs: Inputs
    geometry: Geometry
    lookup: dict
    reference: SweepPath
    world_from_scan: np.ndarray | None = None
    output_dir: Path | None = None

    @property
    def region(self):
        return self.reference.record['region']

    def validate(self, region=None):
        continuous_path_kind(self.region)
        if (self.reference.record.get('trim_scope') != 'region_endpoints_only'
                or any(part['kind'] != 'source' for part in self.reference.parts)):
            raise ValueError('Only source-preserving six-region perception results are supported')
        if region is not None and region != self.region:
            raise ValueError('Planning region differs from perception output')
        if self.geometry.report.get('region') != self.region:
            raise ValueError('Perception geometry and continuous line regions differ')
        if not self.reference.parts:
            raise ValueError('Perception output contains no continuous gingival line')
        for item in self.geometry.report.get('inputs', {}).values():
            if fingerprint(item['path'])['sha256'] != item['sha256']:
                raise ValueError('Perception source changed; rerun perception')
        if self.world_from_scan is not None:
            validate_transform(self.world_from_scan)
        return self

    def save(self, output, *, diagnostics=True):
        output = Path(output).resolve()
        output.mkdir(parents=True, exist_ok=True)
        # Only serialized provenance is relative; live planning objects keep resolved paths.
        report = deepcopy(self.geometry.report)
        report['path_base'] = 'manifest'
        for item in report.get('inputs', {}).values():
            item['path'] = relative_path(item['path'], output)
        metadata = report.get('source_metadata', {})
        if metadata.get('dataset_dir'):
            metadata['dataset_dir'] = relative_path(metadata['dataset_dir'], output)
        replace(self.geometry, report=report).save(output, diagnostics=diagnostics)
        np.savez_compressed(output/'tooth_lookup.npz', **self.lookup)
        np.savez_compressed(output/'region_mesh.npz', vertices=self.geometry.mesh_vertices_m,
                            faces=self.geometry.mesh_faces, labels=self.geometry.face_tooth_ids)
        dump(output/'continuous_gingival_line.json', self.reference.record)
        stations = self.reference.samples(.00025)
        points = np.array([self.reference.evaluate(s) for s in stations])
        normals = np.array([self.reference.normal(s) for s in stations])
        np.savez_compressed(output/'continuous_gingival_line.npz', parameters_m=stations,
                            points_scan_m=points, normals_scan=normals)
        if self.world_from_scan is not None:
            world = validate_transform(self.world_from_scan)
            np.save(output/'world_from_scan_m.npy', world)
            np.savetxt(output/'continuous_gingival_line_world_m.csv',
                       points@world[:3, :3].T+world[:3, 3], delimiter=',',
                       header='x_m,y_m,z_m', comments='')
        dump(output/'perception_region.json', {
            'schema_version': 1, 'module': 'perception', 'jaw': self.inputs.jaw,
            'path_base': 'manifest',
            'region': self.region, 'units': {'position': 'm', 'source': 'mm'},
            'inputs': {'jaw': self.inputs.jaw, **{key: relative_path(getattr(self.inputs, key), output)
                                                for key in ('scan', 'topology', 'prediction')}},
            'world_from_scan_m': self.world_from_scan,
            'files': {name: fingerprint(output/name)['sha256'] for name in (
                'geometry.json', 'geometry.npz', 'tooth_planes.json', 'tooth_lookup.npz',
                'region_mesh.npz', 'continuous_gingival_line.json', 'continuous_gingival_line.npz')},
        })
        self.output_dir = output
        return output/'perception_region.json'

    @classmethod
    def load(cls, path):
        path = Path(path).resolve()
        if path.is_dir():
            path /= 'perception_region.json'
        record = json.loads(path.read_text(encoding='utf-8'))
        if record.get('schema_version') != 1 or record.get('module') != 'perception':
            raise ValueError('Unsupported prepared perception region')
        folder = path.parent
        artifact_paths = record.get('artifact_paths', {})
        if set(artifact_paths) - {'region_mesh.npz'}:
            raise ValueError('Unsupported shared perception artifact')
        def artifact(name):
            return manifest_path(artifact_paths[name], folder, 'manifest') if name in artifact_paths else folder/name
        for name, digest in record['files'].items():
            if Path(name).name != name or fingerprint(artifact(name))['sha256'] != digest:
                raise ValueError('Prepared perception artifact changed; rerun perception')
        report = json.loads((folder/'geometry.json').read_text(encoding='utf-8'))
        report_base = report.pop('path_base', None)
        for item in report.get('inputs', {}).values():
            item['path'] = str(manifest_path(item['path'], folder, report_base))
        metadata = report.get('source_metadata', {})
        if metadata.get('dataset_dir'):
            metadata['dataset_dir'] = str(manifest_path(metadata['dataset_dir'], folder, report_base))
        planes = {int(k): v for k, v in json.loads((folder/'tooth_planes.json').read_text(encoding='utf-8')).items()}
        with np.load(folder/'geometry.npz', allow_pickle=False) as arrays:
            strokes = []
            for item in report['strokes']:
                mask = arrays['stroke_id'] == item['id']
                strokes.append(Stroke(item['id'], item['curve_id'], item['tooth'], item['region'],
                                      arrays['points_scan_m'][mask], arrays['tangents_scan'][mask],
                                      arrays['u'][mask], arrays['sample_index'][mask]))
        with np.load(artifact('region_mesh.npz'), allow_pickle=False) as mesh:
            geometry = Geometry(strokes, planes, report, mesh['vertices'], mesh['faces'], mesh['labels'])
        with np.load(folder/'tooth_lookup.npz', allow_pickle=False) as arrays:
            lookup = {name: arrays[name] for name in arrays.files}
        values = record['inputs']
        inputs = Inputs(values['jaw'], *(manifest_path(values[k], folder, record.get('path_base'))
                                        for k in ('scan', 'topology', 'prediction')))
        reference = SweepPath(json.loads((folder/'continuous_gingival_line.json').read_text(encoding='utf-8')))
        world = record.get('world_from_scan_m')
        return cls(inputs, geometry, lookup, reference,
                   None if world is None else validate_transform(world), folder).validate(record['region'])

    @classmethod
    def load_from_manifest(cls, path, region):
        """Resolve an entry in a six-region index relative to that index file."""
        path = Path(path).resolve()
        record = json.loads(path.read_text(encoding='utf-8'))
        if record.get('module') != 'perception' or record.get('schema_version') != 1:
            raise ValueError('Unsupported perception regions manifest')
        return cls.load(manifest_path(record['regions'][region], path.parent, record.get('path_base')))


def prepare_region(inputs, cfg=None, options=None, *, world_from_scan=None, output=None,
                   regional_dataset=None):
    """Perform all curve fitting and connection here, before entering planning."""
    cfg = RegionConfig() if cfg is None else RegionConfig.from_config(cfg)
    options = options or SweepOptions()
    from .frozen import build_regional_geometry, continuous_regional_reference
    dataset = validate_region_inputs(inputs, regional_dataset)
    geometry, lookup = build_regional_geometry(dataset, inputs, cfg)
    reference = continuous_regional_reference(geometry, cfg, options)
    reference.record['geometry_owner'] = 'perception'
    result = PreparedRegion(inputs, geometry, lookup, reference, world_from_scan).validate(cfg.region)
    if output is not None:
        result.save(output)
    return result


def prepare_regions(inputs, cfg=None, options=None, *, world_from_scan=None, output=None,
                    regional_dataset=None, regions=None):
    """Export independent continuous regions; never join different regions."""
    cfg = RegionConfig() if cfg is None else RegionConfig.from_config(cfg)
    regional_dataset = validate_region_inputs(inputs, regional_dataset)
    names = regions or REGION_ORDER
    results = {}
    for name in names:
        results[name] = prepare_region(inputs, replace(cfg, region=name), options,
            world_from_scan=world_from_scan, output=None if output is None else Path(output)/name,
            regional_dataset=regional_dataset)
    if output is not None:
        dump(Path(output)/'perception_regions.json', {
            'schema_version': 1, 'module': 'perception', 'jaw': inputs.jaw,
            'path_base': 'manifest',
            'region_order': list(names), 'cross_region_connections': False,
            'regions': {name: relative_path(result.output_dir/'perception_region.json', output)
                        for name, result in results.items()}})
    return results
