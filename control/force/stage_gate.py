"""Validate the previous actual dental stage before advancing local experiments."""
import json
from pathlib import Path
from planning.infrastructure.io import fingerprint


def verify_previous(stage, source, manifest, previous, force_config, material_file=None):
    if stage=='hold': return None
    required='hold' if stage=='short' else 'short'
    if previous is None: raise ValueError(f'Local {stage} requires --previous-stage pointing to a passed {required} report')
    path=Path(previous).resolve()
    report=json.loads(path.read_text())
    if (report.get('experiment_stage')!=required or report.get('stage_passed') is not True
            or report.get('status')!='completed' or not report.get('scanned_surface_contact_passed')
            or not report.get('signal_audit_passed')):
        raise ValueError('Previous actual dental stage did not pass all acceptance gates')
    for key,file in [('source_trajectory','trajectory.npz'),('source_planning_report','planning_report.json')]:
        if report.get(key,{}).get('sha256')!=fingerprint(Path(source)/file)['sha256']:
            raise ValueError('Previous stage belongs to another source plan')
    model=report.get('contact_model_manifest') or {}
    if model.get('sha256')!=fingerprint(manifest)['sha256']:
        raise ValueError('Contact assets changed since previous stage')
    provenance=model.get('material_provenance')
    if material_file is not None and (not provenance or provenance.get('sha256')!=fingerprint(material_file)['sha256']):
        raise ValueError('Contact material changed since previous stage')
    from dataclasses import asdict
    config=asdict(force_config)
    if any(report['force_control'].get(k)!=v for k,v in config.items() if k!='target_force_N'):
        raise ValueError('Force controller or protection settings changed since previous stage')
    audit=path.parent/'contact_validation.json'
    if report.get('contact_validation_file',{}).get('sha256')!=fingerprint(audit)['sha256']:
        raise ValueError('Previous original-scan contact audit changed')
    return dict(required_stage=required,report=fingerprint(path),audit=fingerprint(audit))
