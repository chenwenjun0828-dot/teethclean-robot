"""Run hold -> short -> regional, with a saved, fail-closed gate between stages."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil

from planning.infrastructure.io import ROOT, dump, fingerprint
from .runner import main as run_stage


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params',type=Path,default=ROOT/'config/force_outer_front.json')
    parser.add_argument('--out',type=Path)
    parser.add_argument('--contact-model',type=Path,help='Override contact manifest without editing the saved config')
    parser.add_argument('--viewer',action='store_true')
    args=parser.parse_args(argv)
    output=(args.out or ROOT/'outputs/control/force/local_front'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')).resolve()
    if output.exists() and any(output.iterdir()): parser.error('Use a new empty output directory')
    params=args.params.resolve()
    config=json.loads(params.read_text())
    if config.get('force_component')!='normal' or config.get('contact_margin_mm')!=0:
        parser.error('Local staged experiments require normal force and zero contact margin')
    manifest=(args.contact_model or ROOT/config['contact_model']).resolve()
    output.mkdir(parents=True,exist_ok=True)
    summary=dict(simulation_only=True, scope='Local front work-band validation only',
        status='running', force_control_passed=False, configuration=fingerprint(params),
        contact_manifest=fingerprint(manifest), stages=[])
    evidence=output/'contact_assets'; evidence.mkdir()
    for name in ('manifest.json','geometry_failure.json','geometry_failures.json','failed_patch.npz','contact_layer_overlay.png','first_contact_errors.png','first_contact_errors.csv','gap_errors.csv'):
        file=manifest.parent/name
        if file.exists(): shutil.copyfile(file,evidence/name)
    stopped=False
    for stage, force, tolerance in [('hold',.5,.1),('short',.5,.1),('regional',1.,.2)]:
        if stopped:
            summary['stages'].append(dict(stage=stage,status='not_run',reason='Previous stage did not pass'))
            continue
        command=['--params',str(params),'--out',str(output/stage),'--stage',stage,
                 '--contact-model',str(manifest),
                 '--target-force-n',str(force),'--force-tolerance-n',str(tolerance),
                 '--trace-speed-mm-s','1','--contact-schedule','regional','--max-duration-s','300']
        if args.viewer: command.append('--viewer')
        if stage!='hold':
            command.extend(['--previous-stage',str(output/('hold' if stage=='short' else 'short')/'force_trajectory_report.json')])
        try:
            code=run_stage(command)
            report_path=output/stage/'force_trajectory_report.json'
            report=json.loads(report_path.read_text())
            passed=code==0 and report.get('stage_passed') is True
            summary['stages'].append(dict(stage=stage,status='passed' if passed else 'failed',
                report=str(report_path.relative_to(output)),stop_reason=report.get('stop_reason'),
                stage_passed=passed))
        except (ValueError, OSError, RuntimeError) as error:
            passed=False
            summary['stages'].append(dict(stage=stage,status='failed',stop_reason=str(error),stage_passed=False))
        stopped=not passed
        dump(output/'stages_report.json',summary)
    summary.update(status='stopped' if stopped else 'completed',force_control_passed=not stopped)
    dump(output/'stages_report.json',summary)
    print(json.dumps(dict(output=str(output),**summary),ensure_ascii=False))
    return 2 if stopped else 0


if __name__=='__main__': raise SystemExit(main())
