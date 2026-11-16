"""One local CLI planning process with immutable inputs and bounded result access."""
from datetime import datetime
import copy
import json
from pathlib import Path
import subprocess
import sys
import threading
import uuid

from planning.config import Config, resolve_pose_angles
from planning.infrastructure.io import ROOT
from .planning_results import actual_tcp_path, read_json, summarize


PARAMETERS = frozenset({'region', 'planning_mode', 'pose_angles_deg', 'standoff_mm',
                        'speed_mm_s', 'transfer_speed_mm_s', 'pose_spacing_mm',
                        'local_angle_search', 'check_mode'})


class PlanningJobs:
    def __init__(self, source, output_root=None):
        self.source = source
        self.output_root = Path(output_root or ROOT / 'outputs/interactive_planning').resolve()
        self.lock = threading.RLock()
        self.jobs = {}
        self.active = None
        self.closed = False

    def configuration(self, values):
        if not isinstance(values, dict) or set(values) != PARAMETERS:
            raise ValueError('Submit exactly the supported planning parameters')
        values = copy.deepcopy(values)
        angles = resolve_pose_angles(values['pose_angles_deg'])
        cfg = Config(region=values['region'], planning_mode=values['planning_mode'],
                     tilt_deg=angles[0], tilt_azimuth_deg=angles[1],
                     standoff_mm=values['standoff_mm'], motion_mode='non-contact',
                     speed_mm_s=values['speed_mm_s'], transfer_speed_mm_s=values['transfer_speed_mm_s'],
                     pose_spacing_mm=values['pose_spacing_mm'],
                     local_angle_search=values['local_angle_search'], check_mode=values['check_mode'])
        if cfg.standoff_mm > 30:
            raise ValueError('Interactive standoff must be within 0..30 mm')
        source = self.source(cfg.region)
        return {**source, **values, 'motion_mode': 'non-contact',
                'geometry_only': False, 'execute': False, 'headless': True,
                'orientation_audit': 'strict'}

    def start(self, values):
        with self.lock:
            if self.closed:
                raise ValueError('Planning service is closing')
            if self.active is not None:
                raise ValueError('A planning task is already running')
            configuration = self.configuration(values)
            job_id = datetime.now().strftime('%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8]
            directory = self.output_root / job_id
            directory.mkdir(parents=True, exist_ok=False)
            configuration['out'] = str(directory / 'plan')
            (directory / 'parameters.json').write_text(
                json.dumps(configuration, indent=2, allow_nan=False), encoding='utf-8')
            job = {'id': job_id, 'state': 'running', 'parameters': copy.deepcopy(values),
                   'directory': str(directory), 'configuration': configuration,
                   'result': None, 'cancel_requested': False}
            self.jobs[job_id] = job
            self.active = job_id
            try:
                with (directory / 'planner.log').open('wb') as log:
                    job['process'] = subprocess.Popen(
                        [sys.executable, '-u', '-m', 'planning', '--params', str(directory / 'parameters.json')],
                        cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                        creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
            except OSError as error:
                job.update(state='failed', error=str(error))
                self.active = None
                self._save(job)
                return self.snapshot(job_id)
            self._save(job)
            job['thread'] = threading.Thread(target=self._finish, args=(job,), daemon=True)
            job['thread'].start()
            return self.snapshot(job_id)

    def _save(self, job):
        record = {key: value for key, value in job.items() if key not in ('process', 'thread')}
        (Path(job['directory']) / 'task.json').write_text(
            json.dumps(record, indent=2, allow_nan=False), encoding='utf-8')

    def _finish(self, job):
        code = job['process'].wait()
        try:
            with self.lock:
                if job['cancel_requested']:
                    job['state'] = 'cancelled'
                    return
                job['stage'] = 'reading_certificates'
            directory = Path(job['directory']) / 'plan'
            result = summarize(directory, job['parameters']['check_mode'], code)
            try:
                result['path'] = actual_tcp_path(directory)
            except (ValueError, RuntimeError, OSError, KeyError) as error:
                result['display_error'] = str(error)
            with self.lock:
                job.update(result=None if job['cancel_requested'] else result, returncode=code,
                           state='cancelled' if job['cancel_requested'] else
                           ('succeeded' if result['passed'] else 'failed'))
        except Exception as error:
            with self.lock:
                job.update(state='cancelled' if job['cancel_requested'] else 'failed', error=str(error))
        finally:
            with self.lock:
                self.active = None
                self._save(job)

    def snapshot(self, job_id=None):
        with self.lock:
            if job_id is None:
                if not self.jobs:
                    return None
                job_id = next(reversed(self.jobs))
            if job_id not in self.jobs:
                raise ValueError('Unknown task')
            job = self.jobs[job_id]
            value = copy.deepcopy({key: item for key, item in job.items()
                                   if key not in ('thread', 'process')})
        directory = Path(value['directory'])
        with (directory / 'planner.log').open('rb') as log:
            log.seek(0, 2)
            log.seek(max(0, log.tell()-16000))
            value['log'] = log.read().decode('utf-8', errors='replace')
        value['report_status'] = read_json(directory / 'plan/planning_report.json').get('status')
        plan = str(directory / 'plan')
        value['replay_command'] = f'& "{sys.executable}" -m planning.view --plan "{plan}" --paused'
        return value

    def cancel(self, job_id):
        with self.lock:
            if job_id not in self.jobs:
                raise ValueError('Unknown task')
            job = self.jobs[job_id]
            if job['state'] == 'running':
                job['cancel_requested'] = True
                if job['process'].poll() is None:
                    job['process'].terminate()
        return self.snapshot(job_id)

    def close(self):
        with self.lock:
            self.closed = True
            job_id = self.active
        if job_id:
            self.cancel(job_id)
            job = self.jobs[job_id]
            try:
                job['process'].wait(timeout=5)
            except subprocess.TimeoutExpired:
                job['process'].kill()
                job['process'].wait()
            job['thread'].join()
