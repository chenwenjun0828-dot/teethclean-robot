"""Position-servo execution of a verified planning.trajectory.Trajectory."""
from pathlib import Path
import csv
import time
import numpy as np
import mujoco
from planning.infrastructure.robot import rotation_error
from planning.infrastructure.io import dump
from planning.pose_search.constraints import TCP_FRAME_ID, axis_convention
from planning.config import Config, CONTINUOUS_PATH_KINDS


def robot_contacts(model, data):
    result = []
    for contact in data.contact[:data.ncon]:
        names = [model.body(int(model.geom_bodyid[g])).name or '' for g in (contact.geom1, contact.geom2)]
        if any(n.startswith('ur_') for n in names):
            result.append(' / '.join(names))
    return result


def step_checked(model, data, steps=1):
    for _ in range(steps):
        mujoco.mj_step(model, data)
        if robot_contacts(model, data):
            raise RuntimeError('Robot contact during simulation')
        if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise RuntimeError('Non-finite robot state')
        if any(w.number for w in data.warning):
            raise RuntimeError('MuJoCo simulation warning')


class GingivalTCPController:
    def __init__(self, model, plan, checker):
        self.model, self.plan, self.checker = model, plan, checker
        self.robot = checker.robot
        if list(plan.joint_names) != list(self.robot.joint_names):
            raise ValueError('Trajectory joint names do not match the scene')
        self.start_time = None
        self.gain = model.actuator_gainprm[:, 0].copy()
        self.damping = -model.actuator_biasprm[:, 2].copy()
        if model.nu != 6 or np.any(self.gain <= 0) or not np.allclose(model.actuator_biasprm[:, 1], -self.gain):
            raise ValueError('Controller requires the six configured position actuators')
        if not np.array_equal(model.actuator_trnid[:, 0], self.robot.joints):
            raise ValueError('Actuator ordering differs from trajectory joint ordering')
        self.last_clearance_check = -np.inf
        self.minimum_clearance_m = float('inf')
        self.max_position_mm = 0.
        self.max_rotation_deg = 0.

    def update(self, data):
        if self.start_time is None:
            if np.max(abs(data.qpos[self.robot.qadr]-self.plan.joints[0])) > .01:
                raise ValueError('Robot is not at the validated trajectory start')
            self.start_time = float(data.time)
        elapsed = float(data.time-self.start_time)
        state = self.plan.sample(elapsed)
        if robot_contacts(self.model, data):
            raise RuntimeError('Robot contact during execution')
        if elapsed-self.last_clearance_check >= .01-1e-9:
            check = self.checker.check(data.qpos[self.robot.qadr], 0.)
            self.last_clearance_check = elapsed
            self.minimum_clearance_m = min(self.minimum_clearance_m, check.clearance_m)
            if not check.valid:
                raise RuntimeError('Actual geometry collision: '+check.pair)
        desired = self.robot.fk(state['q'])
        actual = self.robot.pose(data)
        error = float(np.linalg.norm(actual[:3, 3]-desired[:3, 3])*1000)
        angle = float(np.degrees(np.linalg.norm(rotation_error(desired[:3, :3], actual[:3, :3]))))
        self.max_position_mm = max(self.max_position_mm, error)
        self.max_rotation_deg = max(self.max_rotation_deg, angle)
        if elapsed > .1 and (error > 2. or angle > 2.):
            raise RuntimeError(f'Tracking error exceeds execution limit: {error:.3f} mm / {angle:.3f} deg')
        inertia = np.empty((self.model.nv, self.model.nv))
        if hasattr(data, 'qM'):
            mujoco.mj_fullM(self.model, inertia, data.qM)
        else:
            mujoco.mj_fullM(self.model, data, inertia)
        torque = inertia[np.ix_(self.robot.dadr, self.robot.dadr)] @ state['ddq'] + data.qfrc_bias[self.robot.dadr]
        command = state['q'] + (torque+self.damping*state['dq'])/self.gain
        data.ctrl[:] = np.clip(command, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1])
        return state, error, angle


def execute(model, data, plan, checker, output, headless=True):
    """Simulation only. The collision checker must be bound to this scene."""
    output = Path(output)
    controller = GingivalTCPController(model, plan, checker)
    viewer = None
    report = {'status': 'running', 'duration_s': plan.duration, 'simulation_only': True,
              'contact_force_control': False, 'geometry_check_period_s': .01,
              'self_contact_check_period_s': model.opt.timestep}
    if not headless:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
    rows, next_record, last_wall = [], 0., time.perf_counter()
    try:
        start = float(data.time)
        while data.time-start <= plan.duration+.1:
            if viewer is not None and not viewer.is_running():
                report['status'] = 'interrupted'; break
            state, error, angle = controller.update(data)
            if data.time-start >= next_record:
                rows.append([data.time-start, state['phase'], state['tooth'], error, angle,
                             *data.qpos[checker.robot.qadr]])
                next_record += .02
                if viewer is not None:
                    viewer.sync()
                    time.sleep(max(0., .02-(time.perf_counter()-last_wall)))
                    last_wall = time.perf_counter()
            step_checked(model, data)
        else:
            report['status'] = 'completed'
    except (ValueError, RuntimeError) as error:
        report.update(status='failed', error=str(error))
        raise
    finally:
        if viewer is not None: viewer.close()
        report.update(max_position_error_mm=controller.max_position_mm,
                      max_rotation_error_deg=controller.max_rotation_deg,
                      minimum_actual_clearance_mm=(controller.minimum_clearance_m*1000
                          if np.isfinite(controller.minimum_clearance_m) else None),
                      simulation_time_s=float(data.time))
        dump(output/'execution_report.json', report)
        with (output/'execution_trace.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s', 'phase', 'tooth', 'position_error_mm', 'rotation_error_deg', *plan.joint_names])
            writer.writerows(rows)
    return report


def load_verified(directory):
    """Use the same exported-trajectory verification as planning and its viewer."""
    import json
    report = json.loads((Path(directory)/'planning_report.json').read_text(encoding='utf-8'))
    expected_axes = axis_convention(Config.from_saved(report.get('config', {})))
    if report.get('axis_convention') != expected_axes or report.get('tcp_frame_id') != TCP_FRAME_ID:
        raise ValueError('Retired TCP frame: replan using shaft-axial +X and radial +Y')
    from planning.motion_planning.execution import load_verified as load
    return load(directory)


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description='Execute a verified regional plan in MuJoCo')
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args(argv)
    model, data, trajectory, checker = load_verified(args.plan)
    report = execute(model, data, trajectory, checker, args.plan, headless=args.headless)
    print(report)
    return 0 if report['status']=='completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
