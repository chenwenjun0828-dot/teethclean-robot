"""Real MuJoCo sphere/plane feedback test; not a dental trajectory validation."""
import argparse
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from .contact_experiment import contacts, write_csv
from planning.infrastructure.io import dump, fingerprint
from .compliance import ForceControlConfig, NormalAdmittance, SensorContactForce, smoothstep
from .force_guard import PredictiveContactGuard


MODEL = '''<mujoco>
  <option timestep=".001" integrator="implicitfast"/>
  <worldbody>
    <geom name="dental_mandibular_visual" type="plane" size=".1 .1 .01"/>
    <body name="ur_carrier" pos="0 0 .002">
      <joint name="normal" type="slide" axis="0 0 1" range="-.004 .004"/>
      <inertial pos="0 0 0" mass=".1" diaginertia=".001 .001 .001"/>
      <body name="ur_tool">
        <geom name="ur_end_scaler_geom" type="sphere" size=".001" mass=".05"/>
        <site name="ur_scaler_force_site"/>
        <site name="ur_scaler_tcp" pos="0 0 -.001"/>
      </body>
    </body>
  </worldbody>
  <actuator><position joint="normal" kp="1000" kv="20"
    ctrlrange="-.004 .004" forcerange="-5 5"/></actuator>
  <sensor><force name="ur_scaler_force" site="ur_scaler_force_site"/>
    <torque name="ur_scaler_torque" site="ur_scaler_force_site"/></sensor>
</mujoco>'''


def run(output, target_force_N=1.):
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError('Use a new empty output directory')
    output.mkdir(parents=True, exist_ok=True)
    config = ForceControlConfig(target_force_N=target_force_N)
    if target_force_N >= 2.:
        raise ValueError('Benchmark target must remain below the 2 N soft guard')
    model = mujoco.MjModel.from_xml_string(MODEL)
    data = mujoco.MjData(model)
    robot = SimpleNamespace(site=model.site('ur_scaler_tcp').id, qadr=np.array([0]))
    observer, controller = SensorContactForce(model), NormalAdmittance(config)
    guard = PredictiveContactGuard(model, robot, 'mandibular', observer, 2.5)
    rows, reason = [], ''
    for index in range(5001):
        mujoco.mj_forward(model, data)
        measured = observer.read(data)
        target = config.target_force_N*smoothstep(data.time/config.target_ramp_s)
        controller.step(measured['force_N'], target, model.opt.timestep)
        hits, bad = contacts(model, data, robot, 'mandibular')
        oracle = np.sum([[h[f'force_on_robot_world_{axis}_N'] for axis in 'xyz'] for h in hits], axis=0) if hits else np.zeros(3)
        rows.append({'time_s': float(data.time), 'target_N': float(target),
                     'force_N': measured['force_N'], 'normal_displacement_mm': controller.displacement*1000,
                     'tcp_height_mm': float(data.site_xpos[robot.site, 2]*1000),
                     'sensor_solver_difference_N': float(np.linalg.norm(measured['external_force_world_N']-oracle)),
                     **dict(zip(['Fx_N', 'Fy_N', 'Fz_N', 'Tx_Nm', 'Ty_Nm', 'Tz_Nm'], measured['raw_sensor_wrench']))})
        if bad or measured['force_N'] >= config.force_stop_N or any(w.number for w in data.warning):
            reason = 'Contact, force or dynamics guard triggered'
            break
        if index == 5000:
            break
        command = np.array([controller.displacement+data.qfrc_bias[0]/1000])
        predicted = guard.check(data, command, 'acquire')
        if not predicted['safe']:
            reason = predicted['reason']
            break
        data.ctrl[:] = command
        mujoco.mj_step(model, data)
    steady = np.array([r['force_N'] for r in rows if r['time_s'] >= 4.-1e-9])
    report = {'scope': 'One-axis sphere/plane controller component test only',
              'simulation_only': True, 'full_robot_contact_trajectory_verified': False,
              'mujoco_version': mujoco.__version__, 'force_control': asdict(config),
              'feedback': 'Compensated installed sensor; solver force is an independent audit only',
              'sample_rate_hz': 1000, 'duration_s': rows[-1]['time_s'], 'samples': len(rows),
              'stop_reason': reason, 'peak_force_N': max(r['force_N'] for r in rows),
              'steady_mean_N': float(steady.mean()) if len(steady) else None,
              'steady_min_N': float(steady.min()) if len(steady) else None,
              'steady_max_N': float(steady.max()) if len(steady) else None,
              'steady_rmse_N': float(np.sqrt(np.mean((steady-config.target_force_N)**2))) if len(steady) else None,
              'max_sensor_solver_difference_N': max(r['sensor_solver_difference_N'] for r in rows),
              'code': fingerprint(Path(__file__))}
    report['controller_benchmark_passed'] = bool(not reason and len(steady) >= 1000
        and abs(steady-config.target_force_N).max() <= .2 and report['peak_force_N'] < 2.5
        and report['max_sensor_solver_difference_N'] < 1e-6)
    write_csv(output/'benchmark_force.csv', rows)
    dump(output/'benchmark_report.json', report)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, constrained_layout=True)
    time = [r['time_s'] for r in rows]
    axes[0].plot(time, [r['force_N'] for r in rows], label='Sensor feedback')
    axes[0].plot(time, [r['target_N'] for r in rows], '--', label='Ramped target')
    axes[0].set_ylabel('Contact force (N)')
    axes[0].legend()
    axes[1].plot(time, [r['tcp_height_mm'] for r in rows])
    axes[1].set_ylabel('Sphere surface height (mm)')
    axes[1].set_xlabel('Simulation time (s)')
    fig.suptitle('Controller component test: sphere / plane, not dental path validation')
    fig.savefig(output/'benchmark_force.png', dpi=150)
    plt.close(fig)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--target-force-n', type=float, default=1.)
    args = parser.parse_args(argv)
    report = run(args.out, args.target_force_n)
    print(report)
    return 0 if report['controller_benchmark_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
