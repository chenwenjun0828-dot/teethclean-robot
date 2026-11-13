"""Synthetic mesh/prism seam regression; not actual dental/tool acceptance."""
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

from planning.infrastructure.io import dump
from .compliance import ForceControlConfig,NormalAdmittance,SensorContactForce,feedback_force,smoothstep
from .contact_experiment import write_csv,contacts
from .force_guard import PredictiveContactGuard
from .local_surface import generate


def run(output):
    output=Path(output)
    if output.exists() and any(output.iterdir()): raise ValueError('Use a new empty output directory')
    output.mkdir(parents=True,exist_ok=True)
    v=np.array([[-2.,-2,0],[2,-2,0],[2,2,0],[-2,2,0]])
    f=np.array([[0,1,2],[0,2,3]])
    # Deliberately separate the two adjacent prisms, to cross their shared edge.
    parts=generate(v,f,np.array([1,2]),[0,1],initial_scale_mm=10.)
    from .benchmark import MODEL
    spec=mujoco.MjSpec.from_string(MODEL)
    plane=spec.geom('dental_mandibular_visual'); plane.contype=plane.conaffinity=0
    sphere=spec.geom('ur_end_scaler_geom'); sphere.contype=sphere.conaffinity=0
    # A mesh cube is used only in this synthetic regression fixture; production
    # assets always load the actual end_scaler STL, never this test shape.
    corners=np.array([[x,y,z] for x in (-.0001,.0001) for y in (-.0001,.0001) for z in (-.0001,.0001)])
    from scipy.spatial import ConvexHull
    spec.add_mesh(name='test_probe',uservert=corners.ravel().tolist(),userface=ConvexHull(corners).simplices.ravel().tolist())
    spec.body('ur_tool').add_geom(name='ur_force_tool_0000',type=mujoco.mjtGeom.mjGEOM_MESH,
        meshname='test_probe',mass=0.,contype=2,conaffinity=4,solref=[.02,1.],solimp=[.9,.95,.001,.5,2.])
    spec.site('ur_scaler_tcp').pos=[0.,0.,-.0001]
    for i,p in enumerate(parts):
        name=f'prism_{i}'
        spec.add_mesh(name=name,uservert=(p['vertices']*.001).ravel().tolist(),userface=p['faces'].ravel().tolist())
        spec.worldbody.add_geom(name=f'force_mandibular_tooth_01_{i}',type=mujoco.mjtGeom.mjGEOM_MESH,
            meshname=name,contype=4,conaffinity=2,solref=[.02,1.],solimp=[.9,.95,.001,.5,2.])
    model=spec.compile(); data=mujoco.MjData(model)
    robot=SimpleNamespace(site=model.site('ur_scaler_tcp').id,qadr=np.array([0]))
    cfg=ForceControlConfig(target_force_N=.5)
    observer,controller=SensorContactForce(model),NormalAdmittance(cfg)
    guard=PredictiveContactGuard(model,robot,'mandibular',observer,2.5,[1])
    body=model.body('ur_carrier').id
    rows=[]; reason=''
    for index in range(11001):
        # Prescribed slow tangential translation only in this component model.
        time=float(data.time)
        x=-.0005+.001*smoothstep(np.clip((time-6.)/4.,0.,1.))
        model.body_pos[body,0]=x
        mujoco.mj_forward(model,data)
        measurement=observer.read(data); force=feedback_force(measurement,[0.,0.,1.])
        target=.5*smoothstep(time/.4)
        controller.step(force,target,.001)
        hits,bad=contacts(model,data,robot,'mandibular',[1])
        rows.append(dict(time_s=time,x_mm=x*1000,normal_force_N=force,contact_count=len(hits)))
        if bad or force>=2.5 or any(w.number for w in data.warning): reason='Contact/force/dynamics protection'; break
        if index==11000: break
        command=np.array([controller.displacement+data.qfrc_bias[0]/1000])
        prediction=guard.check(data,command,'acquire',normal_world=np.array([0.,0.,1.]))
        if not prediction['safe']: reason=prediction['reason']; break
        data.ctrl[:]=command; mujoco.mj_step(model,data)
    steady=np.array([r['normal_force_N'] for r in rows if r['time_s']>=5.])
    report=dict(scope='Synthetic flat mesh with adjacent 0.5 mm prisms and mesh probe only',
        actual_dental_or_tool_validation=False,simulation_only=True,force_control=asdict(cfg),
        stop_reason=reason,samples=len(rows),peak_force_N=max(r['normal_force_N'] for r in rows),
        steady_rmse_N=float(np.sqrt(np.mean((steady-.5)**2))) if len(steady) else None,
        steady_min_N=float(steady.min()) if len(steady) else None,
        steady_max_N=float(steady.max()) if len(steady) else None,
        max_simultaneous_contacts=max(r['contact_count'] for r in rows))
    report['passed']=bool(not reason and len(steady)>=5000 and np.all(abs(steady-.5)<=.1))
    write_csv(output/'seam_force.csv',rows); dump(output/'seam_report.json',report)
    return report


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path,required=True)
    result=run(parser.parse_args().out); print(result)
    raise SystemExit(0 if result['passed'] else 2)
