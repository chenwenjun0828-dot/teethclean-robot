"""Normal-force semantics, independent signal audit and contact-asset gates."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import open3d as o3d

from control.force.compliance import feedback_force,force_components,ForceControlConfig
from control.force.contact_path import ContactPath,ContactPathConfig
from control.force.contact_assets import close_boundaries,first_hit_error,decompose_with_budget
from control.force.contact_model import load_contact_model
from control.force.motion_guard import MotionProgressGuard
from control.force.validation import signal_metrics
from control.force.runner import main
from tests.test_contact_path import example


class NormalForceTests(unittest.TestCase):
    def test_normal_is_signed_and_distinct_from_resultant(self):
        observation={'external_force_world_N':np.array([.6,0.,.8])}
        values=force_components(observation,[0,0,1])
        self.assertAlmostEqual(values['normal_force_N'],.8)
        self.assertAlmostEqual(values['tangential_force_N'],.6)
        self.assertAlmostEqual(feedback_force(observation,[0,0,1],'resultant'),1.)
        self.assertAlmostEqual(feedback_force(observation,[0,0,1]),.8)
        with self.assertRaisesRegex(ValueError,'wrong sign'):
            feedback_force(observation,[0,0,-1])
        # Rotate both force and surface normal: the controlled value is invariant.
        r=np.array([[0,0,1],[0,1,0],[-1,0,0.]])
        self.assertAlmostEqual(feedback_force({'external_force_world_N':r@observation['external_force_world_N']},r@np.array([0,0,1.])),.8)

    def test_pure_tangential_force_does_not_satisfy_normal_target(self):
        observation={'external_force_world_N':np.array([1.,0,0])}
        self.assertEqual(feedback_force(observation,[0,0,1]),0.)
        path=ContactPath(*example(),config=ContactPathConfig(stable_contact_s=.5))
        path.stage='acquire'
        for _ in range(1000):path.update(0.,0.,.001,total_force_N=1.)
        self.assertEqual(path.stage,'acquire')
        state=path.update(.5,0,.001,total_force_N=2.1)
        self.assertEqual(state['force_target_N'],0.)
        self.assertIn('hard limit',path.update(.5,0,.001,total_force_N=3.)['fault_reason'])

    def test_independent_audit_reprojects_logged_vectors(self):
        rows=[]
        for i in range(10):
            row=dict(time_s=i*.001,phase='trace',compensated_force_N=1.,applied_command_force_N=1.,
                     regulated_force_N=1.,applied_regulated_force_N=1.) # deliberately dishonest scalar columns
            for axis,value in zip('xyz',[1.,0,0]):
                row['external_force_world_'+axis+'_N']=value
                row['applied_external_force_world_'+axis+'_N']=value
                row['normal_world_'+axis]=1. if axis=='z' else 0.
            rows.append(row)
        metrics=signal_metrics(rows,1.,.2,.05,'normal')
        self.assertEqual(metrics['trace_mean_N'],0.)
        self.assertEqual(metrics['trace_contact_loss_s'],.01)
        self.assertEqual(metrics['peak_force_N'],1.)
        rows[0]['external_force_world_x_N']=3.
        with self.assertRaisesRegex(ValueError,'resultant'):
            signal_metrics(rows,1.,.2,.05,'normal')

    def test_no_pose_progress_is_detected(self):
        guard=MotionProgressGuard()
        guard.update(0.,-.001,0.)
        with self.assertRaisesRegex(RuntimeError,'no measured'):
            guard.update(0.,-.001,1.)
        guard=MotionProgressGuard()
        guard.update(0.,-.001,0.);guard.update(-.0001,-.001,1.)

    def test_capping_keeps_original_faces_and_geometry_gate_rejects_shift(self):
        cube=o3d.geometry.TriangleMesh.create_box(2.,2.,2.)
        vertices=np.asarray(cube.vertices).copy()
        faces=np.asarray(cube.triangles)
        keep=~np.all(vertices[faces][:,:,2]==0,axis=1)
        open_mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices),
                                          o3d.utility.Vector3iVector(faces[keep]))
        closed=close_boundaries(open_mesh)
        self.assertTrue(closed.is_watertight())
        check=first_hit_error(cube,[cube],np.array([[1.,1.,1.]]),5.)
        self.assertTrue(check['geometry_passed'])
        moved=o3d.geometry.TriangleMesh(cube).translate([.3,0,0])
        self.assertFalse(first_hit_error(cube,[moved],np.array([[1.,1.,1.]]),5.)['geometry_passed'])

    def test_failed_geometry_never_reaches_physics(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'manifest.json'
            path.write_text(json.dumps(dict(schema_version=1,status='failed_geometry_gate')))
            with self.assertRaisesRegex(ValueError,'geometry gate'):
                load_contact_model(path,Path(directory),None,None)

    def test_contact_model_keeps_robot_inertia_and_original_model_untouched(self):
        from pipeline.simulation import create_simulation,DENTAL_MODEL_PATH
        from planning.infrastructure.io import fingerprint
        base,data=create_simulation()
        before=base.geom_contype.copy()
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);source=folder/'source';source.mkdir()
            for name in ['trajectory.npz','planning_report.json']:(source/name).write_text(name)
            file=folder/'part.ply'
            o3d.io.write_triangle_mesh(str(file),o3d.geometry.TriangleMesh.create_box(1,1,1))
            part=dict(file='part.ply',sha256=fingerprint(file)['sha256'],body='ur_end_scaler_link',role='tool',tooth=None)
            record=dict(schema_version=1,status='verified',units='mm',selected_teeth=[1],jaw='mandibular',
                checks=[dict(label=label,samples=1,geometry_passed=True,maximum_error_mm=0.) for label in ['tooth_01','tool']],
                parts=[part],scene_sha256=fingerprint(DENTAL_MODEL_PATH)['sha256'],
                source_trajectory_sha256=fingerprint(source/'trajectory.npz')['sha256'],
                source_plan_sha256=fingerprint(source/'planning_report.json')['sha256'],
                material_status='uncalibrated',coverage='synthetic unit test')
            path=folder/'manifest.json';path.write_text(json.dumps(record))
            model,clone,info=load_contact_model(path,source,base,data)
            np.testing.assert_array_equal(base.geom_contype,before)
            np.testing.assert_allclose(model.body_mass,base.body_mass,atol=1e-12)
            np.testing.assert_allclose(model.body_inertia,base.body_inertia,atol=1e-12)
            self.assertEqual(model.nq,base.nq)
            self.assertEqual(model.geom('ur_end_scaler_geom').contype,0)
            self.assertEqual(info['material_status'],'uncalibrated')

    def test_native_geometry_worker_has_a_deadline(self):
        cube=o3d.geometry.TriangleMesh.create_box(2.,2.,2.)
        with self.assertRaisesRegex(TimeoutError,'exceeded'):
            decompose_with_budget(cube,.05,seconds=.001)

    def test_simple_solid_decomposition_passes_surface_gate(self):
        cube=o3d.geometry.TriangleMesh.create_box(2.,2.,2.)
        from control.force.contact_assets import mesh
        parts=decompose_with_budget(cube,.05,seconds=30.)
        report=first_hit_error(cube,[mesh(v,f) for v,f in parts],np.array([[1,1,1.]]),5.)
        self.assertTrue(report['geometry_passed'])

    def test_parameter_file_and_cli_override(self):
        with tempfile.TemporaryDirectory() as directory:
            file=Path(directory)/'params.json'
            file.write_text(json.dumps(dict(source='outputs/plan',force_component='normal',stage='hold',
                target_force_n=.5,force_tolerance_n=.1,contact_model='assets/contact/example.json')))
            report=dict(status='completed',force_control_passed=False,stage_passed=True,stop_reason='',
                recorded_duration_s=10.,retained_path_progress_fraction=0.,peak_compensated_force_N=.5,
                force_tracking_passed=True,scanned_surface_contact_passed=True)
            with patch('control.force.force_trajectory.run',return_value=report) as run,contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(['--params',str(file),'--stage','short']),0)
            args,kwargs=run.call_args
            self.assertEqual(args[2].force_component,'normal')
            self.assertEqual(kwargs['stage'],'short')
            self.assertTrue(kwargs['contact_model'].is_absolute())
