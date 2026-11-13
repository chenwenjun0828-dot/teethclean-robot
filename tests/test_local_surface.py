"""Local contact topology, original-surface evidence and stage interlocks."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from control.force import local_surface as surface
from control.force.local_checks import validate_evidence
from control.force.scan_guard import OriginalScanGuard
from control.force.stages import main as stages


def square(x=0., z=0., label=1):
    v=np.array([[x,0,z],[x+1,0,z],[x+1,1,z+.02],[x,1,z]])
    return v,np.array([[0,1,2],[0,2,3]]),np.full(2,label)


class LocalSurfaceTests(unittest.TestCase):
    def test_open_nonplanar_surface_is_extruded_without_capping(self):
        v,f,l=square(); original=v.copy()
        patches=surface.generate(v,f,l,np.arange(2),initial_scale_mm=2.)
        self.assertTrue(patches)
        for p in patches:
            n=len(p['original_vertex_ids'])
            np.testing.assert_array_equal(p['vertices'][:n],v[p['original_vertex_ids']])
            np.testing.assert_allclose(p['vertices'][n:]-p['vertices'][:n],np.tile(-.5*p['normal'],(n,1)))
            self.assertLessEqual(p['maximum_error_mm'],.05)
            self.assertIn('working',p['face_kinds']); self.assertIn('bottom',p['face_kinds'])
        np.testing.assert_array_equal(v,original)

    def test_disconnected_gap_and_tooth_labels_are_never_merged(self):
        v,f,l=square(); w,g,k=square(1.01,label=2)
        vertices=np.r_[v,w]; faces=np.r_[f,g+len(v)]
        for labels in (np.r_[l,k],np.ones(4,int)):
            patches=surface.generate(vertices,faces,labels,np.arange(4),initial_scale_mm=10.)
            for p in patches:
                ids=p['source_face_ids']
                self.assertTrue(np.all(ids<2) or np.all(ids>=2))
        # Connected triangles sharing an edge still cannot cross a tooth label.
        patches=surface.generate(v,f,np.array([1,2]),np.arange(2),initial_scale_mm=10.)
        self.assertEqual(len(patches),2)

    def test_concave_growth_does_not_fill_a_missing_corner(self):
        # A connected L footprint must not become one convex solid.
        v=np.array([[0,0,0],[.2,0,0],[.2,.1,0],[.1,.1,0],[.1,.2,0],[0,.2,0]])
        f=np.array([[0,1,3],[1,2,3],[0,3,5],[3,4,5]])
        patches=surface.generate(v,f,np.ones(4,int),np.arange(4),initial_scale_mm=1.)
        self.assertGreater(len(patches),1)
        for p in patches: self.assertLess(p['reverse_error_mm'],1e-5)

    def test_first_contact_checks_oblique_rays_and_rejects_shift(self):
        v,f,l=square(); patches=surface.generate(v,f,l,np.arange(2),initial_scale_mm=2.)
        check,_=surface.union_check(v,f,patches,np.arange(2))
        self.assertTrue(check['geometry_passed']); self.assertEqual(check['samples'],48)
        for p in patches: p['vertices']+=np.array([0,0,.2])
        check,_=surface.union_check(v,f,patches,np.arange(2))
        self.assertFalse(check['geometry_passed'])

    def test_working_distance_excludes_internal_bottom(self):
        v,f,l=square(); p=surface.convex_patch(v,f,np.arange(2))
        bottom=p['faces'][p['face_kinds']=='bottom']
        centers=p['vertices'][bottom].mean(1)
        distance=surface.distance_to(surface.scene(v,f),centers)
        self.assertGreater(distance.min(),.49)

    def test_thickening_a_thin_tool_wall_cannot_pass_as_original_tip(self):
        import open3d as o3d
        thin=o3d.geometry.TriangleMesh.create_box(1.,1.,.2)
        v,f=np.asarray(thin.vertices),np.asarray(thin.triangles)
        parts=surface.generate(v,f,np.ones(len(f),int),np.arange(len(f)))
        check,records=surface.union_check(v,f,parts,np.arange(len(f)))
        self.assertFalse(check['geometry_passed'])
        self.assertTrue(any(r['hit_kind']=='bottom' for r in records))

    def test_minimum_scale_failure_has_original_triangle_evidence(self):
        v=np.array([[0.,0.,0.],[.2,0.,0.],[.2,.2,0.],[0.,.2,0.],[.1,.1,-.03]])
        f=np.array([[0,1,4],[1,2,4],[2,3,4],[3,0,4]]); l=np.ones(4,int)
        with self.assertRaises(surface.SurfaceGeometryError) as caught:
            surface.generate(v,f,l,np.arange(4),initial_scale_mm=2.,minimum_scale_mm=2.,tolerance_mm=.0001)
        self.assertEqual(caught.exception.scale,2.)
        self.assertTrue(len(caught.exception.patch['source_face_ids']))

    def test_boundary_coverage_failure_is_not_a_surface_pass(self):
        v,f,l=square(); recipe=dict(normal_correction_mm=3.,tool_radius_mm=3.,boundary_margin_mm=1.,
            thickness_mm=.5,initial_patch_scale_mm=1.,minimum_patch_scale_mm=.25,patch_tolerance_mm=.05,
            final_tolerance_mm=.1,nominal_standoff_mm=1.8)
        z=dict(vertices=v,faces=f,labels=l,work_points=np.array([[0,0,0]]),band_ids=np.array([0]))
        with self.assertRaisesRegex(ValueError,'boundary'):
            validate_evidence(dict(recipe=recipe),z)

    def test_internal_bottom_contact_stops_before_crossing(self):
        # Exercise the runtime facet classifier at the actual prism bottom.
        v,f,l=square(); p=surface.convex_patch(v,f,np.arange(2))
        bottom=p['vertices'][p['faces'][p['face_kinds']=='bottom'][0]].mean(0)
        guard=OriginalScanGuard.__new__(OriginalScanGuard)
        guard.robot=SimpleNamespace(qadr=np.array([0]),site=0)
        guard.jaw='mandibular'; guard.info={'band_radius_mm':8.8}
        guard.band=SimpleNamespace(query=lambda point:(0.,0))
        guard.cache={}; guard._original=lambda q:dict(safe=True)
        guard.parts={'ur_force_tool_0000':dict(body='tool',role='tool',face_kinds=p['face_kinds'])}
        guard.scenes={'ur_force_tool_0000':surface.scene(p['vertices'],p['faces'])}
        guard.model=SimpleNamespace(body=lambda name:SimpleNamespace(id=0),
            geom=lambda i:SimpleNamespace(name='ur_force_tool_0000' if i==0 else 'other'))
        data=SimpleNamespace(qpos=np.array([0.]),site_xpos=np.zeros((1,3)),xpos=np.zeros((1,3)),
            xmat=np.eye(3).reshape(1,9),contact=[SimpleNamespace(geom=[0,1],dist=0.,pos=bottom/1000)])
        result=guard.assess(data,'trace')
        self.assertFalse(result['safe']); self.assertIn('bottom',result['reason'])

    def test_sequence_does_not_run_short_or_region_after_failed_hold(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); manifest=root/'manifest.json'; manifest.write_text('{}')
            config=root/'params.json'; config.write_text(json.dumps(dict(force_component='normal',contact_margin_mm=0,
                contact_model=str(manifest))))
            def fail(args):
                out=Path(args[args.index('--out')+1]); out.mkdir()
                (out/'force_trajectory_report.json').write_text(json.dumps(dict(stage_passed=False,stop_reason='geometry')))
                return 2
            with patch('control.force.stages.run_stage',side_effect=fail) as run:
                self.assertEqual(stages(['--params',str(config),'--out',str(root/'result')]),2)
            self.assertEqual(run.call_count,1)
            report=json.loads((root/'result/stages_report.json').read_text())
            self.assertEqual([s['status'] for s in report['stages']],['failed','not_run','not_run'])

    def test_mesh_probe_crosses_synthetic_prism_seam_without_force_spike(self):
        from control.force.local_benchmark import run
        with tempfile.TemporaryDirectory() as directory:
            report=run(Path(directory)/'seam')
        self.assertTrue(report['passed'],report)
        self.assertLess(report['steady_rmse_N'],.1)
        self.assertGreater(report['max_simultaneous_contacts'],1)

    def test_direct_regional_and_short_cannot_bypass_previous_stage(self):
        from control.force.stage_gate import verify_previous
        from control.force.compliance import ForceControlConfig
        for stage in ('short','regional'):
            with self.assertRaisesRegex(ValueError,'previous-stage'):
                verify_previous(stage,Path('.'),Path('manifest.json'),None,ForceControlConfig())

    def test_changed_original_surface_evidence_cannot_load(self):
        from control.force.contact_model import validate_local_assets
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory); (folder/'source_geometry.npz').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'source geometry changed'):
                validate_local_assets(folder/'manifest.json',dict(source_geometry_sha256='previous'))

    def test_local_loading_preserves_inertia_and_six_axis_sensor_frame(self):
        import open3d as o3d
        from control.force.contact_model import load_contact_model
        from pipeline.simulation import create_simulation,DENTAL_MODEL_PATH
        from planning.infrastructure.io import fingerprint
        base,data=create_simulation()
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)
            for file in ('trajectory.npz','planning_report.json'): (folder/file).write_text(file)
            part_file=folder/'probe.ply'; o3d.io.write_triangle_mesh(str(part_file),o3d.geometry.TriangleMesh.create_box(1,1,1))
            info=dict(schema_version=1,method='local_surface',status='verified',units='mm',selected_teeth=[1],jaw='mandibular',
                checks=[dict(label=k,samples=1,geometry_passed=True,maximum_error_mm=0.) for k in ('tooth_01','tool')],
                parts=[dict(file='probe.ply',sha256=fingerprint(part_file)['sha256'],body='ur_end_scaler_link',role='tool',tooth=None)],
                scene_sha256=fingerprint(DENTAL_MODEL_PATH)['sha256'],
                source_trajectory_sha256=fingerprint(folder/'trajectory.npz')['sha256'],
                source_plan_sha256=fingerprint(folder/'planning_report.json')['sha256'],coverage='Synthetic fixture')
            manifest=folder/'manifest.json'; manifest.write_text(json.dumps(info))
            # Geometry gates have their own tests; here isolate model injection.
            with patch('control.force.contact_model.validate_local_assets'):
                loaded,_,_=load_contact_model(manifest,folder,base,data)
            for field in ('body_mass','body_inertia','body_ipos','body_iquat','site_pos','site_quat','sensor_objid','sensor_adr'):
                np.testing.assert_array_equal(getattr(loaded,field),getattr(base,field))
            self.assertEqual(loaded.geom('ur_end_scaler_geom').contype,0)
            self.assertEqual(loaded.geom('dental_mandibular_visual').contype,0)
            self.assertEqual(loaded.geom('ur_force_tool_0000').contype,2)


if __name__=='__main__': unittest.main()
