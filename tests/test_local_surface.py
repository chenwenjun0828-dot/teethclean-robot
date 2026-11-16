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
    def test_single_point_envelope_excludes_only_unreachable_failures(self):
        from control.force.hold_envelope import distance_mm,reachable_failures
        envelope=dict(centers_mm=np.array([[0.,0.,0.]]),radii_mm=np.array([.1]),
                      normal_scan=np.array([0.,0.,1.]))
        records=[dict(x_mm=x,y_mm=0.,z_mm=0.,geometry_passed=False,probe_valid=True)
                 for x in (0.,1.,3.)]
        reachable,distances=reachable_failures(records,envelope)
        self.assertEqual(len(reachable),2)
        self.assertLess(distances[1],.1)
        self.assertGreater(distances[2],.1)
        self.assertLess(distance_mm([[0.,0.,3.]],envelope)[0],0.)

    def test_filtered_real_tool_tip_pair_preserves_nearby_original_surface(self):
        from planning.infrastructure.io import ROOT
        import open3d as o3d
        from pipeline.simulation import DENTAL_MODEL_PATH
        mesh=o3d.io.read_triangle_mesh(str(ROOT/'assets/simulation/robot/assets/end_scaler.stl'))
        mesh.remove_duplicated_vertices()
        spec=mujoco.MjSpec.from_file(str(DENTAL_MODEL_PATH));spec.compile()
        geom=spec.geom('ur_end_scaler_geom'); rot=np.empty(9);mujoco.mju_quat2Mat(rot,geom.quat)
        v=np.asarray(mesh.vertices)@rot.reshape(3,3).T+np.asarray(geom.pos)*1000
        f=np.asarray(mesh.triangles);tip=np.asarray(spec.site('ur_scaler_tcp').pos)*1000
        filtered,removed,error=surface.filtered_tool_faces(v,f,tip)
        np.testing.assert_array_equal(removed,[2392,2660,2889])
        self.assertEqual(len(filtered),len(f)-3)
        self.assertLess(error,.002)
        with self.assertRaisesRegex(ValueError,'pair changed'):
            surface.filtered_tool_faces(v,f[1:],tip)

    def test_mixed_winding_closed_mesh_has_consistent_outward_normals(self):
        import open3d as o3d
        box=o3d.geometry.TriangleMesh.create_box(1.,1.,.1)
        v,f=np.asarray(box.vertices),np.asarray(box.triangles).copy()
        expected=surface.outward_normals(v,f,np.arange(len(f)))
        f[::3]=f[::3,::-1]
        np.testing.assert_allclose(surface.outward_normals(v,f,np.arange(len(f))),expected)

    def test_open_surface_orientation_does_not_change_under_translation(self):
        v,f,_=square()
        expected=surface.outward_normals(v,f,np.arange(len(f)))
        np.testing.assert_allclose(surface.outward_normals(v+[10.,-30.,-500.],f,np.arange(len(f))),expected)

    def test_opposite_duplicate_faces_cannot_define_an_inward_extrusion(self):
        v=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]])
        f=np.array([[0,1,2],[0,2,1]])
        with self.assertRaises(surface.SurfaceTopologyError) as caught:
            surface.generate(v,f,np.zeros(2,int),np.arange(2),flush=False)
        self.assertEqual(set(caught.exception.source_face_ids),{0,1})

    def test_production_tool_recipe_limits_depth_and_can_be_rebuilt(self):
        import open3d as o3d
        mesh=o3d.geometry.TriangleMesh.create_box(1.,1.,.01)
        v,f=np.asarray(mesh.vertices),np.asarray(mesh.triangles)
        labels=np.zeros(len(f),int); ids=np.arange(len(f))
        prepared=surface.extrusion_fields(v,f,labels,ids,flush=False)
        parts=surface.generate(v,f,labels,ids,flush=False,prepared=prepared)
        for p in parts:
            self.assertLessEqual(p['thickness_mm'],.004001)
            rebuilt=surface.convex_patch(v,f,p['source_face_ids'],.05,
                outward=prepared['normals'][p['source_face_ids']],
                field=prepared['field'],depth_field=prepared['depth_field'])
            np.testing.assert_array_equal(p['vertices'],rebuilt['vertices'])
            np.testing.assert_array_equal(p['faces'],rebuilt['faces'])
        check,_=surface.union_check(v,f,parts,ids,source_normals=prepared['normals'])
        self.assertTrue(check['geometry_passed'],check)

    def test_near_surface_requires_oriented_contact_normal(self):
        v=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]])
        f=np.array([[0,1,2]])
        points=np.array([[.2,.2,-.03]]*3+[[.2,.2,.2]])
        normals=np.array([[0.,0.,1.],[1.,0.,0.],[0.,0.,-1.],[0.,0.,1.]])
        metric=surface.contact_metrics(surface.scene(v,f),points,normals,np.array([[0.,0.,1.]]))
        np.testing.assert_array_equal(metric['geometry_passed'],[True,True,True,False])
        self.assertAlmostEqual(metric['signed_distance_mm'][0],-.03,places=6)

    def test_no_valid_probes_cannot_pass_the_gate(self):
        v,f,l=square(); parts=surface.generate(v,f,l,np.arange(2))
        check,_=surface.union_check(v,f,parts,np.arange(2),band_ids=[])
        self.assertEqual(check['valid_samples'],0)
        self.assertFalse(check['geometry_passed'])
        self.assertEqual(check['uncovered_normal_faces'],[0,1])
        check,_=surface.union_check(v,f,parts,np.array([],int))
        self.assertFalse(check['geometry_passed'])

    def test_runtime_near_side_contact_handles_body_rotation_and_geom_order(self):
        v,f,_=square(); p=surface.convex_patch(v,f,np.arange(2))
        normal=surface.outward_normals(v,f,np.arange(2))[0]
        a=.7; rotation=np.array([[1.,0.,0.],[0.,np.cos(a),-np.sin(a)],[0.,np.sin(a),np.cos(a)]])
        position=np.array([1.,2.,3.]); point=np.array([.6,.3,-.025])
        for order in (0,1):
            guard=OriginalScanGuard.__new__(OriginalScanGuard)
            guard.robot=SimpleNamespace(qadr=np.array([0]),site=0)
            guard.jaw='mandibular'; guard.info={'band_radius_mm':8.8}
            guard.band=SimpleNamespace(query=lambda point:(0.,0)); guard.cache={}
            guard._original=lambda q:dict(safe=True)
            guard.parts={'ur_force_tool_0000':dict(body='tool',role='tool',
                face_kinds=np.array(['side']*len(p['faces'])),source_face_ids=[0,1])}
            guard.scenes={'ur_force_tool_0000':surface.scene(p['vertices'],p['faces'])}
            guard.references={'tool':surface.scene(v,f)}
            guard.reference_normals={'tool':surface.outward_normals(v,f,np.arange(2))}
            guard.triangles={'tool':v[f]}; guard.tip_mm=np.zeros(3)
            guard.model=SimpleNamespace(body=lambda name:SimpleNamespace(id=0),
                geom=lambda i:SimpleNamespace(name='ur_force_tool_0000' if i==order else 'other'))
            world_normal=rotation@normal*(1 if order==0 else -1)
            contact=SimpleNamespace(geom=[0,1],dist=-.00003,
                pos=position+rotation@point/1000,frame=np.r_[world_normal,np.zeros(6)])
            data=SimpleNamespace(qpos=np.array([0.]),site_xpos=np.zeros((1,3)),
                xpos=position[None],xmat=rotation.reshape(1,9),contact=[contact])
            result=guard.assess(data,'trace')
            self.assertTrue(result['safe'],result)
            self.assertEqual(result['contacts'][0]['hit_kind'],'side')
            contact.frame[:3]*=-1
            self.assertTrue(guard.assess(data,'trace')['safe'])
            contact.frame[:3]*=-1; contact.dist=-.00011
            self.assertFalse(guard.assess(data,'trace')['safe'])

    def test_float64_reference_preserves_skinny_triangle_contact_normal(self):
        tri=np.array([[[0.,1.,0.],[1.,1.,.01],[1.,1.,0.]]])
        point=np.array([[.503333333,1.,.0000666667]])
        nearest,ids=surface.closest_on_triangles(point,tri)
        np.testing.assert_allclose(nearest,point,atol=1e-12)
        metric=surface.contact_metrics(None,point,[[0.,1.,0.]],[[0.,1.,0.]],reference=(nearest,ids))
        self.assertTrue(metric['geometry_passed'][0])

    def test_boundary_probe_rejects_new_gap_fill_even_when_near_original_edge(self):
        v=np.array([[0.,0.,0.],[1.,0.,0.],[1.,1.,0.],[0.,1.,0.]])
        f=np.array([[0,1,2],[0,2,3]])
        expanded=(v-[.5,.5,0.])*1.2+[.5,.5,0.]
        check,_=surface.gap_check(v,f,np.ones(2,int),[self._layer_of(expanded,f)],np.arange(2))
        self.assertFalse(check['geometry_passed'])

    def test_later_original_surface_is_not_a_valid_first_hit(self):
        v,f,l=square(); w,g,_=square(z=-.3)
        v=np.r_[v,w]; f=np.r_[f,g+4]
        check,records=surface.union_check(v,f,[self._layer_of(w,g)],np.arange(2))
        self.assertFalse(check['geometry_passed'])
        self.assertTrue(any(r['probe_valid'] and r['error_mm'] is not None
                            and r['error_mm']>.1 for r in records))

    def test_occluded_distant_probes_are_retried_closer_to_the_sample(self):
        v,f,_=square(); w,g,_=square(z=1.)
        vertices=np.r_[v,w]; faces=np.r_[f,g+4]
        check,records=surface.union_check(vertices,faces,[self._layer_of(v,f)],np.arange(2))
        self.assertTrue(check['geometry_passed'],check)
        normal_probes=[r for r in records if r['direction']==0 and r['sample_kind']=='face']
        self.assertTrue(all(r['probe_valid'] and r['standoff_mm']<1. for r in normal_probes))

    def test_source_plan_path_survives_provenance_iteration(self):
        from planning.infrastructure.io import fingerprint
        from control.force.contact_model import validate_local_assets
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); (root/'planning_report.json').write_text(json.dumps(dict(config=dict(standoff_mm=1.8))))
            np.savez(root/'source_geometry.npz',work_points=np.zeros((1,3)),work_normals=np.array([[0.,0.,1.]]))
            ref=SimpleNamespace(samples=lambda spacing:[0.],evaluate=lambda s:np.zeros(3),normal=lambda s:np.array([0.,0.,1.]))
            info=dict(source_geometry_sha256=fingerprint(root/'source_geometry.npz')['sha256'],
                source_inputs=[dict(file=str(root/'planning_report.json'),sha256=fingerprint(root/'planning_report.json')['sha256'])]*3,
                contact_policy=dict(surface.CONTACT_POLICY),recipe=dict(nominal_standoff_mm=1.8))
            with patch('control.force.local_checks.validate_evidence_sources'), \
                 patch('planning.motion_planning.contract.load_sweep_reference',return_value=ref) as load, \
                 patch('control.force.local_checks.validate_evidence',side_effect=ValueError('reached coverage')):
                with self.assertRaisesRegex(ValueError,'reached coverage'):
                    validate_local_assets(root/'manifest.json',info,root)
            self.assertEqual(load.call_args.args[0],root)

    def test_open_nonplanar_surface_is_extruded_without_capping(self):
        v,f,l=square(); original=v.copy()
        patches=surface.generate(v,f,l,np.arange(2),initial_scale_mm=2.)
        self.assertTrue(patches)
        for p in patches:
            n=len(p['original_vertex_ids'])
            np.testing.assert_array_equal(p['vertices'][:n],v[p['original_vertex_ids']])
            # The extrusion is strictly inward, by at most the recipe depth.
            delta=p['vertices'][n:]-p['vertices'][:n]
            self.assertLessEqual(float(np.linalg.norm(delta,axis=1).max()),.05+1e-9)
            # The extrusion goes strictly inward, opposite the outward normal.
            self.assertTrue(np.all(delta@p['normal']<0.))
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
        self.assertGreater(distance.min(),.049)

    def test_thickening_a_thin_tool_wall_cannot_pass_as_original_tip(self):
        # A thin shell must never be thickened past its own wall: the
        # extrusion would break through the far side and the layer would
        # then stick out of the tool, which is not the original tip.
        import open3d as o3d
        thin=o3d.geometry.TriangleMesh.create_box(1.,1.,.2)
        v,f=np.asarray(thin.vertices),np.asarray(thin.triangles)
        labels=np.ones(len(f),int)
        wall=surface.local_wall_thickness(v,f,np.arange(len(f)),limit_mm=.5)
        self.assertLess(wall.min(),.2+1e-9)
        parts=surface.generate(v,f,labels,np.arange(len(f)),thickness_mm=.5,
                               wall_thickness_mm=wall)
        for p in parts:
            self.assertLessEqual(p['thickness_mm'],wall.min()+1e-9)
        # The wall-limited layer must not break through the far side: from
        # every bottom point, the inward ray still finds material ahead.
        whole=surface.scene(v,f)
        for p in parts:
            n=len(p['original_vertex_ids'])
            bottom=p['vertices'][n:]
            direction=-p['normal']
            rays=o3d.core.Tensor(np.c_[bottom,np.tile(direction,(len(bottom),1))].astype(np.float32))
            hit=whole.cast_rays(rays)['t_hit'].numpy()
            self.assertTrue(np.all(np.isfinite(hit)),hit)
            self.assertTrue(np.all(hit>1e-6),hit)

    def _layer_of(self, vertices, faces, kind='working'):
        """A contact layer that is exactly the given mesh."""
        return dict(vertices=np.asarray(vertices,float).copy(),
                    faces=np.asarray(faces,int).copy(),
                    face_kinds=np.array([kind]*len(faces)))

    def test_a_zero_extrusion_layer_never_fails_the_gate(self):
        # The harness must not manufacture failures: a layer that IS the
        # original surface has zero deviation by construction, so every
        # probe must pass.  This is the self-check of the measurement code.
        v,f,l=square()
        check,records=surface.union_check(v,f,[self._layer_of(v,f)],np.arange(2))
        self.assertTrue(check['geometry_passed'],check)
        self.assertEqual(check['failed_samples'],0)
        self.assertGreater(check['valid_samples'],0)

    def test_a_full_original_scan_layer_never_fails_the_gate(self):
        # The same self-check with the whole original mesh as the layer.
        v,f,l=square()
        check,records=surface.union_check(v,f,[self._layer_of(v,f)],np.arange(2))
        self.assertTrue(check['geometry_passed'],check)
        self.assertEqual(check['failed_samples'],0)

    def test_a_zero_depth_prism_still_exposes_its_working_face(self):
        # A degenerate extrusion has no volume, so the hull of the top and
        # bottom points collapses.  The patch must still report a working
        # face rather than claim the patch has no surface.
        v,f,l=square()
        patch=surface.convex_patch(v,f,np.arange(2),thickness_mm=0.)
        self.assertIn('working',patch['face_kinds'])

    def test_a_probe_starting_inside_the_solid_is_not_a_geometry_failure(self):
        # A concave surface has an inward pocket: a ray that starts there
        # measures nothing, so it must be counted separately, not failed.
        v=np.array([[0.,0.,0.],[2.,0.,0.],[2.,2.,0.],[1.,1.,-1.],[0.,2.,0.]])
        f=np.array([[0,1,3],[1,2,3],[2,4,3],[4,0,3]]); l=np.ones(4,int)
        patches=surface.generate(v,f,l,np.arange(4),initial_scale_mm=10.)
        check,records=surface.union_check(v,f,patches,np.arange(4))
        for r in records:
            if not r['probe_valid']:
                self.assertIn(r['probe_invalid_reason'],
                              ('origin_behind_surface','reference_outside_work_band',
                               'occluded_before_sample','missing_reference','approach_into_surface'))
            else:
                self.assertTrue(r['geometry_passed'])

    def test_shared_edge_patches_extrude_to_the_same_bottom_points(self):
        # The flush seam: two patches that share an edge must move that
        # edge's vertices to the SAME places, so their side walls are
        # coincident and no groove exists between them.
        v=np.array([[0.,0.,0.],[1.,0.,0.],[1.,1.,0.],[0.,1.,0.],[.5,.5,-.4]])
        f=np.array([[0,1,4],[1,2,4],[2,3,4],[3,0,4]]); l=np.ones(4,int)
        patches=surface.generate(v,f,l,np.arange(4),initial_scale_mm=10.)
        self.assertGreater(len(patches),1)
        bottoms={}
        for p in patches:
            n=len(p['original_vertex_ids'])
            for vertex,point in zip(p['original_vertex_ids'],p['vertices'][n:]):
                bottoms.setdefault(int(vertex),set()).add(tuple(np.round(point,9)))
        shared=[k for k,v in bottoms.items() if len(v)>1]
        self.assertEqual(shared,[],'a shared vertex was extruded to two different places')

    def test_adjacent_patches_first_hit_is_the_working_face(self):
        # Along the approach direction, the first hit at a seam must be the
        # working surface, not a neighbouring patch's internal wall.
        v=np.array([[0.,0.,0.],[1.,0.,0.],[1.,1.,0.],[0.,1.,0.],[.5,.5,-.4]])
        f=np.array([[0,1,4],[1,2,4],[2,3,4],[3,0,4]]); l=np.ones(4,int)
        patches=surface.generate(v,f,l,np.arange(4),initial_scale_mm=10.)
        check,records=surface.union_check(v,f,patches,np.arange(4))
        valid=[r for r in records if r['probe_valid']]
        self.assertTrue(valid)
        for r in valid:
            self.assertEqual(r['hit_kind'],'working',r)
            self.assertLessEqual(r['error_mm'],.1)

    def test_the_tool_wall_limits_the_extrusion_depth(self):
        # A thin shell must not be thickened past its own wall.
        import open3d as o3d
        thin=o3d.geometry.TriangleMesh.create_box(1.,1.,.2)
        v,f=np.asarray(thin.vertices),np.asarray(thin.triangles)
        labels=np.ones(len(f),int)
        wall=surface.local_wall_thickness(v,f,np.arange(len(f)),limit_mm=.5)
        self.assertLess(wall.min(),.2)
        parts=surface.generate(v,f,labels,np.arange(len(f)),thickness_mm=.05,
                               wall_thickness_mm=wall)
        for p in parts:
            self.assertLessEqual(p['thickness_mm'],.2+1e-9)
        check,records=surface.union_check(v,f,parts,np.arange(len(f)))
        self.assertTrue(check['geometry_passed'],check)

    def test_minimum_scale_failure_has_original_triangle_evidence(self):
        # A saddle cannot be covered by a convex cap, so the patch must be
        # rejected and the rejection must carry the original triangles.
        xs=np.linspace(0.,1.,3)
        vertices=np.array([[x,y,x*y] for x in xs for y in xs])
        faces=[]
        for i in range(2):
            for j in range(2):
                a=i*3+j; b=i*3+j+1; c=(i+1)*3+j+1; d=(i+1)*3+j
                faces+=[[a,b,c],[a,c,d]]
        faces=np.array(faces); labels=np.ones(len(faces),int)
        with self.assertRaises(surface.SurfaceGeometryError) as caught:
            surface.generate(vertices,faces,labels,np.arange(len(faces)),initial_scale_mm=2.,
                             minimum_scale_mm=2.,tolerance_mm=.0001)
        self.assertEqual(caught.exception.scale,2.)
        self.assertTrue(len(caught.exception.patch['source_face_ids']))

    def test_boundary_coverage_failure_is_not_a_surface_pass(self):
        v,f,l=square(); recipe=dict(normal_correction_mm=3.,tool_radius_mm=3.,boundary_margin_mm=1.,
            thickness_mm=.05,initial_patch_scale_mm=1.,minimum_patch_scale_mm=.25,patch_tolerance_mm=.05,
            final_tolerance_mm=.1,nominal_standoff_mm=1.8)
        z=dict(vertices=v,faces=f,labels=l,work_points=np.array([[0,0,0]]),band_ids=np.array([0]))
        with self.assertRaisesRegex(ValueError,'boundary'):
            validate_evidence(dict(recipe=recipe),z)

    def test_internal_bottom_contact_with_wrong_normal_remains_near_source(self):
        # Exercise the runtime facet classifier at the actual prism bottom.
        v,f,l=square(); p=surface.convex_patch(v,f,np.arange(2))
        bottom=p['vertices'][p['faces'][p['face_kinds']=='bottom'][0]].mean(0)
        guard=OriginalScanGuard.__new__(OriginalScanGuard)
        guard.robot=SimpleNamespace(qadr=np.array([0]),site=0)
        guard.jaw='mandibular'; guard.info={'band_radius_mm':8.8}
        guard.band=SimpleNamespace(query=lambda point:(0.,0))
        guard.cache={}; guard._original=lambda q:dict(safe=True)
        guard.parts={'ur_force_tool_0000':dict(body='tool',role='tool',face_kinds=p['face_kinds'],source_face_ids=[0,1])}
        guard.scenes={'ur_force_tool_0000':surface.scene(p['vertices'],p['faces'])}
        guard.references={'tool':surface.scene(v,f)}
        guard.reference_normals={'tool':surface.outward_normals(v,f,np.arange(len(f)))}
        guard.triangles={'tool':v[f]}
        guard.tip_mm=np.zeros(3)
        guard.model=SimpleNamespace(body=lambda name:SimpleNamespace(id=0),
            geom=lambda i:SimpleNamespace(name='ur_force_tool_0000' if i==0 else 'other'))
        data=SimpleNamespace(qpos=np.array([0.]),site_xpos=np.zeros((1,3)),xpos=np.zeros((1,3)),
            xmat=np.eye(3).reshape(1,9),contact=[SimpleNamespace(geom=[0,1],dist=0.,pos=bottom/1000,
                                                               frame=np.diag([1.,1.,-1.])[[2,1,0]].ravel())])
        result=guard.assess(data,'trace')
        self.assertTrue(result['safe'],result)

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
