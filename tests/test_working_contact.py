"""Analytic whole-surface contact proofs and policy/loaded-model regressions."""
import json
from types import SimpleNamespace
import unittest

import fcl
import numpy as np

from planning.infrastructure.contact import ContactPolicy, HeightfieldContactSurface as PlanarContactSurface, partition_working_faces
from planning.infrastructure.collision import Check, CollisionChecker, bvh, mesh_from_geom


def plane(z=0., extent=.005):
    return (np.array([[-extent, -extent, z], [extent, -extent, z],
                      [extent, extent, z], [-extent, extent, z]]),
            np.array([[0, 1, 2], [0, 2, 3]]))


def tip(z=0.):
    return np.array([[[-.0003, -.0002, z], [.0003, -.0002, z], [0., .0003, z]]])


def plane_with_remote_fold():
    coordinates = np.linspace(-.005, .005, 5)
    vertices = [[x, y, 0.] for y in coordinates for x in coordinates]
    faces = []
    for row in range(4):
        for column in range(4):
            a = row*5+column
            faces.extend([[a, a+1, a+6], [a, a+6, a+5]])
    vertices.extend([[.0016, -.0002, -.0002], [.0016, .0002, -.0002], [.0016, 0., .0002]])
    faces.append([25, 26, 27])
    return np.asarray(vertices), np.asarray(faces)


def checker(policy=None, forbidden_z=.0005):
    """Actual FCL pair queries with deterministic analytic FK and meshes."""
    result = CollisionChecker.__new__(CollisionChecker)
    result.contact_policy = policy or ContactPolicy(.05)
    result.contact_manifest = {}
    result.calls = 0; result.cache = {}
    result.model = SimpleNamespace(geom=lambda i: SimpleNamespace(name=('ur_end_scaler_geom', 'ur_sensor')[i]),
                                   body=lambda i: SimpleNamespace(name='ur_tool'), geom_bodyid=[0, 0])
    data = SimpleNamespace(ncon=0, contact=[], geom_xmat=np.tile(np.eye(3).ravel(), (2, 1)),
                           geom_xpos=np.array([[0., 0., 0.], [0., 0., forbidden_z]]))
    def fk(q, full=False):
        data.geom_xpos[0, 2] = q[0]
    result.robot = SimpleNamespace(verify_data=data, fk=fk)
    result.working_geom = 0
    result.working_vertices = tip()[0]
    result.working_faces = np.array([[0, 1, 2]])
    half = .00002
    result.parts = [(1, 0, fcl.CollisionObject(fcl.Box(2*half, 2*half, 2*half)))]
    result.part_bounds = {1: np.array([[x, y, z] for x in (-half, half)
                                     for y in (-half, half) for z in (-half, half)])}
    result.teeth = []; result.tooth_bounds = {}; result.contact_surfaces = {}
    # Real checker retains these BVHs too: python-fcl objects need mesh lifetime.
    result.meshes = []
    for name, z in [('mandibular', 0.), ('maxillary', .1)]:
        vertices, faces = plane(z)
        shape = bvh(vertices, faces); result.meshes.append(shape)
        result.teeth.append((name, fcl.CollisionObject(shape)))
        result.tooth_bounds[name] = (vertices.min(axis=0), vertices.max(axis=0))
        result.contact_surfaces[name] = PlanarContactSurface(vertices, faces)
    return result


class ContactPolicyTests(unittest.TestCase):
    def test_parameters_reject_nonfinite_boolean_and_wrong_sign(self):
        for name in ('contact_tolerance_mm', 'working_segment_radius_mm', 'nonworking_clearance_mm'):
            for value in (True, np.bool_(False), float('nan'), float('inf'), -1., '0.05'):
                arguments = {'contact_tolerance_mm': .05, name: value}
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    ContactPolicy(**arguments)
        with self.assertRaises(ValueError):
            ContactPolicy(.05, 0.)
        self.assertEqual(ContactPolicy(0).nonworking_clearance_mm, 0.)

    def test_partition_exact_radius_mixed_faces_and_tcp_transform(self):
        tcp = np.eye(4); tcp[:3, 3] = [-.01, 0, 0]
        vertices = np.array([[.01, 0, 0], [.011, 0, 0], [.01, .0002, 0], [.012, 0, 0]])
        faces = np.array([[0, 1, 2], [1, 2, 3]])
        _, working, forbidden, mixed = partition_working_faces(vertices, faces, tcp, ContactPolicy(.05))
        np.testing.assert_array_equal(working, [0])
        np.testing.assert_array_equal(forbidden, [1])
        np.testing.assert_array_equal(mixed, [1])

    def test_empty_partition_fails_explicitly(self):
        with self.assertRaisesRegex(ValueError, 'empty'):
            partition_working_faces(tip()[0]+.01, [[0, 1, 2]], np.eye(4), ContactPolicy(.05))


class OrientedPlanarProofTests(unittest.TestCase):
    def setUp(self):
        self.vertices, self.faces = plane()
        self.surface = PlanarContactSurface(self.vertices, self.faces)

    def test_separation_touch_and_negative_side_are_signed(self):
        for z in (.0001, 0., -.00005, -.0002):
            with self.subTest(z=z):
                gap = self.surface.verify(tip(z))
                self.assertTrue(gap.verified, gap.reason)
                self.assertAlmostEqual(gap.gap_m, z, places=14)

    def test_crossing_internal_mesh_edge_is_supported(self):
        # The projected tool spans both triangles of the square, not one face.
        self.assertTrue(self.surface.verify(tip()).verified)

    def test_vertical_triangle_interior_intersection_gets_whole_face_extent(self):
        triangle = np.array([[[-.0003, 0, -.00008], [.0003, 0, -.00008], [0, 0, .0002]]])
        gap = self.surface.verify(triangle)
        self.assertTrue(gap.verified, gap.reason)
        self.assertEqual(gap.gap_m, -.00008)

    def test_tool_triangle_enclosing_small_scan_is_not_certified_from_samples(self):
        vertices, faces = plane(extent=.0001)
        gap = PlanarContactSurface(vertices, faces).verify(tip())
        self.assertFalse(gap.verified)

    def test_hole_inside_projected_triangle_is_unverifiable(self):
        # A regular oriented rectangular annulus with all tool vertices in its
        # support, but a hole fully enclosed in the tool triangle's interior.
        outer, _ = plane(extent=.005)
        inner, _ = plane(extent=.00002)
        vertices = np.vstack((outer, inner))
        faces = []
        for a, b in [(0, 1), (1, 2), (2, 3), (3, 0)]:
            faces.extend([[a, b, b+4], [a, b+4, a+4]])
        gap = PlanarContactSurface(vertices, faces).verify(tip())
        self.assertFalse(gap.verified)
        self.assertIn('boundary', gap.reason)

    def test_open_edge_under_tool_is_unverifiable(self):
        triangle = tip(); triangle[:, :, 0] += .0049
        self.assertFalse(self.surface.verify(triangle).verified)

    def test_curved_heightfield_is_supported_but_inconsistent_winding_is_not(self):
        curved = self.vertices.copy(); curved[3, 2] += .00001
        gap = PlanarContactSurface(curved, self.faces).verify(tip())
        self.assertTrue(gap.verified, gap.reason)
        reversed_faces = self.faces.copy(); reversed_faces[1] = reversed_faces[1, ::-1]
        gap = PlanarContactSurface(self.vertices, reversed_faces).verify(tip())
        self.assertFalse(gap.verified)
        self.assertIn('winding', gap.reason)

    def test_creased_heightfield_interior_peak_is_checked_without_sampling(self):
        vertices = np.vstack((self.vertices, [0., 0., .0001]))
        faces = np.array([[0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4]])
        surface = PlanarContactSurface(vertices, faces)
        triangle = tip(.00005); triangle[:, :, :2] *= 10
        # Each working vertex is 0.01 mm above the piecewise planar surface,
        # but the pyramid apex is inside the face and penetrates by 0.05 mm.
        gap = surface.verify(triangle)
        self.assertTrue(gap.verified, gap.reason)
        self.assertAlmostEqual(gap.gap_m, -.00005, places=14)
        engine = checker()
        engine.working_vertices = triangle[0]-[0., 0., .00005]
        engine.contact_surfaces['mandibular'] = surface
        self.assertTrue(engine.check([.00005]).valid)
        self.assertFalse(engine.check([.000049]).valid)

    def test_vertical_working_face_on_shared_edge_keeps_complete_domain(self):
        triangle = np.array([[[-.0002, -.0002, -.00003], [.0002, .0002, -.00003], [0., 0., .0002]]])
        gap = self.surface.verify(triangle)
        self.assertTrue(gap.verified, gap.reason)
        self.assertAlmostEqual(gap.gap_m, -.00003, places=14)

    def test_original_winding_defines_local_side_without_global_outside_claim(self):
        surface = PlanarContactSurface(self.vertices, self.faces[:, ::-1])
        gap = surface.verify(tip(.0001))
        self.assertTrue(gap.verified)
        self.assertAlmostEqual(gap.gap_m, -.0001)

    def test_overlapping_geometric_sheets_with_distinct_ids_are_unverifiable(self):
        vertices = np.vstack((self.vertices, self.vertices))
        faces = np.vstack((self.faces, self.faces+4))
        self.assertFalse(PlanarContactSurface(vertices, faces).verify(tip()).verified)

    def test_whole_jaw_far_aabb_proof_does_not_invent_a_signed_gap(self):
        gap = self.surface.verify(tip(.01))
        self.assertTrue(gap.verified)
        self.assertIsNone(gap.gap_m)

    def test_disconnected_nearby_surfaces_are_not_hidden_by_nearest_contact(self):
        vertical = np.array([[-.0001, -.0001, -.0002], [.0001, -.0001, .0002], [0, .0001, .0002]])
        vertices = np.vstack((self.vertices, vertical))
        faces = np.vstack((self.faces, [4, 5, 6]))
        self.assertFalse(PlanarContactSurface(vertices, faces).verify(tip()).verified)


class AdaptiveSupportTests(unittest.TestCase):
    def test_remote_fold_does_not_block_local_working_contact(self):
        vertices, faces = plane_with_remote_fold()
        surface = PlanarContactSurface(vertices, faces)
        for height in (.0001, 0., -.00005):
            with self.subTest(height=height):
                gap = surface.verify(tip(height))
                self.assertTrue(gap.verified, gap.reason)
                self.assertAlmostEqual(gap.gap_m, height, places=14)

    def test_adaptive_support_keeps_penetration_limit_and_original_sign(self):
        engine = checker()
        vertices, faces = plane_with_remote_fold()
        engine.contact_surfaces['mandibular'] = PlanarContactSurface(vertices, faces)
        self.assertTrue(engine.check([-.00005]).valid)
        excessive = engine.check([-.00005001])
        self.assertFalse(excessive.valid)
        self.assertTrue(excessive.contact_verified)
        self.assertAlmostEqual(excessive.working_gap_m, -.00005001)

    def test_uncovered_tool_to_support_segments_are_not_certified(self):
        vertices, faces = plane_with_remote_fold()
        vertices[:, 2] -= .0008
        triangle = tip(); triangle[0, 2, 2] = .0015
        gap = PlanarContactSurface(vertices, faces).verify(triangle)
        self.assertFalse(gap.verified)

    def test_intersecting_fold_cannot_be_removed_by_shrinking_support(self):
        vertices, faces = plane_with_remote_fold()
        vertices[-3:, 0] = 0.
        gap = PlanarContactSurface(vertices, faces).verify(tip())
        self.assertFalse(gap.verified)

    def test_remote_fold_with_overlapping_bounds_uses_triangle_separation(self):
        vertices, faces = plane_with_remote_fold()
        vertices[-3:] = [[0., -.0002, .001], [0., .001, -.0002], [0., .001, .001]]
        gap = PlanarContactSurface(vertices, faces).verify(tip(-.00005))
        self.assertTrue(gap.verified, gap.reason)
        self.assertIn('triangle_chart', gap.reason)
        self.assertAlmostEqual(gap.gap_m, -.00005, places=14)

    def test_curved_jaw_uses_separate_charts_for_opposite_working_faces(self):
        angles = np.arange(32)*2*np.pi/32
        vertices = np.array([[.001*np.cos(a), y, .001*np.sin(a)]
                             for a in angles for y in (-.003, .003)])
        faces = []
        for i in range(32):
            a, b = 2*i, 2*((i+1) % 32)
            faces.extend([[a, a+1, b+1], [a, b+1, b]])
        surface = PlanarContactSurface(vertices, faces)
        working = []
        for i in (0, 32):
            face = surface.triangles[i]
            center = face.mean(axis=0)
            working.append(center+.1*(face-center)+surface.normals[i]*.00005)
        gap = surface.verify(np.array(working))
        self.assertTrue(gap.verified, gap.reason)
        self.assertIn('triangle_chart', gap.reason)
        self.assertAlmostEqual(gap.gap_m, .00005, places=13)

    def test_competing_opposite_sheets_in_contact_range_remain_unverifiable(self):
        vertices, faces = plane()
        vertices = np.vstack((vertices, vertices+[0., 0., .00002]))
        faces = np.vstack((faces, faces[:, ::-1]+4))
        gap = PlanarContactSurface(vertices, faces).verify(tip(.00001))
        self.assertFalse(gap.verified)


class PatchCacheTests(unittest.TestCase):
    def test_reused_patch_recomputes_working_gap_and_coverage(self):
        vertices, faces = plane()
        surface = PlanarContactSurface(vertices, faces)
        self.assertEqual(surface.verify(tip(.0001)).gap_m, .0001)
        self.assertEqual(surface.verify(tip(-.0002)).gap_m, -.0002)
        self.assertEqual(len(surface._patch_cache), 1)
        shifted = tip(); shifted[:, :, 0] += .0049
        self.assertFalse(surface.verify(shifted).verified)

    def test_same_size_different_face_sets_do_not_alias_and_lru_is_bounded(self):
        vertices, faces = plane()
        all_vertices = np.vstack([vertices+[i*.02, 0., i*.00001] for i in range(18)])
        all_faces = np.vstack([faces+4*i for i in range(18)])
        surface = PlanarContactSurface(all_vertices, all_faces)
        for i in range(18):
            triangle = tip(.0003)+[i*.02, 0., 0.]
            gap = surface.verify(triangle)
            self.assertTrue(gap.verified, gap.reason)
            self.assertAlmostEqual(gap.gap_m, .0003-i*.00001)
        self.assertEqual(len(surface._patch_cache), 16)
        # Revisit an evicted patch: the complete proof is reconstructed.
        self.assertEqual(surface.verify(tip(-.0001)).gap_m, -.0001)
        self.assertEqual(len(surface._patch_cache), 16)

    def test_boundary_prefilter_keeps_roundoff_band(self):
        vertices, faces = plane()
        surface = PlanarContactSurface(vertices, faces)
        self.assertTrue(surface.verify(tip()).verified)
        triangle = np.array([[[.0049, -.0001, 0.], [.005-.5e-12, 0., 0.], [.0049, .0001, 0.]]])
        gap = surface.verify(triangle)
        self.assertFalse(gap.verified)
        self.assertEqual(gap.reason, 'projection_crosses_patch_boundary')

    def test_caller_mesh_mutation_cannot_stale_cached_geometry(self):
        vertices, faces = plane()
        surface = PlanarContactSurface(vertices, faces)
        expected = surface.verify(tip(.0001))
        vertices[:] = 10.
        faces[:] = 0
        self.assertEqual(surface.verify(tip(.0001)), expected)


class CheckerPolicyTests(unittest.TestCase):
    def test_touch_and_tolerance_boundary_pass_but_excess_and_wrong_side_fail(self):
        engine = checker()
        for z, valid in [(0., True), (-.00005, True), (-.00005001, False), (-.0002, False), (.0002, True)]:
            with self.subTest(z=z):
                result = engine.check([z], clearance_m=.1)
                self.assertEqual(result.valid, valid)
                self.assertTrue(result.contact_verified)
                self.assertAlmostEqual(result.working_gap_m, z)
                self.assertGreater(result.clearance_m, .0004)

    def test_valid_working_contact_cannot_hide_forbidden_contact(self):
        engine = checker(forbidden_z=0.)
        result = engine.check([0.])
        self.assertFalse(result.valid)
        self.assertIn('ur_sensor', result.pair)

    def test_forbidden_zero_clearance_still_rejects_touch(self):
        result = checker(forbidden_z=.00002).check([0.])
        self.assertFalse(result.valid)

    def test_configurable_forbidden_clearance_is_fixed_for_cached_checks(self):
        engine = checker(ContactPolicy(.05, nonworking_clearance_mm=.5))
        self.assertFalse(engine.check([0.], clearance_m=0).valid)
        self.assertFalse(engine.check([0.], clearance_m=-1).valid)
        self.assertEqual(engine.calls, 1)

    def test_both_jaws_are_checked_for_working_contacts(self):
        engine = checker()
        vertices, faces = plane(.0002)
        engine.contact_surfaces['maxillary'] = PlanarContactSurface(vertices, faces)
        result = engine.check([0.])
        self.assertFalse(result.valid)
        self.assertIn('maxillary', result.pair)
        self.assertEqual(result.working_gap_m, -.0002)

    def test_ambiguous_contact_is_reported_and_not_cached_as_valid(self):
        engine = checker()
        vertices, faces = plane(); faces[1] = faces[1, ::-1]
        engine.contact_surfaces['mandibular'] = PlanarContactSurface(vertices, faces)
        result = engine.check([0.])
        self.assertFalse(result.valid)
        self.assertFalse(result.contact_verified)
        self.assertIn('contact_unverifiable', result.pair)
        self.assertFalse(engine.check([0.]).valid)

    def test_distant_jaw_inside_overlapping_whole_bounds_needs_no_contact_permission(self):
        engine = checker()
        vertices, faces = plane(extent=.01)
        vertices[:, 2] = vertices[:, 0]+.01
        engine.contact_surfaces['mandibular'] = PlanarContactSurface(vertices, faces)
        engine.teeth[0] = ('mandibular', fcl.CollisionObject(bvh(vertices, faces)))
        result = engine.check([0.])
        self.assertTrue(result.valid, result.pair)
        self.assertTrue(result.contact_verified)
        self.assertIsNone(result.working_gap_m)

    def test_unsigned_far_separation_cannot_permit_enclosed_working_mesh(self):
        engine = checker()
        vertices, faces = plane(-.01, extent=.01)
        vertices = np.vstack((vertices, vertices+[0., 0., .02]))
        faces = [*faces[:, ::-1], *(faces+4)]
        for a, b in ((0, 1), (1, 2), (2, 3), (3, 0)):
            faces.extend([[a, b, b+4], [a, b+4, a+4]])
        faces = np.asarray(faces)
        engine.contact_surfaces['mandibular'] = PlanarContactSurface(vertices, faces)
        engine.teeth[0] = ('mandibular', fcl.CollisionObject(bvh(vertices, faces)))
        result = engine.check([0.])
        self.assertFalse(result.valid)
        self.assertFalse(result.contact_verified)
        self.assertIn('contact_unverifiable', result.pair)

    def test_self_collision_is_preserved(self):
        engine = checker()
        engine.robot.verify_data.contact = [SimpleNamespace(geom1=0, geom2=1)]
        engine.robot.verify_data.ncon = 1
        self.assertFalse(engine.check([0.]).valid)

    def test_edge_preserves_separate_gap_and_contact_proof(self):
        result = checker().edge(np.array([-.00005]), np.array([0.]), .1)
        self.assertTrue(result.valid)
        self.assertTrue(result.contact_verified)
        self.assertEqual(result.working_gap_m, -.00005)

    def test_check_old_positional_fields_remain_compatible(self):
        result = Check(True, .01, 'old')
        self.assertIsNone(result.working_gap_m)
        self.assertFalse(result.contact_verified)


class LoadedContactSupportRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from pipeline.simulation import create_simulation
        from pipeline.oral import jaw_overrides, load_jaw_settings
        from planning.infrastructure.robot import Robot
        model, data = create_simulation(jaw_settings=jaw_overrides(load_jaw_settings(), 20))
        robot = Robot(model, data)
        # Registered mandible and legacy joint states from the zero-offset
        # [50, 0, 50] run. These fixtures do not depend on generated outputs.
        world = np.array([
            [-.9942778940411979, .09812521461819636, -.04222453880308263, .3026108353547764],
            [-.07089088164977872, -.3104004394162069, .9479588862968387, -1.3115037492379624],
            [.07991215376848035, .945527899887866, .31558047818856644, 1.234133949432421],
            [0., 0., 0., 1.]])
        cls.engine = CollisionChecker(robot, {'mandibular': world}, contact_policy=ContactPolicy(.2))

    @staticmethod
    def _current_mount_joints(legacy_joints):
        # Counteract the tool mount roll to preserve the original Cartesian
        # contact cases, including the measured penetration below.
        joints = np.asarray(legacy_joints, dtype=float).copy()
        joints[5] -= np.pi
        return joints

    def test_previously_blocked_curved_neighborhoods_are_certified(self):
        states = [
            [-1.7299686716075664, -2.0712635248789875, 2.151029583178344,
             -2.2989155251206004, -.47430028772571786, 1.5369997930276895],
            [-1.7296054572689155, -2.0535125487198926, 2.1352427644586607,
             -2.3014433320612078, -.474519705212777, 1.5376334186115246],
            [-1.7306525938519237, -2.0359591496200804, 2.124583866318827,
             -2.3067113670664856, -.4738875741719961, 1.53580522367057],
            [-1.711378179424212, -2.0005800050628046, 2.1099685744387893,
             -2.3566950594997245, -.4857354480561731, 1.5687432902444929],
            [-1.6977959603396529, -1.9751542136996447, 2.1032471230880105,
             -2.395096088534687, -.49434332941750503, 1.5910667694779015],
            [-1.6957993305330037, -1.9668460812810298, 2.1044792276390467,
             -2.4074709614978467, -.4956259634201601, 1.5942880332933855]]
        for layer, q in zip((115, 463, 810, 1389, 1736, 1852), states):
            with self.subTest(layer=layer):
                result = self.engine.check(self._current_mount_joints(q))
                self.assertTrue(result.valid, result.pair)
                self.assertTrue(result.contact_verified)
                self.assertGreater(result.clearance_m, 0.)
                self.assertGreaterEqual(result.working_gap_m, -.0002)

    def test_real_excess_working_penetration_remains_rejected(self):
        q = [-1.7250550766010484, -2.0213672409582175, 2.1178277069834532,
             -2.3231889623771003, -.4772823282761167, 1.5455257813556464]
        result = self.engine.check(self._current_mount_joints(q))
        self.assertFalse(result.valid)
        self.assertTrue(result.contact_verified)
        self.assertGreater(result.clearance_m, 0.)
        self.assertAlmostEqual(result.working_gap_m*1000, -.2424036256, places=6)


class LoadedModelPartitionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from pipeline.simulation import create_simulation
        from planning.infrastructure.robot import Robot
        model, data = create_simulation()
        cls.robot = Robot(model, data)
        cls.engine = CollisionChecker(cls.robot, contact_policy=ContactPolicy(.05))

    def test_actual_partition_and_json_manifest(self):
        manifest = self.engine.contact_manifest
        self.assertEqual(len(manifest['working_face_indices']), 48)
        self.assertEqual(len(manifest['nonworking_face_indices']), 8102)
        self.assertEqual(len(manifest['mixed_boundary_face_indices']), 36)
        self.assertEqual(json.loads(json.dumps(manifest, allow_nan=False)), manifest)
        self.assertEqual(set(manifest['jaws']), {'mandibular', 'maxillary'})
        names = [self.robot.model.geom(g).name for g, _, _ in self.engine.parts]
        self.assertIn('ur_force_sensor_geom', names)
        self.assertIn('ur_end_scaler_geom', names)

    def test_tcp_frame_partition_matches_actual_fk_vertices(self):
        tool = self.robot.model.geom('ur_end_scaler_geom').id
        vertices, faces = mesh_from_geom(self.robot.model, tool)
        self.robot.fk(self.robot.home)
        data = self.robot.data
        tcp = self.robot.pose(data)
        local = (vertices @ data.geom_xmat[tool].reshape(3, 3).T+data.geom_xpos[tool]-tcp[:3, 3]) @ tcp[:3, :3]
        expected = np.flatnonzero((np.linalg.norm(local[faces], axis=2) <= .001).all(axis=1))
        np.testing.assert_array_equal(expected, self.engine.contact_manifest['working_face_indices'])

    def test_manifest_reconstruction_is_stable_at_different_robot_home(self):
        original_home = self.robot.home.copy()
        try:
            self.robot.home[0] += .1
            rebuilt = CollisionChecker(self.robot, contact_policy=ContactPolicy(.05))
            self.assertEqual(rebuilt.contact_manifest, self.engine.contact_manifest)
        finally:
            self.robot.home[:] = original_home


if __name__ == '__main__':
    unittest.main()
