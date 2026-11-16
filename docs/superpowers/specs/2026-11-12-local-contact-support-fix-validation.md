# Local Contact Support Repair Validation

Date: 2026-11-12

## Conditions

- Prepared input: outputs/tutorial/manual/regions/outer_front, original line.
- Mode: simple, full collision checking, angles [50, 0, 50] degrees.
- Jaw opening: 20 degrees; working radius: 1 mm.
- Zero standoff; contact tolerance: 0.2 mm; nonworking clearance: 0 mm.
- local_angle_search=false; existing common-angle correction grid retained.
- Backend: whole_triangle_oriented_heightfield_v3.

## Previously Rejected Poses

All six representative nominal joint states previously rejected by the broad
support test now pass the complete contact policy, with positive forbidden-part
clearance. They are retained in loaded-model regression tests independently of
generated planning output.

| Layer | Parameter (mm) | Working directional gap (mm) | Result |
| --- | --- | --- | --- |
| 115 | 4.053613 | +0.005762 | Verified |
| 463 | 13.645052 | +0.018381 | Verified with complete face subdivision |
| 810 | 23.276161 | +0.075982 | Verified with local charts |
| 1389 | 39.097940 | +0.017257 | Verified with local charts |
| 1736 | 49.414981 | -0.101998 | Verified with complete face subdivision |
| 1852 | 52.633946 | -0.130086 | Verified; upper jaw excluded by mesh separation |

At layer 1852, the upper-jaw mesh is actually 26.417669 mm from the working
mesh despite overlapping overall bounds. All original jaw and working vertices
have a separating-plane gap of 17.183253 mm along the nearest-point direction.
This certificate excludes the entire convex hull and carries no signed-gap claim.

## Tolerance Check

Nominal layer 1041, parameter 29.076060 mm, tooth 9, has a verified conservative
directional gap of -0.242404 mm and remains rejected by the -0.2 mm limit. This
is a directional bound, not an exact Euclidean penetration measurement.

An independent check of all 125 common-angle corrections and their eight
returned IK branches at that point found 56 accepted candidates out of 1000.
These belong to eight angle corrections, with seven accepted branches each.
The largest verified directional gap was -0.164034 mm at correction
[-2.5, -5, 5] degrees. Therefore this nominal failure alone does not prove
infeasibility of the whole common-angle search. It does verify that the repair
has not removed the tolerance gate.

Artifact: outputs/planning/contact_front_50_0_50_zero_tol020_contact_fix/mandatory_1041_diagnostic.json.
This is a point diagnostic, not an edge or trajectory certificate.

## Full Planning

The final run is in outputs/planning/contact_front_50_0_50_zero_tol020_contact_fix_final.
Result: no_continuous_outer_front_sweep, execution_ready=false. All 125 common
angle candidates were tested, with local angle search disabled. Planning took
116.529 seconds and made 1496 uncached collision queries. No executable
trajectory was generated; trajectory dynamics acceptance was not reached.

The remaining obstruction is fully localized on the discrete search grid:

- 117 corrections have no accepted IK branch at mandatory layer 1041.
- The remaining eight corrections pass every one of the 17 mandatory samples.
- Six of those eight have no accepted IK branch at dense layer 129,
  parameter 4.702593 mm, tooth 11.
- Correction [-2.5, -2.5, 5] fails at layer 130, parameter 4.752555 mm.
- Correction [-2.5, -5, 5] fails at layer 131, parameter 4.802517 mm.

At each of these eight dense cuts, seven branches intersect the mandibular
mesh with the scaler's NONWORKING partition. The remaining branch has a
forearm/scaler or forearm/sensor self-collision. These are positive collision
queries, not folded-support or missing local orientation diagnoses. The
configured zero nonworking clearance still prohibits intersection. Increasing
working contact tolerance does not permit these nonworking collisions.

Final-code diagnostics:

- nominal_probe_diagnostic.json: 16/17 nominal mandatory samples accepted;
  only nominal layer 1041 exceeds the working tolerance.
- surviving_angle_cut_diagnostic.json: all 17 mandatory samples accepted for
  each of the eight remaining common corrections.
- nonworking_layer129_diagnostic.json: all branches checked for all eight
  corrections; six cuts are confirmed at layer 129.
- remaining_angle_cut_diagnostic.json: all branches checked at the layer-130
  and layer-131 cuts for the remaining two corrections.

All four files are in the final output directory. This proves failure for the
specified discrete grid and unchanged working partition; it does not prove
physical impossibility over unrestricted angles, offsets or working regions.

The intermediate _local_support_v3 directory contains an earlier implementation
attempt. The _contact_fix run was interrupted to add early proof of an
infeasible dense node bank. Neither is the final result.

```powershell
.venv/Scripts/python.exe -m planning --params config/planning_contact.json --contact-tolerance-mm 0.2 --no-local-angle-search --out outputs/planning/contact_front_50_0_50_zero_tol020_contact_fix_reproduction
```

## Regression

Planning discovery: 116 tests passed. External contact/configuration, saved-plan,
position-control and existing contact-workflow regressions: 117 tests passed.
Total: 233 passed, including 43 contact backend/loaded-model tests. The final
repeat includes deeply enclosed meshes and early exit at a proven infeasible
dense layer.

Coverage includes irrelevant folds, loose overlapping bounds, original triangle
distance exclusion, opposite chart directions, genuine intersecting folds,
opposing sheets, holes inside working faces, open boundaries, original signed
side and the real out-of-tolerance pose. Probe early exit is tested while all
IK branches remain available to complete node and edge validation. Unsigned
distance cannot permit an enclosed tip; an actual all-vertex separating plane
is required for the extra far exclusion.
