# Fixed 190-degree roll with a closer robot mount

Date: 2026-11-09 (Asia/Shanghai).

## Scope

Move the simulated robot toward the mandibular middle-region source geometry
while keeping the local roll fixed at 190 degrees (canonical -170 degrees).
Reuse the perception and registration from
`tmp/pipeline_fixed_roll_10_lower_20261108_trial01/`. Regenerate geometry,
reachable source windows, and continuous connectors for each complete trial.
The frozen scene/robot/oral configuration hashes and tooth transforms are
verified by the validation helper.

The original mount is `[0.815, -0.9, 0.77]` m. Horizontal translation follows
the unit direction `[-0.793447681, -0.608638462, 0]` toward the mean middle-region
source position. Height and mounting quaternion are preserved. The target
shaft tilt remains 15 degrees, offset 1.5 mm, required mesh clearance 0.15 mm,
minimum per-tooth source coverage 25%, and joint-step limit 0.15 rad.

Only temporary experiment helpers and this record were changed. The scene XML,
production planner, and default simulation placement were not changed: no new
placement passed complete validation. These are non-contact simulation trials,
not hardware or force-contact trials.

## Reach Screening

`tmp/fixed_roll_190_mount_screen_20261109.json` screens the previous 0.05 mm
trimmed reference, with 316 samples. This is an IK/graph precheck, not a collision
or execution certificate, and does not regenerate that particular reference.

| Horizontal Move Toward Teeth | Targets With No IK |
| --- | --- |
| 0 mm | 75 |
| 20 mm | 52 |
| 40 mm | 37 |
| 60 mm | 32 |
| 80 mm | 21 |
| 100 mm | 0 |
| 120 mm | 0 |
| 150 mm | 0 |
| 180 mm | 0 |

## Complete Replanning

Full replanning was run at 100, 150, and 180 mm horizontal translations:

| Move | Mount Position in Metres | Outcome |
| --- | --- | --- |
| 100 mm | `[0.735655232, -0.960863846, 0.77]` | No executable continuous path |
| 150 mm | `[0.695982848, -0.991295769, 0.77]` | No executable continuous path |
| 180 mm | `[0.672179417, -1.009554923, 0.77]` | No executable continuous path |

At 100 mm, default trims of 1, 2, and 3 mm still fail per-tooth coverage. Each
of the three placements was then tried with trims of 0.05 through 0.40 mm in
0.05 mm increments. All three contain candidates with IK at every sample but
no connected graph under the joint-step constraint. The 100 mm, 0.05 mm trim
candidate still has an empty IK bank on its regenerated last bridge; this is
different from screening the previous frozen reference above. The 0.40 mm
trim candidates fail instance-9 coverage.

For the 150 mm translation and 0.10 mm trim candidate, all 291 positions have
IK solutions. The first graph disconnection is at zero-based layer 68 on the
10-to-9 bridge: adjacent TCPs are 0.683 mm apart, orientations differ by
18.642 degrees, and the smallest maximum joint change from a reachable branch
is 0.2972 rad, above 0.15 rad. The maximum orientation step anywhere is
18.762 degrees, below the 20-degree limit. Segmentation identifiers are not
verified FDI tooth numbers.

Before collision and orientation constraints, the minimum whole-graph joint
step threshold at this placement is 0.30075 rad. This figure alone cannot
certify a feasible trajectory.

A further 150 mm trial uses `spacing_mm=0.0625` and 0.10 mm trims. Geometry and
source assignments are regenerated at that resolution; this is not simply
resampling the previous curve. All 902 positions have IK, but the graph still
disconnects at layer 197. The first required joint step is 0.15874 rad and the
whole-graph threshold before collision/orientation checks is 0.21911 rad.

All 28 full-planner attempts across these five experiments failed before
trajectory audit or dynamics execution. No executable trajectory or successful
simulation execution is claimed.

## Independent Collision Screening

The 150 mm, 0.10 mm trim reference was additionally checked at every sample
with all IK solutions against the full tool/robot and both tooth meshes.
Of 291 target positions, 79 have no solution satisfying collision/clearance
requirements. These are individual-state checks, not a swept collision audit.

Probes at layers 0, 100, 160, and 230 confirm mesh intersection at every IK
solution: the tool intersects the maxillary mesh at 0, 160, and 230, and the
mandibular mesh at 100. The checker's clearance value of -1 m is a collision
sentinel, not a measured penetration depth.

For a given world TCP pose, changing robot placement or IK branch leaves the
tool-to-tooth geometry unchanged. Consequently, moving the base alone cannot
remove these particular tool/tooth intersections. Connector or tool-pose targets
need revision before another full feasibility claim. The observations apply
to the tested references, not every conceivable fixed-190-degree path.

## Artifacts and Reproduction

Each trial directory contains `experiment.json` and `tracking/planning_report.json`:

- `tmp/pipeline_fixed_roll_190_mount100_20261109_trial01/`
- `tmp/pipeline_fixed_roll_190_mount100_20261109_trim_scan/`
- `tmp/pipeline_fixed_roll_190_mount150_20261109_trim_scan/`
- `tmp/pipeline_fixed_roll_190_mount180_20261109_trim_scan/`
- `tmp/pipeline_fixed_roll_190_mount150_20261109_dense/`

The 150 mm trim-scan directory also contains `candidate_2_graph_diagnosis.json`
with the 79 rejected samples and `candidate_2_graph_diagnosis_probed.json`
with the explicit collision probes. Its dense counterpart contains
`candidate_1_graph_diagnosis.json`.

```powershell
.\.venv\Scripts\python.exe -m tmp.validate_fixed_roll_pipeline --source tmp/pipeline_fixed_roll_10_lower_20261108_trial01 --out tmp/my_mount150_trial --roll-deg 190 --mount-toward-mm 150 --trim-mm .05 --max-trim-mm .4 --trim-step-mm .05
.\.venv\Scripts\python.exe -m tmp.inspect_mount_sweep --run tmp/my_mount150_trial --candidate 2 --check-nodes
.\.venv\Scripts\python.exe -m tmp.inspect_mount_sweep --run tmp/my_mount150_trial --candidate 2 --probe-layers 0 100 160 230
```

Use a new empty output directory. The helper also accepts an explicit
`--mount-position-m X Y Z` instead of `--mount-toward-mm`.
