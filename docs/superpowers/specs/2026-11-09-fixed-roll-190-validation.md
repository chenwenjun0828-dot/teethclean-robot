# Fixed 190-degree roll validation

Date: 2026-11-09 (Asia/Shanghai).

## Scope

Validate the currently configured mandibular middle-region continuous sweep
with local axial roll fixed at 190 degrees, represented internally by its
equivalent -170 degrees. The region contains segmentation instances 10, 9, 1,
and 2; these identifiers are not verified FDI tooth numbers. This is not a
full-mouth or force-contact execution experiment.

The frozen perception inputs, registration, initial robot configuration, and
mount from `tmp/pipeline_fixed_roll_10_lower_20261108_trial01/` were reused.
The scene XML, robot XML, and oral settings fingerprints were verified against
that run, as were the actual scene tooth transforms. Geometry, fixed-roll
source reachability, and connectors were regenerated for the new angle.

The shaft tilt remains 15 degrees, path offset 1.5 mm, minimum mesh clearance
0.15 mm, minimum per-tooth source coverage 25%, and adjacent joint-step limit
0.15 rad. Existing IK, mesh collision checks, audits, and execution guards are
used. No production algorithms or defaults were changed.

## Results

Neither validation run produced an executable trajectory. Both stopped before
trajectory audit and dynamics execution; no control or contact result is claimed.

| Run | Endpoint Trims | Outcome | Planning Time | Experiment Time |
| --- | --- | --- | --- | --- |
| `pipeline_fixed_roll_190_lower_20261109_trial01` | 1, 2, 3 mm | Per-tooth coverage rejection | 3.31 s | 4.12 s |
| `pipeline_fixed_roll_190_lower_20261109_trim_scan` | 0.05 to 0.40 mm, step 0.05 mm | IK or coverage rejection | 4.03 s | 4.84 s |

Times exclude interpreter/import startup and the reused perception computation.

With default trims of 1 or 2 mm, instance 9 retains two 1.5 mm source intervals,
totalling approximately 22.9% of its 13.0924 mm source, below the 25% minimum.
The 3 mm attempt fails coverage for instance 10.

Smaller trims of 0.05, 0.10, 0.15, and 0.20 mm satisfy coverage and reach
candidate generation, but have target poses with no IK solutions. Trims of
0.25 through 0.35 mm fail instance-2 coverage; 0.40 mm fails instance-9 coverage.

An additional diagnostic scanned every target of the four coverage-valid
references, including the portion after the planner's first rejection:

| Trim | Samples | Targets Without IK | First Rejected Layer | Largest Minimum Required Arm Length |
| --- | --- | --- | --- | --- |
| 0.05 mm | 316 | 75 | 121 | 884.05 mm |
| 0.10 mm | 291 | 69 | 104 | 878.64 mm |
| 0.15 mm | 284 | 67 | 101 | 877.77 mm |
| 0.20 mm | 301 | 86 | 94 | 871.03 mm |

The first failure is on the bridge connecting source strokes 1 and 9 within
tooth instance 9. The whole-reference diagnostic also finds missing IK on the
9-to-1 and 1-to-2 bridges. At the first failure the tool-facing filter passes,
but the exact analytic solver has zero raw branches, before joint-step graph
checks or full-path lazy collision checks. Source feasibility already includes
collision checks.

The robot's two principal links are 425 and 392 mm long, totalling 817 mm.
For each shoulder/wrist branch the diagnostic subtracts the robot's lateral
and wrist offsets from the required wrist pose before evaluating the two-link
reach. Some targets require even the shortest branch to exceed that 817 mm
bound, so the elbow cosine is greater than one. This is a reach failure for
the current pose and mount, rather than a restrictive adjacent joint-step limit.
The 884.05 mm figure is a two-link reach requirement, not the robot's advertised
total TCP reach or a required base translation distance.

These results establish failure of the tested references in the current setup,
not impossibility of every possible 190-degree path or robot placement. A next
experiment can retain 190 degrees while adjusting the robot mount or the
connector geometry, followed by the same coverage, collision, interpolation,
and dynamics checks.

## Artifacts and Reproduction

Run summaries are `experiment.json` in the two run directories under `tmp/`.
Their `tracking/planning_report.json` files contain the authoritative planner
attempts and source windows. The trim scan also contains
`fixed_roll_190_diagnosis.json` with branch reach calculations and all empty-IK
intervals. No `trajectory.npz` or successful execution report was produced.

```powershell
.\.venv\Scripts\python.exe -m tmp.validate_fixed_roll_pipeline --source tmp/pipeline_fixed_roll_10_lower_20261108_trial01 --out tmp/my_fixed_roll_190_default --roll-deg 190
.\.venv\Scripts\python.exe -m tmp.validate_fixed_roll_pipeline --source tmp/pipeline_fixed_roll_10_lower_20261108_trial01 --out tmp/my_fixed_roll_190_trim_scan --roll-deg 190 --trim-mm .05 --max-trim-mm .4 --trim-step-mm .05
.\.venv\Scripts\python.exe -m tmp.diagnose_fixed_roll_190 --run tmp/my_fixed_roll_190_trim_scan
```

Use a new empty output directory. The validation helper converts 190 degrees
to -170 degrees for the existing planning API. For a fresh perception/pipeline
run the equivalent option is `--fixed-roll-deg=-170`.
