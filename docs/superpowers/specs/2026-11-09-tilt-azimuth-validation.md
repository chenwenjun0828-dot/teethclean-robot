# Tilt Azimuth Validation

Date: 2026-11-09. Scope: add independent local tilt direction while preserving
the existing planning, collision and control algorithms.

## Parameter Contract

`tilt_deg` sets the shaft angle from the inward reference normal, strictly
between 0 and 90 degrees. `tilt_azimuth_deg` sets the tilt direction about the
outward normal, finite in [-180, 180]. Defaults remain 15 and 0 degrees.
Azimuth 0 follows projected forward tangent, +90 follows `normal cross tangent`,
and -90 follows its opposite. The zero-roll frame rotates with azimuth; axial
roll is still measured in that frame. These are local angles, not world pitch.
Changing orientation keeps the TCP position and normal offset fixed.

Both planning and pipeline CLIs accept `--tilt-deg` and `--tilt-azimuth-deg`.
Pipeline API, tracking, dry-run and temporary/no-output execution pass both
values. Configuration exports and geometry agree before saved-plan execution.
Legacy configuration without azimuth loads as zero. The independent timed FK
audit checks the expected shaft direction separately from the pose constructor.

## Automated Checks

- Before edits: 146 integration tests passed in 42.801 s.
- Focused orientation, roll, planning and pipeline checks: 45 passed in 3.308 s.
- Complete planning suite: 63 passed in 4.497 s.
- Complete integration suite: 147 passed in 49.952 s with `OMP_NUM_THREADS=1`.
- The actual pipeline CLI accepted both angles in a dry run.

Commands for the complete suites:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s planning -t . -q
$env:OMP_NUM_THREADS = '1'
.\.venv\Scripts\python.exe -m unittest discover -s tests -q
```

The default-thread integration run failed one existing automatic-registration
fitness assertion (0.2373 versus a required value above 0.8); its isolated
default-thread rerun also failed (0.2000). The isolated single-thread rerun and
the complete single-thread suite passed with registration ICP RMSE 0.1624 mm.
This is consistent with nondeterministic parallel RANSAC sampling despite its
fixed seed. Registration code, its thresholds and the test assertion were not
changed. The test uses the legacy zero azimuth and fails before pose planning.
Default-thread registration variability remains a separate limitation.

Coverage includes legacy orientation equivalence, cardinal tilt directions,
constant normal angle, orthonormal frames, rigid-transform equivariance,
fixed TCP position, invalid arguments, source and whole-path propagation,
fixed-roll and roll-policy references, continuous and segmented audits,
saved configuration consistency, CLI parsing and no-output propagation.

## Actual Model Comparison

The preview reuses the user's original reference index 100 and camera view,
with jaw opening 20 degrees, fixed roll 190 degrees (canonical -170), 1.5 mm
normal offset, and the previously tested robot mount 150 mm closer to the jaw.
All panels use exact IK, the same TCP target and the original model geometry.
These are static candidate poses; no trajectory was executed.

At this reference, positive azimuth 60 degrees lowers the handle in the user's
marked direction, while negative 60 raises it. Increasing tilt from 15 to 35
degrees makes this change more visible. The direction can differ elsewhere
because the tooth/path frame varies along the trajectory.

![Before and after](../../../tmp/tilt_azimuth_preview_20261109/downward_comparison.png)

The six-case comparison (tilt 15/35, azimuth 0/-60/+60), exact joints, TCP
positions and collision probes are in
`tmp/tilt_azimuth_preview_20261109/`. The corresponding reproducible helper is
`tmp/preview_tilt_azimuth.py`. All six candidates still intersect the lower
jaw at this original connector reference. The upper jaw is clear; the
before/after upper clearances are 18.52 and 22.01 mm. Rendered contact dots
are full-tool FCL results, not evidence of a permitted treatment contact.

## Whole-Path Experiment

The full continuous planner was run with tilt 35, azimuth +60, roll 190,
jaw opening 20 degrees, and the 150 mm closer mount. It reused frozen
perception with the lower-jaw rigid transform applied to the registration.
It tested eight source-trim candidates, 0.05 through 0.4 mm in 0.05 mm steps.
All failed `no_continuous_ik_path`, first disconnected layers 9 through 12.
Planning took 7.119 s, total experiment time 8.334 s; perception was reused.
There was no executable trajectory and no control execution or timed FK audit.

Artifacts:
`tmp/pipeline_fixed_roll_190_mount150_open20_tilt35_azimuth60_20261109/experiment.json`
and its `tracking/` configuration, geometry, candidates and planning report.

Example for a fresh pipeline run using the new controls:

```powershell
.\.venv\Scripts\python.exe -m pipeline.run --jaw mandibular --registration-method automatic --jaw-opening-deg 20 --fixed-roll-deg -170 --tilt-deg 35 --tilt-azimuth-deg 60 --no-output
```

This command uses the normal scene mount; the closer mount was an explicit
experiment override. The angles demonstrate the requested orientation control,
not a validated executable whole-jaw plan. Planning and collision rejection
remain active and stop execution when no complete trajectory passes.

## Default Pipeline Settings Recheck

The exact angles (-170 roll, 35 tilt, +60 azimuth) were rechecked using the
same frozen perception and the pipeline's default source trims (1, 2, 3 mm),
rather than the earlier fine-trim experiment. Limits remained 0.15 rad joint
step and 20 degrees orientation step. No perception or planner code changed.

| Mount | Jaw opening | Planning time | Result |
| --- | --- | --- | --- |
| Default | 0 degrees | 2.765 s | No continuous IK at layer 28 for trim 1; trims 2/3 lack source coverage |
| Default | 20 degrees | 3.283 s | No continuous IK at layers 65/61 for trims 1/2; trim 3 lacks source coverage |
| 150 mm closer | 0 degrees | 4.021 s | No continuous IK at layer 27 for trim 1; trims 2/3 lack source coverage |
| 150 mm closer | 20 degrees | 3.441 s | No continuous IK at layers 59/50 for trims 1/2; trim 3 lacks source coverage |

These runs produced no executable trajectories and attempted no control.
Artifacts are in `tmp/tilt35_azimuth60_recheck_{default_mount,mount150}_open{0,20}_20261109/`.

The closer-mount, open-20, trim-1 candidate was separately screened across all
319 samples and all exact IK solutions. Every sample has at least one
collision/clearance-valid solution (4 to 8 valid solutions per sample), but
this does not establish collision-free edges or a feasible timed trajectory.
The first graph disconnection is at layer 59, in the connector from tooth
instance 10 to 9. From solutions reachable up to that layer, the minimum
maximum-per-joint change to the next bank is 3.161344 rad (about 181.13 degrees),
versus the allowed 0.15 rad (8.59 degrees). The TCP step is 0.196504 mm and
the orientation step is 3.129148 degrees. No orientation-step violations occur
anywhere in this candidate (maximum 11.104903 degrees). Thus the first search
failure is a joint-continuity constraint, before lazy node/edge collision
validation. Even allowing a different starting branch, this sampled candidate
needs a maximum joint step of at least 2.807909 rad when ignoring orientation
and collision limits. The audit details are in
`tmp/tilt35_azimuth60_recheck_mount150_open20_20261109/candidate_1_graph_diagnosis.json`.

The earlier static reference-100 contact image uses the original reference
path, not this newly retained/trimmed candidate. Its local collision result
should not be attributed to the new candidate. The conclusion is limited to
the tested path candidates and limits; it is not a proof that no physical
trajectory with these tool angles can exist.
