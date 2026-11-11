# Fixed 10-degree roll experiment

Date: 2026-11-08 (Asia/Shanghai).

## Scope and interface

The requested experiment fixes the continuous trace's local axial roll to
10 degrees, with the existing 15-degree shaft tilt and 1.5 mm non-contact offset.
The simulation scene, robot mount, path reconstruction, IK, full-mesh collision
checks, joint-step limits, and execution guards remain in use.

`--fixed-roll-deg` is available in `pipeline.run` and in `planning --continuous`.
The library option is `SweepOptions.fixed_roll_deg`; `None` keeps sampled roll
search. Finite values within +/-180 degrees are accepted, including zero and
negative angles. Segmented mode and the searched Y/Z corridor cannot be
combined with fixed roll. The fixed value propagates through `--no-output` and
is validated during `--dry-run`.

Both source reachability and composite-path candidate generation use the same
single roll. Interpolation keeps that target roll constant. Saved report and
reference options must agree, target rolls are checked, and the independent
full-FK audit still evaluates actual timed interpolation. The existing one-degree
orientation error tolerance applies to actual motion. The initial approach
starts at home and is not required to have the trace roll. World orientation
can change with the local normal/tangent frame. This does not constrain the
curved working tip's blade contact angle independently.

## Observed lower-jaw result

The complete pipeline was invoked with automatic registration, fixed 10-degree
roll, headless execution, and persistent output in
`tmp/pipeline_fixed_roll_10_lower_20261108_trial01/`.

The pipeline stopped in planning after 9.58 seconds, before trajectory audit
or dynamics execution. Perception took 5.98 seconds. Geometry, planner setup,
and pose/motion planning took 0.82, 1.20, and 0.71 seconds respectively.

At 1 mm endpoint trims, all 234 sampled positions had IK candidates. The
whole-path graph disconnected at zero-based layer 32 on the bridge from tooth
instance 10 to instance 9. These are segmentation instance identifiers, not
verified FDI tooth numbers. For the closest candidate pair reachable from the
start, the adjacent TCP positions were 0.248 mm apart and orientations differed
by 6.95 degrees, below the 20-degree orientation-step limit. The minimum possible
maximum joint change was 0.2054 rad (elbow), above the existing 0.15 rad limit;
wrist 3 also changed by 0.1743 rad. This first graph rejection preceded the lazy
collision checks for candidate nodes/edges; source feasibility had already
included collision checks.

At 2 and 3 mm trims, retained source coverage for tooth instance 10 fell below
the required 25%. Resampling the same saved composite reference at 0.125 and
0.0625 mm still failed whole-path graph connectivity (386 and 738 samples).
These are diagnostic searches, not new end-to-end pipeline runs or proof of
mathematical unreachability. Limits and roll were not relaxed. Diagnostics are
stored in `fixed_roll_diagnosis.json` next to the pipeline report.

## Verification and reproduction

All 145 tests under `tests/` and all 53 tests under `planning/` passed. Added
checks cover fixed-roll propagation with persistent/ephemeral output, invalid
requests, source/search failure without alternative rolls, reference consistency,
and rejection of a changed target roll or incorrect actual full-FK orientation.

```powershell
.\.venv\Scripts\python.exe -m pipeline.run --jaw mandibular --registration-method automatic --fixed-roll-deg 10 --headless --output tmp/my_fixed_roll_10_trial
.\.venv\Scripts\python.exe -m tmp.fixed_roll_10_diagnostics --run tmp/pipeline_fixed_roll_10_lower_20261108_trial01
.\.venv\Scripts\python.exe -m unittest discover -s tests -q
.\.venv\Scripts\python.exe -m unittest discover -s planning -t . -q
```

Use a new empty output directory for a persistent run. Add `--no-output` instead
of `--output ...` to remove all run artifacts on completion or failure. Omit
`--headless` for GUI execution; this failed plan never reaches the viewer.

## Follow-up: relaxed joint-step limits

The user requested relaxing the blocking requirement. `pipeline.run` and the
standalone planning entry now accept `--joint-step-rad`; the default remains
0.15 rad because no relaxed limit has passed fixed-roll execution yet. This
setting controls adjacent IK candidate displacement and does not change joint
velocity/acceleration, the 0.004 rad collision sweep resolution, reference pose
tolerances, clearance, or runtime guards. The API, dry-run, ephemeral recursion,
and reports preserve the selected value.

Using the frozen reference and registration from the original 10-degree run,
limits of 0.25, 0.3, 0.5, and 1.0 rad still disconnected at graph layers 33, 34,
36, and 36. The bottleneck threshold for any whole-path graph connection before
collision checks was 2.94504 rad, substantially above the first observed
0.2054 rad gap. At 3.0 and 3.2 rad, lazy collision checks rejected the candidate
connections: the tool collided with both jaw meshes and some branches had
forearm/wrist or forearm/tool self contacts. Neither setting reached a valid
complete path, interpolation audit, or dynamics execution.

Resampling at 0.0625 mm still failed at joint-step limits of 0.25, 0.3, and
0.5 rad. At 0.3 rad, the existing reconstruction was also retried with endpoint
trims from 0.25 to 1.75 mm in 0.25 mm increments. Trims through 1.5 mm failed
graph connectivity; 1.75 mm failed tooth-instance-10 source coverage. The run
is stored in `tmp/fixed_roll_10_trim_quarter_joint_03_20261108/`. These frozen-input
diagnostics do not certify execution or imply mathematical unreachability.

The original joint-step diagnostic is reproducible with:

```powershell
.\.venv\Scripts\python.exe -m tmp.fixed_roll_step_trials --run tmp/pipeline_fixed_roll_10_lower_20261108_trial01 --limits 0.25 0.3 0.5 1
.\.venv\Scripts\python.exe -m tmp.fixed_roll_step_trials --run tmp/pipeline_fixed_roll_10_lower_20261108_trial01 --limits 3 3.2
.\.venv\Scripts\python.exe -m tmp.fixed_roll_trim_trials --source tmp/pipeline_fixed_roll_10_lower_20261108_trial01 --out tmp/my_fixed_roll_trim_trial
```

After exposing the joint-step option, all 146 tests under `tests/` and all 54
under `planning/` passed (200 total). A full pipeline retry with
`--fixed-roll-deg 10 --joint-step-rad 0.3 --headless --no-output` also stopped
before execution. That new automatic registration had 0.2959 mm ICP RMSE and
failed source reachability for tooth instance 10; it differs from the frozen
0.1624 mm registration used for the controlled graph trials above. Its
temporary directory was removed. No successful relaxed-step execution is claimed.
