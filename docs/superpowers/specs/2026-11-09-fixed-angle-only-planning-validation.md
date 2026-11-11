# Fixed-Angle-Only Planning Validation

Validated on 2026-11-09 with the repository `.venv`.

## Removed Runtime Paths

- Segmented motion and pose-selection implementations and exports.
- Old infrastructure planner containing duplicate segmented/roll-search logic.
- Sampled-roll candidates, roll-grid settings and public CLI switches.
- Directed Y/Z policy, corridor search and policy-specific axial timing/audits.
- Pipeline planning-mode selectors and segmented fallback paths.

Initial approach generation now lives in `infrastructure/transfer.py`. The
sweep still starts from a validated approach and contains one continuous trace.
The planner searches IK branches at prescribed poses, never alternative angles.
Default local tilt/azimuth/roll is `(15, 0, 0)` degrees.

Saved fixed-angle continuous plans remain readable through explicit metadata
parsers. Saved segmented, variable-roll and direction-corridor plans are rejected
before execution. Historical-result visualization helpers are retained.

## Tests

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s planning -t . -p 'test_*.py' -q
# 93 tests passed
.\.venv\Scripts\python.exe -m unittest discover -s tests -t . -q
# 169 tests passed
git diff --check
# passed
```

Tests cover removed CLI rejection, fixed defaults, regional IK and collisions,
no angle fallback, selected-tooth mapping, saved-plan consistency, actual timed
FK, C2 timing, downsampling, execution gates and visualization. A generated-sweep
regression verifies that the segment stores selected teeth separately from
per-interval trajectory labels.

The first broad test run also encountered an intermittent automatic-registration
fitness assertion outside the planning changes. The final full run passed without
changes to the registration implementation or that fitness assertion.

## Real Saved-Trajectory Preview

```powershell
.\.venv\Scripts\python.exe -m planning.view --plan tmp/fixed_55_m20_m141_mount150_open20_dense005_20261109/tracking_90s_e81d0ad342 --snapshot tmp/fixed_angle_only_validation_20261109/trajectory_preview.png
```

The viewer restored the saved jaw opening, validated the source, loaded the
90-second continuous fixed-angle trajectory and rendered the scaler, teeth,
source/connector paths and TCP marker. The screenshot was inspected visually.
This check reuses a validated existing trajectory; it does not assert that a
new angle tuple or another region has a feasible full path.

Planning and pipeline dry runs also passed with explicit left/right region
selection and the local angles `(55, -20, -141)` degrees.
