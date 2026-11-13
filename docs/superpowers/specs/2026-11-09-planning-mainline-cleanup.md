# Planning Mainline Cleanup

The user settled the mainline as dense pose sampling plus the existing fixed-path
TOPPRA timing, and authorized deleting previous planning logic. This cleanup
preserves that algorithm and its validation policy.

## Kept

- Separate geometry sampling (0.25 mm) and pose sampling (0.05 mm).
- Fixed-angle continuous IK and independently exported pose arrays.
- One natural joint cubic, TOPPRA/ECOS local timing, C2 clock and fast approach/retreat.
- Requested total duration including all motion phases.
- Existing position, orientation-policy, rate, coverage, collision and dynamics checks.
- Existing CLI entrypoints, visualization and valid historical artifact playback.

## Removed

- Recursive midpoint IK refinement.
- Trace downsampling and reference reindexing for iterative repair.
- Harmonic-quintic trajectory generation and iterative local/global retiming.
- Velocity-scale audit retries and duplicate legacy validation entrypoints.
- Strict constant-cruise generation and the unused retiming sample-spacing flag.
- Unused mount candidate search, duplicate legacy Config and unused static viewer.

## Organization

New trajectory construction lives in motion_planning/fixed_path.py; numerical
geometry lives in infrastructure/fixed_spline.py; local clock allocation remains
in infrastructure/local_speed.py. Version-1 playback is isolated in
infrastructure/legacy_trajectory.py. Existing NPZ keys and the constant_trace
Python field retain their historical names for artifact compatibility.

The local speed-profile flag remains accepted for current commands, but constant
is rejected. Historical configurations migrate on read without changing the
saved trajectory clock. New timing-cache profiles use version 3 after removal
of their unused sample-spacing setting.

Regression coverage retains interpolation, rates, collision rejection, exact
endpoint clocks, serialization, provenance and pipeline routing. Tests specific
to deleted algorithms are removed or migrated to the current implementation.

## Full Pipeline Validation

Output: `outputs/pipeline/middle_mainline_cleanup_90s/tracking`.
Automatic registration, 20-degree jaw opening, fixed angles (55, -20, -140),
2 mm trim, 2.5 mm offset, 0.05 mm pose spacing, 90 s target,
1 mm/s cleaning cap, 150 mm/s free cap, diagnostic orientation auditing.

- 1141 original pose nodes; 1143 total waypoints including entry/exit.
- Entry 3.040020 s; cleaning 86.772480 s; retreat 0.187500 s; total 90.000000 s.
- Minimum actual clearance 0.889480 mm; dynamics completed; execution_ready=true.
- Pipeline wall time 83.68 s, including 76.46 s planning and 54.04 s validation.
- The completed dynamics run was reused; no second pipeline simulation.
- Previous version-2 TOPPRA and legacy quintic validated artifacts both loaded.
- 106 planning tests and 172 project tests passed (278 total).
- The small retiming integration test verifies unchanged spline coefficients,
  retained pose exports, serialized playback, and exactly one dynamics call.

The existing orientation diagnostic policy is unchanged. This verifies the
non-contact MuJoCo workflow; this cleanup makes no algorithmic speed or audit
performance change.
