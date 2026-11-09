# Continuous Planning and Phase-Aware Speed Design

## Context

The supplied plan is already a continuous middle sweep: its request and report
use `path_kind=continuous_middle_sweep`, with one trace block and no retreats or
transfers inside the sweep. Its 12,004 waypoints are sampled at 0.005 mm and the
trajectory builder imposes a 0.04 s minimum interval, so the dense trace spends
about 480 s on waypoint timing alone. The desired completion time is about 90 s,
with free-space approach/transfer/retreat motion faster than the cleaning trace.

## Goals

- Make the standalone `planning` entry point prefer continuous planning while
  retaining an explicit segmented debug mode.
- Preserve the current continuous sweep geometry, tooth ordering, and validated
  approach/transfer structure.
- Reduce dense trace waypoints only when the retained interpolation remains
  within the existing position, orientation, joint-step, and collision limits.
- Assign separate timing limits to trace/surface-transfer and free-space phases.
- Produce an execution-ready plan around 90 s only after independent FK,
  collision, rate, and MuJoCo dynamics audits pass.
- Fail with the measured minimum achievable duration when the requested target
  cannot satisfy the saved safety limits.

## Design

### Planning mode

`planning.__main__` will expose a mutually exclusive continuous/segmented mode,
defaulting to continuous. `--segmented` remains available for debugging and
legacy comparisons. Existing explicit `--continuous` calls remain valid.

### Trace reduction

The fast-plan builder will operate on an already validated continuous plan. It
keeps the home/approach and transfer nodes, the first and last node of the
continuous trace, and every phase/tooth boundary. Interior trace nodes are
selected by cumulative TCP arc length plus joint/orientation change thresholds.
The candidate is rebuilt as a C2 trajectory and audited against the original
continuous sweep reference; if any interval exceeds the existing interpolation
error or collision limits, the offending interval is split and retried. This
keeps the saved sweep contract consistent rather than silently dropping its
parameter mapping.

### Phase-aware timing

The retimer uses a slower nominal speed for `trace` and `surface_transfer` and a
higher speed for `ingress`, `transfer`, `retreat`, and other free-space phases.
Joint speed, joint acceleration, orientation speed, and exact polynomial extrema
remain hard limits. The target duration defaults to 90 s for the fast execution
profile; timing is scaled only after the local phase limits pass. The result is
written as a separate fast-plan directory so the original validated artifact is
unchanged and its fingerprints remain meaningful.

### Execution interface

`planning.execute` will accept a target-duration/profile option and will report
whether it executed the original or a generated fast plan. The existing jaw
opening override is propagated to the retime validation simulation. The command
will not bypass `load_verified`; the generated plan must have fresh report,
trajectory, trace, motion, and dynamics fingerprints before execution.

## Error handling

- A continuous plan that cannot be reduced without exceeding geometry or
  collision tolerances remains unmodified.
- If the hard rate limits require more than 90 s, execution stops with the
  required minimum duration and the limiting audit values.
- Missing or stale source fingerprints, mismatched sweep contracts, and failed
  MuJoCo contact checks remain fatal.

## Verification

- Unit tests cover default continuous routing, explicit segmented routing,
  trace-node reduction boundaries, phase-specific duration allocation, and
  target-duration rejection.
- The supplied `fixed_55_m20_m141_mount150_open20_dense005_20261109` plan is
  used as an integration fixture. Its fast output must retain one continuous
  trace block, all four teeth, positive minimum clearance, passing FK/trace and
  motion audits, and a completed MuJoCo execution at or below the 90 s target.
- The final command output records the selected plan directory, duration, phase
  timing, and audit status.
