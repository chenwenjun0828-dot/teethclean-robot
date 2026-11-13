# Continuous Planning and 90 s Execution Validation

Date: 2026-11-09. All execution below is non-contact MuJoCo simulation.

## Source and output

Source: `tmp/fixed_55_m20_m141_mount150_open20_dense005_20261109/tracking`.
The source was already `continuous_middle_sweep`, with one trace block,
all four teeth, no internal retreat/transfer, and duration 507.145709 s.
Its files were preserved.

Validated output:
`tmp/fixed_55_m20_m141_mount150_open20_dense005_20261109/tracking_90s_e81d0ad342`.

The original command now selects this cached output after verifying source,
geometry, scene, trajectory, sweep-reference and dynamics-validation records:

```powershell
.\.venv\Scripts\python.exe -m planning.execute `
  --plan tmp/fixed_55_m20_m141_mount150_open20_dense005_20261109/tracking `
  --jaw-opening-deg 20
```

Add `--headless` for execution without a viewer. `--original-timing` selects
the original 507 s artifact. Initial generation/audit time is separate from
the 90 s simulated movement. Headless execution is not wall-clock paced.

## Measured results

| Measure | Result |
|---|---:|
| Planned duration | 90.000000 s |
| Transfer | 3.075000 s |
| Ingress | 0.093510 s |
| Continuous cleaning trace | 86.831490 s |
| Trace sweep length | 40.327972 mm |
| Mean reference progress during trace | 0.464439 mm/s |
| Audited peak trace TCP speed | 0.853917 mm/s |
| Audited peak free-space TCP speed | 148.514846 mm/s |
| Waypoints before / after | 12,004 / 680 |
| Moving internal waypoints | 667 |
| Continuous trace blocks | 1 |
| Internal retreats / transfers | 0 / 0 |
| Trace position error | 0.088734 mm (limit 0.1 mm) |
| Trace orientation error | 0.989494 deg (limit 1 deg) |
| Joint speed maximum | 1.597837 rad/s (limit 2.2) |
| Joint acceleration maximum | 11.963913 rad/s^2 (limit 12) |
| TCP angular-speed bound | 170.570056 deg/s (limit 200) |
| Planned minimum clearance | 0.150851 mm (limit 0.15 mm) |
| Maximum dynamics position tracking error | 0.304788 mm |
| Maximum dynamics rotation tracking error | 0.198570 deg |
| Minimum actual clearance | 0.151809 mm |
| Execution status | completed |
| Simulation including final settling | 90.102000 s |

The trace retains the same four teeth, sweep reference, fixed roll -141 deg,
tilt 55 deg, azimuth -20 deg, mount, jaw opening 20 deg, and non-contact offset
1.5 mm. These clearance values have a small modelled margin; the checks are
finite-resolution simulation checks, not real-robot calibration validation.

## Timing decisions

The 0.04 s minimum interval in the original trajectory builder made the
12,000-node trace slow despite relatively high configured rate limits.
Trace downsampling starts at 0.1 mm spacing, preserves phase/tooth boundaries,
and restores original nodes where actual-FK interpolation checks fail.

The final 680-node plan converged after nine geometry/timing attempts. Cartesian
speed limits are enforced locally; unchanged intervals are not slowed by a
single worst corner. The feasible locally retimed duration was 77.354962 s;
the remaining time was assigned to the trace, preserving free-space timing.

A trial at a 0.7 mm/s peak trace limit required 105.690 s. The final profile
uses a 1.0 mm/s peak limit and reaches an audited peak of 0.853917 mm/s after
allocating the full 90 s budget. This adjustment preserves the user's requested
fast approach/transfer and slower cleaning behavior.

## Checks

- 78 planning tests passed, including 11 new default-mode, reduction, phase
  timing, cache-integrity and failure-routing tests.
- 28 pipeline/reachability tests passed on the final code; automatic registration
  tests also passed in the earlier 38-test pipeline/registration/reachability run.
- Standalone planning dry-run accepted fixed roll without `--continuous` and
  exported non-null continuous sweep options.
- Actual-FK trace and mesh/rate audits passed. Independent MuJoCo dynamics
  validation and subsequent headless execution both completed.
- A read-only cache preparation call verified that the original command reuses
  the validated output rather than regenerating it. Replay results are stored in
  `execution_report.json`; immutable `dynamics_validation.json` retains the
  validation proof and matching report fingerprint.
