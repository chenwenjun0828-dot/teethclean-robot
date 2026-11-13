# Dense Poses With Local Speed Reductions: 90-Second Validation

## Accepted Behavior

The user approved local cleaning slowdowns to prioritize completion within 90 seconds.
Keep the independent pose-stage sampling at 0.05 mm. Motion planning must consume
the selected IK nodes without adding pose samples or changing their joint spline.
Approach and final retreat may run faster than cleaning. Orientation-reference
matching is deferred at the user's request; its measured errors must remain visible.

## Implementation

- A natural joint cubic Q(u) passes through every supplied IK node exactly.
- TOPPRA 0.6.3 with ECOS allocates local parameter speeds under joint velocity,
  acceleration, TCP speed and conservative angular-speed limits.
- Each timing cell stays inside one original cubic interval. Smoothing the scalar
  clock, with zero scalar acceleration at timing knots, makes Q(u(t)) C2 without
  refitting Q. Bernstein bounds certify continuous joint velocity and acceleration.
- Cartesian speeds are probed after clock smoothing, with a 0.5% timing margin.
  The exported curve receives the usual independent motion/collision audit.
- An outward, IK-checked 15 mm retreat follows the completed continuous sweep.
  Checked joint shortcuts avoid stopping at every temporary retreat IK sample.
- Entry/exit timing is kept fast. Only the cleaning block is stretched to fill
  the requested duration. Infeasible targets are rejected.
- `--speed-profile local` is the new default. `constant` retains strict TCP cruise.
  Old saved configurations without this field retain their historical constant mode.
- `--orientation-audit diagnostic` explicitly disables only the orientation rejection
  gate. `orientation_passed` and `orientation_enforced` remain separately recorded.
  Strict auditing remains the default. Position, collision and rate limits still apply.

The numerical TOPPRA grid and scalar-clock cells do not create new pose/IK nodes.
The saved NPZ contains version 2 scalar-clock coefficients, consumed identically by
the controller and viewer. Loading checks the clock/method, audit policy, scene and
artifact fingerprints. Timing-cache version is now 2.

## Reproduction

Fresh geometry reconstruction, pose search, motion generation, audits and simulation
on the existing registered inputs:

```powershell
.\.venv\Scripts\python.exe tmp/validate_dense_local_pipeline_20261109.py
```

Output: `outputs/pipeline/dense_local_speed_pipeline_90s_20261109/tracking`.
The script requires a new empty output, so use a different output directory when
repeating. Automatic camera registration was not rerun; the saved registration
and jaw-opening scene were restored and verified.

Parameters: mandibular middle, angles 55/-20/-140 degrees, jaw opening 20 degrees,
trim 2 mm, standoff 2.5 mm, pose spacing 0.05 mm, geometry spacing 0.25 mm,
TCP cap 1 mm/s, entry/exit cap 150 mm/s, joint caps 2.2 rad/s and 12 rad/s^2,
angular cap 200 degrees/s, diagnostic orientation audit.

| Measurement | Result |
| --- | ---: |
| Supplied pose nodes | 1141 |
| Exported waypoints including home and retreat | 1143 |
| Approach | 3.040019952 s |
| Cleaning | 86.772479861 s |
| Retreat | 0.187500187 s |
| Total motion | 90.000000000 s |
| Certified minimum cleaning duration for generated clock | 66.889163524 s |
| Joint speed bound | 1.627066350 rad/s |
| Joint acceleration bound | 7.059868859 rad/s^2 |
| Angular speed bound | 137.950734096 degrees/s |
| Maximum measured trace TCP speed | 0.651994870 mm/s |
| Minimum actual clearance | 0.889479754 mm |
| Maximum position deviation from reference | 0.039057890 mm |
| Orientation-reference deviation, diagnostic only | 6.666710289 degrees |
| Physics tracking position error, all phases | 0.419680199 mm |
| Physics tracking rotation error | 0.199393740 degrees |
| Full planning and validation wall time | 77.122 s |

The 90 seconds refers to commanded motion. The controller's initial settling and
last physics step bring the logged simulation clock to 90.102 seconds.
The conservative minimum is for the generated smoothed clock, not a proof of the
globally shortest possible trajectory. Collision checks retain their existing
finite sampling resolution. This result is a non-contact MuJoCo simulation.

111 planning tests and 24 pipeline integration tests passed. New tests cover
unchanged geometry and nodes, C2 boundaries, finite-difference derivatives,
continuous rate bounds, serialization/stretching, infeasible durations, explicit
orientation diagnostics, and CLI forwarding. `planning.execute --headless` loaded
and reused the completed dynamics proof for the generated 90-second plan.

Algorithm reference: [TOPPRA documentation](https://hungpham2511.github.io/toppra/).
