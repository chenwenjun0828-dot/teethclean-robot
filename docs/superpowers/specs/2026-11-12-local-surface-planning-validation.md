# Local Surface Planning Validation

Date: 2026-11-12. Angles: `[50, 0, 50]` degrees. Mode: `local_surface`.

## Conditions

- Prepared region: `outputs/tutorial/manual/regions/outer_front`.
- Teeth: 12, 11, 10, 9, 1, 2, 3 (dataset labels).
- Jaw opening: 20 degrees; non-contact standoff: 1.8 mm.
- Pose spacing: 0.25 mm; 430 pose nodes.
- Clearance requirement: 0.15 mm; joint step limit: 1 rad; orientation step: 30 degrees.
- Each local angle correction is restricted to +/-5 degrees.
- Original perception curve and blended tooth-plane normals are preserved.

## Results

| Check | Result |
| --- | --- |
| Viewer local frame vs planner, registered-world transform | Passed |
| Reachability, connected IK and motion-rate audit | `geometry_only_verified`, never execution-ready |
| Full planning status | `continuous_outer_front_sweep` |
| Simulation readiness | `execution_ready=true`, simulation only |
| Angle corrections selected | `[0, 0, 0]`; nominal tuple is feasible |
| Source coverage | 96.3361% |
| Total duration, entry and exit included | 99.7090 s |
| Cleaning duration | 62.3420 s |
| Reference trajectory minimum clearance | 0.371008 mm |
| Simulated actual minimum clearance | 0.370809 mm |
| Independent trace maximum position error | 0.000938 mm |
| Independent trace maximum rotation error | 0.000405 degrees |
| Actual local angle deviation, Z/Y/X | 0.000292 / 0.000406 / 0.000234 degrees |
| Dynamics maximum position tracking error | 0.058469 mm |
| Dynamics maximum rotation tracking error | 0.015216 degrees |
| Saved executable plan reload | Passed, 436 trajectory waypoints |

The independent reference audit sampled 3866 trace states. The motion/collision
audit sampled 4991 trajectory states. The dynamics run completed successfully.
These results apply to the conditions above; contact mode with zero standoff
requires its own planning and validation run.

## Regression and Viewer Checks

Planning regression suite: 116 tests passed. Selected planning parameters,
pipeline, reachability, motion and contact integration suite: 90 tests passed.
Coverage includes nominal/local/common corrections, impossible correction bounds,
world gravity, geometry preservation, reference reload and mode integrity,
geometry-only behavior, diagnostic-mode FK bounds and pipeline execution contracts.

Playwright checked desktop 1440x1000 and mobile 390x844. The selected mode and
angles, changing local rotation along the path, mode switching and camera drag
passed. Both canvases contain rendered geometry (935 and 465 sampled RGB colors,
respectively); no page errors, horizontal overflow or clipped mode button text.
Screenshots and raw results are in `outputs/local_surface_validation/`.

## Reproduce

```powershell
$py = ".\.venv\Scripts\python.exe"
& $py -m planning --params config/planning_local_surface.json --out outputs/planning/my_local_surface
& $py -m planning --params config/planning_local_surface.json --check-mode geometry_only --out outputs/planning/my_local_reachability
& $py -m planning.visualization.pose_angles --params config/planning_pose_angles_local_surface.json
& $py -m planning.view --plan outputs/planning/local_surface_50_0_50 --paused
```

Completed artifacts: `outputs/planning/local_surface_50_0_50/` and
`outputs/planning/local_surface_50_0_50_reachability/`. Each new run must use a
new empty output directory.
