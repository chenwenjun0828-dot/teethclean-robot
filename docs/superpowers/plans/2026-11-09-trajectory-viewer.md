# Planning Trajectory Viewer Implementation

Approved scope: integrate the demonstrated preview into `planning.view`.

- [x] Inspect existing viewer, verified scene loader, trajectory sampling, and docs.
- [x] Add reusable playback/stepping and export-time sampling helpers.
- [x] Integrate verified scene loading, cached overlays, camera presets, live replay,
      snapshot/GIF exports, and useful CLI diagnostics into the existing entry point.
- [x] Add focused behavior tests and update the planning command documentation.
- [x] Run regressions, inspect current-plan exports, and launch the packaged viewer.

## Validation

- All 90 planning tests passed, including 12 focused viewer tests. The focused
  tests also passed after the final export-font compatibility adjustment.
- The packaged viewer opened with the current verified plan at time 45 s. A
  bounded native-window check exercised Space, N, P, R, state overlays, and
  orderly window closure without running control.
- Exported and inspected `planning_view_snapshot.png` (960 x 720) and
  `planning_view_motion.gif` (768 x 576) under
  `tmp/current_trajectory_visualization_20261109/`.
- GIF has 180 frames and 18.000 s duration at 5x, covering the saved 90 s plan
  from start through end. Sampled frames are nonblank and show changing tool
  poses; `planning_view_motion_check.png` records four checked frames.
- Re-loaded the source plan through `load_verified` after export: all checks
  still pass, duration remains 90 s and waypoint count remains 680.
