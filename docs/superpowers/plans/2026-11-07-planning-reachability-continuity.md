# Planning Reachability and Continuity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Improve middle-region planning by selecting a reachable robot mount position, allowing 5°–30° tangent-to-plane geometry, choosing smooth IK branches, and using the smallest collision-safe retreat.

**Architecture:** Keep geometry and the physical TCP frame unchanged. Add mount placement to simulation setup and planning configuration, evaluate pose candidates with a dynamic-programming continuity graph, and replace unconditional 15 mm retreat with direct-then-adaptive transition search. Preserve the existing report, audit, control, and collision contracts.

**Tech Stack:** Python 3, NumPy, SciPy, MuJoCo, FCL, unittest, existing planning/control pipeline.

## Global Constraints

- TCP +X remains exactly aligned with the gingival curve tangent.
- The measured tangent-to-tooth-plane angle is accepted only in the inclusive range 5°–30°; 15° is a preference score.
- Every selected pose and transition must pass IK, joint limits, full mesh clearance, and sweep checks.
- Runtime mount changes must be recorded and verified by control; the XML asset is not rewritten.
- Existing user and prior-rebuild changes remain intact.

---

### Task 1: Add mount and angle configuration contracts

**Files:**
- Modify: `planning/io.py`
- Modify: `pipeline/simulation.py`
- Modify: `planning/__main__.py`
- Test: `tests/test_planning_v2.py` and `tests/test_planning_reachability.py`

**Interfaces:**
- `Config.min_plane_angle_deg`, `Config.max_plane_angle_deg`, `Config.max_orientation_step_deg`, and mount-search fields are serializable by `asdict`.
- `create_simulation(..., robot_mount_position_m=None)` applies a runtime mount translation and calls `mj_forward`.

- [x] Add validation for `0 < min_plane_angle_deg < preferred_plane_angle_deg <= max_plane_angle_deg < 90°` (defaults 5°, 15°, 30°), positive orientation-step and search-step values, and bounded search ranges.
- [x] Add runtime mount override to `create_simulation`, preserving the XML default when omitted.
- [x] Add CLI options for explicit mount position and enabling mount search; pass them through `run`.
- [x] Add tests for config rejection, mount override persistence, and unchanged default simulation.
- [x] Run `python -m unittest tests.test_planning_v2 -v`.

### Task 2: Build reachable mount candidate search

**Files:**
- Create: `planning/mount_search.py`
- Modify: `planning/__main__.py`
- Modify: `planning/planner.py`
- Test: `tests/test_planning_v2.py` and `tests/test_planning_reachability.py`

**Interfaces:**
- `search_mount_positions(center_m, cfg) -> list[np.ndarray]` returns deterministic coarse-to-fine candidates.
- `apply_mount_position(model, position_m, data=None) -> None` updates `robot_mount` and forwards MuJoCo kinematics.
- `choose_mount_position(center_m, cfg, score)` returns the highest-scoring fully checked candidate and diagnostics.

- [x] Implement bounded coarse grid and local refinement around the best candidate, including the current position as the first candidate.
- [x] Score candidates by valid trace arc length, retained-tooth count, continuity cost, and minimum clearance.
- [x] Ensure failed candidates are diagnostic-only and cannot become the final report.
- [x] Record selected position, candidate count, and score in planning report/request metadata.
- [x] Add deterministic unit tests with a synthetic scorer and a real model mount-application test.

### Task 3: Replace fixed pose branch selection with continuity graph

**Files:**
- Modify: `planning/poses.py`
- Modify: `planning/planner.py`
- Modify: `planning/audit.py`
- Test: `tests/test_planning_v2.py` and `tests/test_planning_reachability.py`

**Interfaces:**
- `pose_at` rejects measured angles outside the configured inclusive range.
- `evaluate_branches` returns a selected continuous candidate sequence with per-sample roll and IK branch metadata.

- [x] Enumerate all roll angles and valid IK solutions at each curve sample instead of fixing one roll for the whole stroke.
- [x] Add transition checks for joint step and TCP rotation step; use dynamic programming to minimize joint change and roll penalties; record the geometrically fixed plane angle.
- [x] Refine selected edges with intermediate IK until geometric interpolation uses at most 20% of the position/angle error budget; collision-blocked graph edges trigger alternative branch search. Infeasible graph gaps remain explicit splits.
- [x] Preserve `trace_edge` collision and FK interpolation validation for every selected edge.
- [x] Export actual angle range, configured orientation-step limit, selected roll sequence, and rejection reasons.
- [x] Add tests for inclusive 5°/30° acceptance, out-of-range rejection, and a synthetic graph that chooses a smooth non-greedy branch.

### Task 4: Implement direct-first adaptive transitions and continuous trace timing

**Files:**
- Modify: `planning/planner.py`
- Modify: `planning/trajectory.py`
- Modify: `planning/README.md`
- Test: `tests/test_planning_v2.py` and `tests/test_planning_reachability.py`

**Interfaces:**
- `adaptive_transition(...)` returns a dictionary containing `segments`, `retreat_mm`, `method`, and `clearance_m` after trying direct, 2, 5, 10, and 15 mm paths.
- `Trajectory` preserves serialized interpolation metadata and reports whether trace nodes are continuous.

- [x] Try direct connection between current endpoint and next ingress endpoint before any retreat.
- [x] If direct fails, solve and check retreat/transfer/ingress at ascending distances; use the first valid distance and record it.
- [x] Keep a stopped boundary at true retreat/transfer actions, while trace blocks use a C2 continuous interpolant with nonzero internal waypoint velocity.
- [x] Extend trajectory save/load and limits reporting without breaking existing old-plan rejection.
- [x] Add tests proving direct transition avoids retreat and blocked direct transition selects the smallest valid retreat.
- [x] Run focused trajectory and transition tests.

### Task 5: Replan, audit, and run control simulations

**Files:**
- Modify: `control/position.py`
- Modify: `planning/README.md`
- Create: `tmp/run_verified_reachability.py` and `tmp/select_verified_mount.py` for reproducible validation
- Outputs: `outputs/planning_v2/continuous_final_mandibular_02/`, `outputs/planning_v2/continuous_final_maxillary_02/`

- [x] Run the full test suite and `git diff --check` (97 tests passed).
- [x] Generate new mandibular and maxillary plans in new output directories with the new defaults.
- [x] Verify independent trace audits, 5°–30° angle ranges, selected mount positions, minimum clearance, and reduced retreat count.
- [x] Run headless control simulations for both plans and inspect execution reports for tracking and collision failures.
- [x] Update README command examples and summarize any remaining geometry-rejected regions.

## Implementation decisions

The bounded screen uses 100 mm coarse spacing and 20 mm local spacing; local candidates are clipped to the original ±100/±50 mm box. Two spatially distinct screen leaders and the original mount receive full motion validation. Both single-point screening and validated-trajectory scores are retained; this is not a global-optimum claim. The 15° reference cannot choose a different angle at the same point while X remains exactly tangent. Actual C2 polynomial extrema bound joint speed/acceleration; the sum of joint angular speeds bounds tool rotation to 15°/s. Interior FK and mesh checks audit the same polynomial consumed by control. Failed smooth joins are retried with smaller shared waypoint velocities, never silently exported.
