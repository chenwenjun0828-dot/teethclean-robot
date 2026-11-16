# Working-Segment Contact Implementation Plan

**Goal:** Permit ideal working-tip contact in existing planning while preserving
prohibited-part and robot collision checks.

**Architecture:** A fixed ContactPolicy and conservative original-mesh partition
are owned by collision infrastructure. Config/CLI, trajectory validation and
runtime execution share the policy and separately report working contact and
prohibited-part clearance.

**Tech Stack:** Python, NumPy, SciPy, python-fcl, Open3D, MuJoCo, unittest.

## Global Constraints

- Working sphere radius: 1 mm by default, TCP centered, all face vertices inside.
- Working contact tolerance: 0.05 mm, never silently increased.
- Nonworking minimum clearance: configurable, default 0 mm; still reject touch.
- Inspect both complete jaws and all forbidden parts; retain sampled edge checks.
- Reject ambiguous contact verification; FCL collision markers are not depths.
- Preserve source geometry and legacy saved-plan semantics.

## Task 1: Collision Policy and Geometry

Files: planning/infrastructure/contact.py, planning/infrastructure/collision.py,
tests/test_working_contact.py.

- [x] Implement validated ContactPolicy and original-face partition/provenance.
- [x] Implement locally oriented contact checking with explicit unverifiable
      results and comprehensive original-face intersection checks.
- [x] Add CollisionChecker(contact_policy=None); preserve the existing checker
      behavior without a policy. Check.clearance_m remains prohibited clearance;
      append optional working_gap_m and contact_verified measurements.
- [x] Verify real TCP partition counts and analytic geometry boundary cases.

## Task 2: Configuration and Pipeline

Files: planning/config.py, planning/__main__.py, planning/pipeline.py,
pipeline/run.py, pipeline/tracking.py.

- [x] Add working_segment_radius_mm=1, contact_tolerance_mm=0.05 and
      nonworking_clearance_mm=0; enable policy in contact motion mode.
- [x] Restore missing legacy contact_tolerance_mm as None to preserve old rules.
- [x] Add matching CLI flags and API propagation, including recursive no-output.
- [x] Construct checker from cfg.tool_contact_policy() and save its identity.

## Task 3: Validation and Execution

Files: planning/infrastructure/audit.py, planning/motion_planning/validation.py,
planning/motion_planning/execution.py, planning/motion_planning/timing.py,
control/position/controller.py.

- [x] Accumulate minimum working signed gap separately from forbidden clearance.
- [x] Require contact policy verification for saved contact-mode execution;
      retain legacy positive-clearance acceptance for old plans.
- [x] Save/restore/compare partition and policy provenance on load and retime.
- [x] Check new config, audit, runtime and tamper cases with unittest.

## Task 4: Documentation and Regression

Files: planning/README.md, config/planning_contact.json and tests as required.

- [x] Document current FCL/MuJoCo behavior, new contact rules and their scope.
- [x] Supply an explicit zero-offset contact configuration and dry-run example.
- [x] Run focused geometry tests and existing planning/control regressions.
- [x] Inspect final diff and report actual validation limits.
- [x] Run simple planning with angles [50, 0, 50], zero standoff, 0.05 mm
      contact tolerance and 0 mm nonworking clearance; retain actual outcome.
- [x] Also test the user-requested 0.14 mm standoff with the same contact policy.
- [x] Test zero standoff with the later user-requested 0.2 mm working tolerance;
      retain the default 0.05 mm tolerance in code.
- [x] Add a boolean local_angle_search option across config, CLI and pipeline;
      set the contact example false per the user's request, retain legacy default.
