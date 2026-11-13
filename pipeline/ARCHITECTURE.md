# Architecture

The pipeline owns orchestration and stage lifecycle only. Perception produces a
registered gingival topology, planning converts that topology into a validated
joint trajectory, and control executes the trajectory with collision and
tracking guards. Stage adapters in `pipeline/stages` expose the same operations
for isolated simulation debugging.

The persistent output contract is `pipeline_report.json` plus stage-owned
artifacts. `--no-output` uses the same stage calls with a temporary workspace;
`--dry-run` stops after validating the jaw, scan, model, and registration
configuration, mandatory regional dataset and matching prediction provenance.

`pipeline.parameters` loads an explicit `--params` JSON using the CLI argument
schema. File values override program defaults; explicit CLI options override
file values. Types, choices, unknown keys and cross-source mutual exclusions
are checked before `run_pipeline`. Relative JSON paths use the project root.
The existing `--config` camera/registration selector and Python API are unchanged.

`run_pipeline` and `run_tracking` default to regional fixed orientation with
bounded five-degree corrections (`planning_mode=simple`). Perception creates
continuous region contracts before tracking. Tracking passes the selected
`PreparedRegion` to `planning.pipeline.run`; geometry construction belongs to
perception, while pose search, IK, timing and auditing belong to planning.
The explicit `continuous` mode retains local gravity/normal orientations.
After planning, tracking requires an executable result and
rechecks the saved continuous reference and one-sweep contract before starting
control. Planning or contract failure stops execution. Temporary runs use the
same fixed-angle configuration; dry runs and persistent reports identify the
continuous workflow.

`tilt_deg` and `tilt_azimuth_deg` pass through the same pipeline/tracking calls
into the planner's `Config`, including temporary runs. Dry-run and persistent
reports include both values. Pipeline validates them before starting perception;
planning owns the local orientation convention and independent replay checks.
