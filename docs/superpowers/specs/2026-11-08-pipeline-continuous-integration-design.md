# Continuous planning in the simulation pipeline

The user requested connecting the existing continuous planning implementation to
the end-to-end pipeline after identifying that it still invoked segmented motion.
Reuse the existing algorithms, configuration defaults, and independent audits.

`pipeline.run` and `run_tracking` default to continuous planning. An explicit
`--planning-mode segmented` keeps the existing mode available for debugging.
Forward `SweepOptions()` only in continuous mode. Preserve registration frames,
the capture scene, simulation speed settings, temporary output handling, and
headless execution. The continuous reference may trim original source arcs and
join them with synthetic connectors; it is not full-jaw coverage or force control.
Use the existing continuous planner's 1.5 mm non-contact offset as the pipeline
default. Keep explicit offset overrides; 0.8 mm failed continuous planning in the
current lower-jaw scene and must not silently trigger a segmented fallback.

Only one initial approach precedes the sweep. Before control, require an
executable result and validate the saved continuous reference and its structural
contract. A failure stops the pipeline without falling back to segmented motion.
Reports identify the selected mode, actual reference kind, continuity and timings.

Verify mode selection at the CLI, pipeline and planner boundaries, temporary
workspace cleanup, continuous failure and unvalidated-result rejection, and
rejection of transfers hidden inside a sweep. Run existing tests plus a real
automatic-registration continuous pipeline and record the measured outcome.
