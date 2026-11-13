# Motion Planning

Consumes the dense, continuous IK nodes exported by pose search. The only new
trajectory strategy is a fixed natural joint cubic with TOPPRA/ECOS local speed
allocation and a C2 scalar clock.

- `continuous.py` connects approach, one uninterrupted cleaning block and final retreat.
- `fixed_path.py` fits the joint path once and allocates time; `plan_pose_sequence` is the small Python API.
- `audit.py` compares actual timed FK against the saved geometric reference.
- `validation.py` performs independent geometry/motion/collision audits and one dynamics run.
- `timing.py` re-clocks a saved trajectory while preserving all its nodes.
- `execution.py` checks provenance and scene consistency, caches validated timing and replays in MuJoCo.
- `contract.py` checks trace continuity, reference mapping and tooth coverage.

Pose sampling remains independent: `--pose-spacing-mm 0.05` belongs to pose
search. The motion stage never repairs geometry by requesting IK points,
downsampling, splitting intervals or refitting its curve.

A target duration includes entry and exit. Their separate TCP speed cap keeps
them fast; spare time slows cleaning. A target below the certified minimum
fails. Geometric or collision failure also stops planning.

Saved-plan retiming retains the source joint velocity, joint acceleration and
angular velocity limits. It changes only the clock and the requested TCP speed
caps. Timing profile version 4 invalidates caches made by the earlier policy
that silently increased those limits.

Use `planning.view` to inspect the saved result. `planning.execute` defaults to
90 seconds and reuses matching validated output; `--original-timing` replays the
saved clock. Old artifact readers remain isolated from new plan generation.
