# Architecture

The motion stage consumes the pose stage's fixed continuous IK nodes.
`continuous.py` connects entry and exit, `fixed_path.py` constructs the fixed
joint cubic and TOPPRA clock, and `validation.py` audits and simulates it once.
The pose spacing is controlled independently by pose search; motion never
requests more poses, removes nodes or repairs the geometric path.

The handoff to position and force controllers is `Trajectory.sample(t)` plus
the validated report and file fingerprints. `timing.py` can replace the clock
of a saved fixed path; `execution.py` restores its scene and checks provenance.
