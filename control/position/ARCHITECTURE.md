# Architecture

`GingivalTCPController` computes position and inverse-dynamics commands for a
MuJoCo scene. `execute` owns the simulation loop and safety checks. `load_verified`
rejects stale plans before execution by checking fingerprints, conventions,
collision audits, and the continuous trajectory audit.

The controller uses metres, radians, and seconds internally. Public reports
convert position errors to millimetres and angular errors to degrees.
