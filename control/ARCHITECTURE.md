# Architecture

Control consumes a validated timed trajectory and dispatches commands through a
`HardwareBackend`. The position stage enforces tracking, collision, and joint
safety; the force stage adds sensor compensation, compliant normal motion, and
hard force guards. MuJoCo and mock backends are deterministic test targets;
`URRobotBackend` is the explicit injection point for a real transport.

Reports use seconds, metres, radians, newtons, and newton-metres internally,
with human-facing error fields converted to millimetres and degrees where
their names state it.
