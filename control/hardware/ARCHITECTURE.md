# Architecture

The protocol carries SI units: radians, metres, seconds, newtons, and
newton-metres. Control policies own limits and fault handling; backends only
translate commands and samples. A hardware deployment must supply a transport
to `URRobotBackend`, making the real connection explicit and testable.
