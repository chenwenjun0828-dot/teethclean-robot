# Architecture

The protocol carries SI units: radians, metres, seconds, newtons, and
newton-metres. Control policies own limits and fault handling; backends only
translate commands and samples. A hardware deployment must supply a transport
to `URRobotBackend`, making the real connection explicit and testable.

The separate ROS 1 trajectory bridge also lives here: `ros_export` owns offline
conversion and interpolation audits, `trajectory_contract` owns the portable
bundle, `ros_runner` owns the action lifecycle and guards, and `ros1_trajectory`
owns ROS communication. It consumes the saved timed curve without MoveIt
replanning. It does not implement the point-command `HardwareBackend` protocol:
an entire action, controller progress and an independent watchdog have different
lifecycle requirements. `__main__` is the explicit user entry point; normal
simulation pipelines never construct the ROS backend.
