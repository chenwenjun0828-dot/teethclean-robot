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

`standalone_pack` statically flattens the authoritative contract, runner, ROS
transport and `standalone_runtime` into one readable Python delivery. It packs
original export bytes and deployment configuration into one versioned JSON by
default. The optional plain format emits only joint names and timed joint points;
the paired script holds editable site configuration and fixed TCP/phase/source
reference data. `standalone_plain` reconstructs the same JointTrajectory and
provides the shared runner's optional input adapter, leaving the original bundle
loader and project CLI defaults intact. Source bytes/snapshots are verified at
build time; runtime separately checks the plain JSON, reference binding and
script integrity. Original source hashes remain provenance; explicit new binding
fields prevent an existing bundle approval from silently authorizing migration.
Only the standalone entry injects the developer-approved geometry replay gate;
the project CLI retains full source checks. The shared runner accepts an explicit
source gate and a pre-send input integrity callback, while state/force/watchdog
guards remain shared. Source restrictions waived by standalone replay are logged,
never rewritten as successful validation. The user-requested delivery is under
`control/employment/2026_11_16/`; generated files are rebuilt rather than edited.

Standalone speed selection scales the saved curve in memory before constructing
the ROS backend. The shared runner's optional curve transform returns a curve,
offline blockers and report details; both the goal and monitoring use that curve.
The original source/authorization checks still bind the original bytes. Speedup
requires explicit site joint velocity/acceleration limits, checked with the same
Bernstein bounds used by the exporter, now owned by `trajectory_contract`.
ROS timestamps and monitor references share the rounded nanosecond clock;
wall-clock guards and force thresholds do not scale. Full-policy retiming is
blocked pending a new audit/export. The original project CLI uses no transform.

`standalone_keyboard` owns Linux terminal lifecycle and thread-safe start/stop
events, with no robot communication. The standalone execute entry can inject it
into the shared runner; other callers retain their previous behavior. The runner
maintains pre-start state checks/heartbeats, rechecks authorization and input
integrity after S, and uses the existing cancel/stop/velocity confirmation for
Space or detected input failures. Operator termination is recorded separately
from successful trajectory completion. Keyboard mode never retimes the curve.

The current dated delivery selects the explicitly approved minimal MoveIt backend.
`moveit_runtime` reads only plain joint points, uses the workstation's existing
MoveIt group/model/controller configuration, and sends one ExecuteTrajectory action.
This is the action used by MoveGroupCommander.execute; using its client directly
keeps per-goal result and cancellation status available without blocking keyboard
monitoring. It never plans an extra start/home move. Completion checks final joints
and fresh stopped state; cancellation additionally requires a terminal action and
new stopped samples. No TCP/phase/source data arrays or deployment dictionary are
embedded. The builder includes only the needed shared numerical/message helpers
and keyboard code, plus a SHA256 binding. Original protected builders and the
project CLI defaults remain available. The minimal client does not claim the
protected backend's force/TCP/watchdog monitoring or contact validation.
