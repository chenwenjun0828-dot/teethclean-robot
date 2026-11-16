# Planning Architecture

Planning consumes a validated `perception.regions.PreparedRegion` and produces one continuous,
collision-audited robot trajectory.

```text
PreparedRegion
  -> pose_search (fixed regional pose and connected IK)
  -> motion_planning (approach, trace, retreat, timing, audit)
  -> planning_report.json + trajectory.npz
```

When `--perception-session` is supplied, the session loader keeps the
`controller_base` contract explicit and anchors it to MuJoCo `ur_base` with
the fixed ROS base-frame adapter. It also restores the capture-time robot
joint observation:

```text
T_world_controller_base = T_world_ur_base @ Rz(pi)
T_world_scan = T_world_controller_base @ T_controller_base_scan
```

`T_controller_base_scan` is audited by recomputing the calibration and
registration chain `T_controller_base_wrist3 @ T_wrist3_color_frame @
T_color_frame_depth_optical @ T_depth_optical_scan`. The same world transforms
are applied to the static mandibular and maxillary bodies. The session metadata,
adapter, capture joints and anchor are saved in `request.json` and
`planning_report.json`; replay rebuilds the scene from those values and rejects
a changed anchor or obsolete adapter. Session assets are simulation inputs only
and retain their `robot_execution_allowed=false` provenance.

The explicit `--session-camera-frame` policy belongs to the session adapter,
not the generic planning Config. New session CLI/API calls share the perception
default `color_optical`; an explicit `declared` selects the archived stream-frame
chain. `color_optical` treats the archived hand-eye numbers as
wrist3-from-color-optical and substitutes SDK color-optical-from-depth-optical
extrinsics, after auditing the original chain and source hashes. Perception owns
this derivation and transforms published base points/directions consistently;
scan-local PreparedRegion geometry stays unchanged. Planning uses the same
effective scan pose for both jaw bodies and the selected region. The optical
interpretation is explicitly marked as a hypothesis without live-TF verification.

Session records retain both published and effective chains, the interpretation
version and dependency fingerprints. Replay checks both jaw poses, chain matrices,
sources and capture joints against reconstruction. Legacy records without a camera
interpretation version are reconstructed only with `declared`. The optical preview
example is `config/planning_session_optical_preview.json`; it exports geometry only
and makes no reachability or collision claim.

Curve fitting, tooth assignment, endpoint trimming and source continuity across tooth boundaries are perception
responsibilities. The planner never rebuilds those objects and rejects a missing or changed perception
artifact through its file fingerprints.

`simple` is the default pose policy: a regional fixed angle tuple followed by bounded ±5 degree corrections
when the nominal pose is blocked. `local_surface` uses each position's local tooth normal and projected world
gravity, with the same bounded corrections relative to that local frame. `continuous` prescribes strict local
angles without correction search. All policies use the
same IK graph, full-tool collision checker and trajectory validation.

New regional trajectories use `straight_tcp_home_return_v1`: TCP entry from
`FK(robot.home)` to the first trace pose, then return from the final trace pose
to exactly the original joint state. Positions are world-frame lines in meters;
rotations follow shortest-path Slerp. Fixed endpoint IK and a connected layered
search are implemented in `infrastructure/transfer.py`; `motion_planning/free_motion.py`
fits C2 quintic joint polynomials, stretches their clock to meet rate limits, and
refines only free segments when their timed FK deviates from the reference.
Intermediate free nodes do not stop; phase endpoints stop. There is no offset,
normal retreat or RRT fallback in this regional policy.

Schema 8 request/report records fingerprint `transfer_reference.json`.
Validation, preview, execution loading and retiming check the actual interpolant
against its finite line, monotonic progress, Slerp and fixed joint endpoints.
This is sampled verification at the existing interpolation tolerances, not an
analytic continuous-line certificate. Retiming retains normalized free-polynomial
coefficients and scales each complete free segment uniformly. Legacy plans retain
their saved curves and validation rules; `retreat_mm` remains a legacy field only.
Full mode retains collision/dynamics gates; geometry-only outputs stay non-executable.

All planning CLI commands accept `--params config/<command>.json` through the shared
`pipeline.parameters` loader. It derives validation from each command's argparse schema,
merges defaults < JSON < explicit CLI, and resolves JSON paths relative to the project root.
Required paths may come from JSON; cross-source conflicts are checked after merging.
This changes command configuration only, not the saved `PreparedRegion` or trajectory contracts.
