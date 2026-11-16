# Local Surface Planning

Add `planning_mode=local_surface` alongside `simple` and `continuous`. The user
approved retaining the simple planner's +/-5 degree correction policy.

At each perception path position, construct the same gravity-projected local
surface frame used by `planning.visualization.pose_angles` in continuous mode.
Interpret the requested Z angle, Y azimuth and tool-X flip in this local frame.
The initial validation tuple is `[50, 0, 50]` degrees.

Reuse bounded angle search and whole-path IK selection. Try the requested tuple,
then one common correction tuple, then smoothly varying corrections. Each
correction remains within +/-5 degrees of the original tuple at that position;
changes in the underlying surface frame do not consume this correction budget.
Keep configured adjacent orientation and joint limits. Do not change the
perception curve, normal blending, tool geometry or collision policy.

Save the local policy, world gravity, original angles and correction knots in
`local_surface_orientation`. Loading verifies the mode, policy, angles, world
transform and geometry provenance. Independently recover actual FK angles in
the local frame at the matched spatial progress, enforcing the +/-5 degree
limits even when orientation auditing is diagnostic. Execution requires this
local FK bound verification.

Expose the mode in standalone planning, pipeline parameters and the angle
viewer. Preserve simple as the default and continuous as strict local angles.
Provide separate planning and angle-viewer parameter files for `[50, 0, 50]`.

Validate geometry preservation, registered-world gravity, local angle recovery,
bounded collision recovery, no-collision diagnostic mode, saved reference
integrity, independent FK bounds and CLI/pipeline propagation. Run actual
prepared outer-front data with the existing 1.8 mm non-contact offset and a
20 degree jaw opening. Report reachability and full collision/dynamics results
separately; no successful preview or geometry-only run grants execution readiness.
