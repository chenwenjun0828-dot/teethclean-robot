# Architecture

`constraints` defines the shaft-axial TCP frame and surface orientation.
`continuous` performs dynamic
programming over candidate banks and asks the collision checker to validate
each joint edge. Units are metres/radians internally, with report angles in
degrees. In continuous mode, each path sample has one prescribed orientation and
may have multiple IK solutions. No angle sampling or direction corridor is applied
in that mode.

`Config.tilt_deg` now selects alpha about fixed projected-gravity Z;
`Config.tilt_azimuth_deg` selects beta about fixed Y = inward normal cross Z.
`fixed_roll_deg` is gamma about the resulting tool +X. These field names remain
for API compatibility. Zero rotation has columns [inward n, Z cross n, Z].
The constructor composes fixed-Y beta after fixed-Z alpha, then local-X gamma.
World gravity is transformed into scan coordinates, independently of tangents.
Source reachability and whole-path candidates use this same pose
contract. The motion audit separately reconstructs the expected shaft axis.

With `Config.planning_mode='simple'`, `planning.simple` fixes one regional
frame at an actual principal-source sample near the cumulative arc midpoint.
The same fixed-Z/fixed-Y/tool-X tuple defines the nominal rotation. Prefer a constant
rotation, then a common bounded correction, then local corrections. Each angle
stays within five degrees of its original value in the same anchor frame.
The finite 2.5-degree search reuses whole-path IK/node/edge checks; the local
graph penalizes deviations and rotation changes. Smooth corrections and their
anchor are saved with the original continuous position reference. FK replay
enforces the component bounds even with diagnostic orientation-error auditing.
Old surface-tangent orientation artifacts must be replanned. At alpha +/-90,
inverse angles are coupled; the bounded audit uses the equivalent representation
nearest the original tuple. A zero gravity projection fails explicitly.

`sample_sweep` owns the dense pose spacing (default 0.05 mm), independently of
the geometry reconstruction spacing (0.25 mm). The selected poses, IK joints
and reference parameters are exported before motion construction. Motion does
not request adaptive pose samples or otherwise change this selection.
