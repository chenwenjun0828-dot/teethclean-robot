# Pose Search

`pose_search` consumes a continuous path emitted by `perception.regions` and selects one connected IK branch.
It does not fit, split, trim or connect gingival curves.

The supported pose tuple is:

```text
[fixed_z_angle, fixed_y_azimuth, tool_x_flip]
```

The `simple` planner first tries a constant regional orientation. If the full path is not collision-free or
reachable, it searches bounded corrections of ±5 degrees for each angle and then uses smooth local corrections.
Every selected pose still passes joint-step, orientation-step, full-tool collision and trajectory audits.

`continuous` keeps the local surface reference frame and is an explicit alternative. Both modes use the same
IK graph and motion-planning validation.
