# Simulation Assets

`scene_ur7e_dental.xml` is the active MuJoCo scene. Its relative includes use
the adjacent robot, teeth, dental-chair, and mouth-opening files. Keep these
relative paths together when copying a simulation asset set.

The default `robot_mount` position is
`[0.6959828478597841, -0.9912957693238451, 0.77]` metres in world coordinates.
This moves the arm horizontally 150 mm toward the dental model from the former
`[0.815, -0.9, 0.77]` position, preserving mount height and rotation. The position
is defined in the scene XML, so native MuJoCo, `pipeline.run`, and standalone
planning share it unless an entry point explicitly restores or overrides a
saved mount position. Jaw opening and planning sample spacing are independent
settings. Changing the scene requires new planning and validation for execution.

The scaler mounting in `robot/ur7e_with_tool.xml` uses the force sensor's
local +Z axis and a 40.5 mm mounting offset. The `end_scaler_link` orientation
is identity, a 180-degree roll from the former mounting. Its mesh, inertia,
TCP, force site and approximate camera move together. Robot joints and the
sensor mounting remain unchanged. Plans generated with the former mounting
need new planning and validation; archived hand-eye calibration is unchanged.
