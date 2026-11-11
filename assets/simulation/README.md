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
