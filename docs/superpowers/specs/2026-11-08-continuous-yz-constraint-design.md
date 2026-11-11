# Continuous sweep with directed Y/Z constraints

The user approved adding a Y-axis reference corridor and roll smoothing and
replanning both jaws. X keeps its original 15 degree inward-normal tilt and
projected-tangent direction. Y's zero reference is the outward normal projected
onto the plane perpendicular to X; Z = X cross Y. The model defines positive Y
as the marked radial side of the bend. A configurable fixed roll offset is explicit
metadata, never selected silently to make a failed plan pass.

Initial trial: 60 degree hard corridor, zero reference offset, 5 degree roll grid,
one degree candidate margin for actual-pose audit. Global path search penalizes
squared directed-reference error integrated over distance and squared roll changes
divided by spatial distance. Collision and X constraints remain hard requirements.

Retained coverage limits cap local trimming so alternative connectors can be tested
without cutting a short tooth below minimum. Search failure remains all-or-nothing.

Full FK audit checks actual Y/Z deviation in addition to original pose constraints.
Timed trajectory audit also checks physical spin about TCP X: at most 60 deg/s and
120 deg/s^2. These are physical angular velocity and acceleration projected onto X,
not the derivative of roll coordinates in a moving tooth frame. Ordered revolute
joint axes provide exact instantaneous angular kinematics; checking remains sampled.
Time scaling enforces these rates and must be rechecked after serialization.

Reference, request and report bind the same policy. Old references with no policy
remain readable. New constrained plans cannot load without Y/Z and spin audits.
Test geometry, sign, wraparound, graph costs, real MuJoCo angular kinematics and
independent rejection of axis violations. Replan original jaw data and run headless
dynamics for successful candidates; record failures and changed coverage honestly.
