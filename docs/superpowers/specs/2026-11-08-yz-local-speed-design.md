# Local timing for a 60–90 second constrained sweep

The user requests faster upper and lower trajectories, within 1–1.5 minutes each.
Target 90 seconds including initial approach. Reuse the approved continuous geometry,
roll references, and 60 degree Y/Z corridor. Keep physical axial spin bounds at
60 deg/s and 120 deg/s^2; preserve original source coverage and collision clearance.

Uniform time compression violates axial acceleration. The existing retimer slows
the whole plan for the worst interval. Instead, expose per-interval axial scaling
requirements and grow local duration floors, recomputing shared waypoint velocities
and C2 quintics after each update. Recheck geometry after timing changes because
nonuniform timing can change the joint interpolation curve. Keep a final uniform
scale only for residual/global bounds and the requested total duration.

Use exact joint polynomial extrema and sampled full-FK physical axial kinematics.
Reduce waypoint velocities locally if the interpolant violates geometric constraints.
Reject nonconvergence or a target below validated limits. Export only after complete
independent geometry, axis, clearance, rate and dynamics checks. Tests cover local
slowdown versus global slowdown, C2 continuity, hard rates and unchanged inputs.

Alternatives considered: relaxing axial bounds changes the accepted motion policy;
replanning geometry and roll could reduce coverage. Local timing is the first choice
because it can preserve both the path and policy while meeting the requested duration.
