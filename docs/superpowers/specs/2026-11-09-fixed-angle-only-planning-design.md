# Fixed-Angle Continuous Planning Only

The user requested removal of segmented motion planning, sampled-roll search,
and the directed Y/Z corridor strategy. Planning now has one motion workflow:
select a left/middle/right region, prescribe three local tool angles, find a
continuous IK branch, connect the initial approach, and audit the timed sweep.

The default angles are `(15, 0, 0)` degrees. The three-value CLI remains the
primary input; individual tilt/azimuth/fixed-roll parameters remain equivalent
ways to prescribe the same fixed strategy. No fallback changes tool angles.

Remove retired strategy implementations, exports, CLI selectors and tests.
Keep initial approach planning, collision checks, coverage checks, timing,
execution validation, geometry-only output, planning.view and visualizations.
Saved fixed-angle continuous plans remain readable, including old unused roll
sampling metadata. Plans using a retired motion/pose strategy require replanning
before execution. Visualizations may retain historical-result rendering.

Update pipeline callers, docs and tests together. Verify all regional fixed-angle
contracts, removed CLI rejection, saved-plan checks and timing. Render an
existing validated continuous trajectory through planning.view after cleanup.
