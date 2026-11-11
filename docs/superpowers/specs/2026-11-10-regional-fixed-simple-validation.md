# Regional Fixed Simple Validation

The simple planner now uses one regional source frame and the existing
tilt/azimuth/roll parameters. Nominal constant orientation is preferred, then a
common correction, then smooth local corrections bounded to five degrees per
component relative to that original frame. The deterministic search grid is
2.5 degrees; failure is not a proof that the continuous angle box is infeasible.

## Registered Middle-Region Data

Reuse the frozen extraction and registration from
`outputs/pipeline/20261110_151638_957810`, with the original robot mounting,
20-degree jaw opening, 15/0/-120-degree angles and 0.5 mm offset.

- Full collision checks reject the first trace point on the bounded grid:
  `no_regional_pose_within_5_deg`, tooth 10, layer 0. The nominal graph reports
  scaler/mandibular collision. Output:
  `tmp/regional_simple_offset05_20261110_155513`.
- With explicitly disabled collision checks, all 1215 poses have the same
  nominal rotation and a connected IK branch. The complete trajectory is
  90 seconds including 3.236 seconds of approach/retreat. Output:
  `tmp/regional_simple_geometry_verified_20261110_160619`.
- Independent FK audit: 10,926 samples, maximum position error
  0.00004956 mm, rotation error 0.0000000031 degrees. Maximum regional component
  deviations are below 0.000000006 degrees. Geometry, interpolation and motion
  limits pass. This geometry-only result is not execution-ready; dental
  clearance and dynamics were not evaluated.
- The same fixed rotation produces local normal angles from 8.843 to 33.616
  degrees across this region. Regional pose constancy does not guarantee the
  same working angle on every tooth.

## Timing Numerics

ECOS reached its iteration limit at timing grid index 4164/4857 on this fixed
orientation path. Retrying equivalent parameter units (scale 100 instead of
1000) succeeds. The curve, IK nodes and physical limits are identical. The
minimum certified cleaning duration is 41.277 seconds.

## Automated Checks

`python -m unittest discover -s planning -t . -q` covers planning, fixed-path
timing, bounded angle recovery, independent FK rejection, replay and region
selection. Pipeline integration is covered by
`python -m unittest tests.test_pipeline_planning tests.test_pipeline_parameters tests.test_geometry_only -q`.
Two existing planning orchestration fixtures now provide the collision query
counter required by the current pipeline report.
