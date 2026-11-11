# Tool Tilt Azimuth

The user approved adding an independent tilt direction parameter after reviewing
the fixed-roll-190 jaw-opening comparison. Keep the TCP position, path geometry,
normal tilt, roll search/fixed roll, collision checks and timing algorithms.

## Orientation Contract

Add `Config.tilt_azimuth_deg`, default 0 degrees, finite in [-180, 180]. Reject
booleans and nonnumeric values. Keep `tilt_deg` in (0, 90) and expose both
`--tilt-deg` and `--tilt-azimuth-deg` in planning and pipeline.

With outward unit normal n, projected forward tangent t, and b = n cross t:

```text
d  = cos(alpha) t + sin(alpha) b
x  = -cos(theta) n + sin(theta) d
y0 = -sin(alpha) t + cos(alpha) b
z0 = x cross y0
y  = cos(phi) y0 + sin(phi) z0
z  = x cross y
```

Here theta is tilt_deg, alpha is tilt_azimuth_deg, and phi is axial roll.
Alpha=0 exactly preserves the previous formula. Positive alpha rotates the
tilt direction from t toward b about outward n by the right-hand rule. The
zero-roll frame rotates with this tilt direction, preserving the existing
relationship between roll 90 degrees and the projected outward normal.

Changing alpha changes orientation about the fixed TCP. It is local to the
path and tooth frame, not a world-down rotation or a robot joint command.
The maximum shaft-direction change on a fixed tilt cone is twice theta;
larger visible inclinations may also require increasing theta. Screenshot
direction must be demonstrated on the actual scene, not inferred from labels.

## Integration

- Use the new parameter in the canonical pose constructor so source windows,
  continuous and segmented candidates, refinement, and contact experiments
  receive it consistently.
- Pass both tilt parameters through pipeline dry runs, temporary/no-output
  runs, tracking, and persisted request/report configuration.
- Independently reconstruct expected shaft direction in the timed FK audit;
  use the extended orientation constructor for the complete rotation check.
- Verify saved orientation configuration against geometry before loading an
  existing trajectory. Missing azimuth fields retain legacy zero semantics.
- Update static shaft previews and module README/architecture documentation.

## Validation

Test legacy zero orientation, positive/negative cardinal directions, fixed
normal angle across rolls/azimuths, right-handed orthonormal frames, rigid
transform equivariance, unchanged TCP position, invalid inputs, source and
whole-path candidate propagation, policy references, and independent audits
that reject a wrong azimuth with an otherwise correct tilt and position.
Verify both CLI routes and pipeline no-output propagation; run both existing
test suites. Render actual-model comparison images at the user's reference
point with roll 190 and jaw opening 20 degrees. Run the complete planner with
an illustrative nonzero azimuth and report failure honestly if no trajectory
passes; adding a parameter does not establish whole-path feasibility.
