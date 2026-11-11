# Regional Fixed Simple Planning

Replace the old simple two-angle/nearest-plane algorithm with the user-approved
regional fixed-orientation policy. Existing continuous mode is unchanged.

- Select an actual source sample nearest the cumulative principal-source
  midpoint, before trimming or adding synthetic connectors. Save its position,
  tooth, tangent, and outward tooth-plane normal.
- Interpret the existing tilt/azimuth/roll tuple in that one reference frame.
  Keep position reconstruction, surface offsets, and coverage checks separate.
- Prefer nominal orientation on the entire regional trace. If needed, search
  constant corrections first, then bounded local corrections on a deterministic
  2.5-degree grid. Each correction is relative to the original angles, never the
  preceding pose, and cannot exceed five degrees per angle.
- Smooth local angle corrections with quintic interpolation. Keep connected IK,
  full-tool collisions, the joint spline, TOPPRA, and simulation validation.
  Verify actual FK angle bounds even when orientation auditing is diagnostic.
- Save the anchor, original angles, corrections, and search policy. Reject old
  simple references and modified angle metadata on replay. Retired two-angle
  arguments exist only to report migration errors.

Limits: finite candidate search cannot prove infeasibility in the continuous
angle box. The representative frame does not establish clinical working-tip
adaptation on every tooth. Failure never expands the angle box, disables
collisions, or silently changes the offset.

Validation covers constant orientation across changing tangents/normals,
representative selection, bounded recovery, drift and interpolation rejection,
artifact round trips, CLI/pipeline propagation, and real registered geometry.
