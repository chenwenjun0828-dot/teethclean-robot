# Local Contact Support Repair

Date: 2026-11-12

## Scope and Evidence

Repair the conservative working-contact false rejection authorized in chat.
Keep the original working partition, loaded jaw triangles, source line, tool
angles, contact tolerance, nonworking clearance and self-collision rules.
Per-position angle search remains disabled in the requested reproduction.

At layer 115, parameter 4.053613 mm, the old 2 mm support included 1201 jaw
faces. Its 60 conflicting normals belonged to faces at least 1.196349 mm from
the working AABB. A 1 mm support certifies this same pose with a positive
directional working gap of 0.005762 mm. The original rejection did not prove
excess penetration or defective mesh winding.

## Certificate

1. Retain the existing whole-working-surface heightfield proof as the fast path.
   Retry support radii from 2 mm down to 0.0625 mm.
2. If a common chart fails, choose individual working-triangle chart directions
   from original nearest-face normals at its centroid and vertices, including
   their average. Open3D proximity selects directions only; all acceptance
   checks use the original float64 triangles.
3. Exclude projections only with conservative separating-axis tests. Use AABB
   distance bounds first, then unsigned FCL triangle distances if necessary.
   Adaptive exact-distance breakpoints prevent a coarse radius grid from
   skipping a usable support. Unsigned distances never supply the contact sign.
4. Require consistent winding, orientation, no overlapping support sheets,
   complete projected working-face coverage and exact barycentric clipping.
   Bound the absolute directional length of every complete tool-to-support
   segment by its support radius. Every omitted face is then separated from
   those segments by distance or projection; no interacting face is hidden.
5. When one chart still cannot cover a curved neighborhood, partition a working
   triangle into four complete children, at most two levels. Every child must
   independently pass the same analytic certificate. This is a complete face
   partition, not sampling. Holes, intersections and ambiguous relevant support
   remain rejection conditions.
6. If local verification fails, use original whole-jaw FCL nearest points only
   to propose a separating direction. Require a projection gap over 2 mm
   between all working vertices and ALL original jaw vertices along this
   direction. This also excludes the jaw convex hull and cannot permit a
   deeply enclosed tip merely because unsigned surface distance is large.
   Certify no nearby contact without a signed-gap claim. This handles large
   overlapping jaw AABBs whose actual surfaces lie far to one side.

The original-mesh contact backend identity changes to
whole_triangle_oriented_heightfield_v3. Existing manifest comparison therefore
requires replanning for artifacts from the previous backend.

## Search Cost

Mandatory pose probes need only one accepted IK branch to pass their necessary
existence test. Stop there and promote a failed probe for the next common-angle
candidate. Preserve angle candidate order, all graph IK branches, complete node
checks, sampled edges, trajectory audits and dynamics acceptance. Record failed
probe layer, parameter and angle correction in the planning report.
If a dense node fails, test all returned IK branches at that same layer; an
entirely invalid bank proves a cut immediately, without downstream graph queries.

## Verification

Cover irrelevant folds, overlapping bounds but separated triangles, intersecting
folds, opposing sheets, curved support with opposite local directions, holes,
boundary crossing, wrong-side gaps and excessive penetration. Reproduce six
previously blocked poses using actual loaded assets and fixed registered joint
states. Keep the real layer-1041 out-of-tolerance pose rejected. Run full planning
and external contact/control regressions, then repeat zero standoff with 0.2 mm
contact tolerance and local angle search disabled.
