# Working-Segment Contact Policy

Date: 2026-11-12

Status: implementation authorized in chat, including the user's revision that
nonworking clearance is configurable and defaults to 0 mm.

## Scope

Apply the contact rules from section 6.2 of the fixed-line optimization proposal
to the existing planning pipeline. Keep simple/continuous pose policies, source
curves, approach/retreat, sampled edge checks, trajectory audits and dynamics.
The node-only optimizer in that proposal is outside this change.

Contact-mode defaults: working radius 1 mm; contact tolerance 0.05 mm;
nonworking clearance 0 mm. Noncontact planning keeps its existing clearance
behavior. Restore legacy saved plans without silently adding contact permission.

## Geometry and Rules

Only ur_end_scaler_geom is partitioned. Convert its loaded mesh to the actual
TCP coordinate frame in metres. A face is working only when all three vertices
are within the configured sphere. All remaining faces, including mixed boundary
faces, remain nonworking. Record original loaded face indices and mesh/transform
identity. Empty working groups fail explicitly.

Current model measurement: 4063 vertices, 8150 faces; at 1 mm there are 48
working faces, 8102 nonworking faces, including 36 mixed boundary faces.

Working faces may contact either complete loaded jaw, independent of tooth
labels. Accept reliable local signed gaps d >= -0.05 mm; preserve raw negative
gaps. Nonworking tool, sensor, mounting and robot/jaw pairs require separation
strictly greater than nonworking_clearance_mm (default 0). Self-contact rejection
remains active. A valid working contact must never hide another invalid pair.

The existing FCL -1 collision marker is not penetration depth. Both loaded jaw
scans are open meshes. Contact checks must use original triangles and explicit
local orientation support; ambiguous topology, side, query or incomplete coverage
fails as contact_unverifiable. Do not use a global inside/outside assertion for
these scans or treat an unsigned distance as a signed gap. Describe the actual
contact verification scope in artifacts.

The implemented backend uses a regular piecewise-planar oriented heightfield.
Clip the complete working triangles in barycentric coordinates against projected
original jaw triangles and evaluate all clipped vertices, including interior
intersections. The reported gap is a conservative local directional bound,
not a Euclidean nearest signed distance. Reject conflicting, folded, overlapping
or incompletely covered support. Whole-jaw AABB separation over 2 mm excludes
contact without making a signed-distance claim.

The v3 follow-up repair in 2026-11-12-local-contact-support-fix-design.md replaces
the single broad support assumption with adaptive local charts and bounded
complete face subdivision. It also permits an all-original-vertex separating
plane gap over 2 mm to certify no nearby contact without a signed-gap claim. All policy
limits and original winding, topology, coverage and overlap checks still apply.

## Integration

Keep work-gap measurements separate from forbidden-part clearance. The checker,
planning audit and runtime controller use one fixed policy. Validation and saved
plan loading accept validated working contact without applying the old global
positive-clearance rule to it. Record and validate the saved partition identity
and policy when loading and retiming; changes require replanning.

Expose working_segment_radius_mm, contact_tolerance_mm and
nonworking_clearance_mm in planning and pipeline APIs/CLI. Validate finite
numbers (no booleans), a positive radius and nonnegative tolerance/clearance.
Changing these parameters does not move the input curve or alter the TCP.
An explicitly selected contact mode may use a positive TCP standoff, as requested
for the 0.14 mm validation case. This offsets target positions along the existing
reference normal and retains the same working-contact policy. Offset inference
and noncontact minimum-clearance rules remain explicit and unchanged.

Expose local_angle_search as a strict boolean in configuration and both CLIs.
Its legacy default is true; the contact example sets false at the user's
request. False disables only the per-position +/-5 degree fallback after all
125 common-angle candidates fail. Common-angle correction and full collision
validation remain active. Report the disabled fallback explicitly on failure.

## Verification

Exercise planar separated, touching, tolerance-boundary and excessive penetration
cases; wrong-side positive unsigned distance; working and prohibited contacts at
the same pose; ambiguous or incomplete local geometry; partition boundary and TCP
transforms; configurable nonworking clearance; saved-policy tampering and legacy
loading. Run existing simple, full-mode, geometry-only and position-controller
tests. Actual front-region infeasibility is a legitimate result and must not be
reported as a successful contact trajectory.
