# Continuous middle-region sweep

The user approved rebuilding the middle path with trimming and reconnection. Implement
one sweep across all four configured teeth, with one initial approach and no retreat,
transfer, re-approach, or disconnected success fragment inside the sweep. Start with
the existing 1.5 mm non-contact mode and retain the existing collision and pose limits.

## Geometry

Keep the original extraction, labels and three-point planes as provenance. Select
ordered principal source strokes, trim endpoints, and join retained source intervals
with quintic curves matching endpoint position and first/second derivatives. Blend
the outward normals smoothly on connectors. All lengths are measured in 3D. Export
the composite path separately; retained original arc length, omitted/trimmed arc
length and synthetic connector length must be distinct. Each tooth must retain at
least the existing minimum coverage. Reject excessive gaps and degenerate tangents.

## Motion

Search a single layered IK/roll graph for the entire composite path, using finer roll
sampling and lazy collision rejection. Failure to reach any layer fails the whole
candidate, never splits it into executable pieces. Try bounded geometry alternatives
and save their outcomes. Refine successful edges against the continuous pose target.
Only the initial approach may use the existing free-space transition planner.

## Export and independent validation

Store a fingerprinted continuous reference with varying normal, source provenance,
parameters and rolls. Export exactly one motion segment, with per-interval tooth
labels. Independently audit the actual timed polynomial with full MuJoCo FK, including
synthetic connectors and blended normals, plus existing collision/rate audits.
Execution loading must reject modified references or a sweep containing transfers.
Existing segmented plans remain readable. Consumers that assume a single tooth and
original spline per segment must reject this new reference until explicitly adapted.

## Acceptance

Meaningful tests cover C2 joins, normal/offset continuity, honest coverage accounting,
disconnected graph rejection, whole-sweep phase integrity and independent FK audit
failure. Run real lower/upper jaw planning and headless dynamics for successful
outputs. Record actual outcomes, including remaining failures; do not claim success
from geometry alone or relabel transfer motion as trace. No hardware execution.
