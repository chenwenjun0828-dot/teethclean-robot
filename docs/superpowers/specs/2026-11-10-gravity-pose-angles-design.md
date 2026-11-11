# Gravity-Referenced Tool Angles

The user specifies and confirms fixed tooth-plane rotation axes. Retain the
public `pose_angles_deg` triple, now meaning angle alpha about fixed projected
gravity z, azimuth beta about fixed y = inward n cross z, and flip gamma about
the final tool +X. Angles are signed right-hand rotations in degrees.

Project world gravity into the tooth plane and normalize it to z. Following
the user's refinement, tool +Y points down at zero angles. The proper tool
rotation has columns [n, z, n cross z]. The fixed rotation Z axis is zero-pose
tool Y; the fixed rotation Y axis is zero-pose tool Z. Keep these rotation
axes fixed during the first two rotations.
Compose R = Rot(y,beta) Rot(z,alpha) R0 Rot(local X,gamma).

Use world gravity, transformed into scan coordinates with the inverse
registration rotation. Reject a vanishing gravity projection with an explicit
error. Permit zero and signed angles. When alpha is +/-90 degrees the second
rotation and tool flip are coupled; recover the representation nearest the
original regional angles for the bounded FK audit.

Simple mode retains one regional frame and the original +/-5-degree component
box. Continuous mode recomputes the gravity frame at each normal. Save gravity,
new axis/policy identifiers, and angle semantics; old validated pose artifacts
require replanning. The Y-down zero reference uses v3 axis/policy identifiers
and also retires the earlier v2 zero reference. Keep offset positions and
shaft directions independent of this zero-flip reference change.

Update the standalone Three.js viewer and verify Python/browser parity,
zero-pose alignment, rotation order and signs, world transforms, singular
projections, bounded corrections, replay and pipeline parameter propagation.
