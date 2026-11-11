# Simple Planning Mode Validation

## Original-file equivalence

Executed only the pure geometry/pose functions extracted with Python AST from
the supplied `tooth_angle_control.py`; ROS initialization and execution were
not loaded. Compared 91 nonuniform-direction points against four three-point
planes, using -75/-33 degree rotations and 2 mm offset.

- Maximum position difference: 0 metres.
- Maximum rotation-matrix element difference: 2.220446049250313e-16.
- All nearest plane IDs matched.

## Automated checks

Planning regression suite: 124 tests passed. Pipeline, registration and
planning integration selection: 81 tests passed. After separating the legacy
axis convention, 64 focused mode, audit, saved-reference and pipeline tests
passed. Tests also cover original TXT formats, normal flips, contiguous holds,
world transforms, JSON/CLI precedence and incorrect saved-mode rejection.

## Real regional planning

Tested the current mandibular middle region, 20 degree jaw opening, original
-75/-33 rotations, 2 mm offset, 2 mm trims, 30 degree adjacent orientation
limit, 1 rad adjacent joint limit and a 90 second requested duration.

Both 0.05 mm and 0.25 mm pose spacings found whole-region IK paths with full
node/edge collision checks (1264 and 370 poses respectively). Both were rejected
by TOPPRA/ECOS with `FailUncontrollable` while timing the fixed joint spline.
No executable trajectory or dynamics verification is claimed for these cases.

Diagnostic artifacts are in `tmp/simple_original_validation_20261110/` and
`tmp/simple_original_coarse_validation_20261110/`, including the generated
pose targets, selected IK joints, saved reference and failure report.
The current three-angle tuple is ignored by simple mode; it retains its
existing meaning in the default continuous mode.
