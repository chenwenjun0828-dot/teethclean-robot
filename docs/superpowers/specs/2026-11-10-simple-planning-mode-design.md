# Simple Planning Mode

## Behavior

Expose `--planning-mode continuous|simple` in independent planning and the full
pipeline, including pipeline JSON parameters. The default remains `continuous`.

The user explicitly selected the supplied old file's logic, including its
actual angle formulas. `simple` computes gradient tangents, three-point normals
flipped toward +Y, nearest plane-center KDTree assignment, and updates the
orientation only when the matched plane index changes. It rotates the normal
about the tangent by -75 degrees, constructs intermediate axes, rotates the
intermediate X about intermediate Y by -33 degrees, and offsets the point along
the matched normal. No curve/normal smoothing is added to that computation.
The current three-angle tuple does not affect this mode.

Expose `--simple-angles-deg A B` for the old rotations. The pure module reads
the old TXT formats and accepts direct arrays. The pipeline supplies current
reconstructed regional points and three-point tooth planes. All old algorithm
waypoints are retained. Position and shortest-quaternion interpolation between
waypoints forms the saved reference; pose-stage subdivision accommodates
translation/rotation limits before IK. The audit checks these prescribed poses,
not the old comment claiming a constant 15-degree normal angle.

## Integration

Keep the implementation in `planning/simple.py`. Both modes share geometric
reconstruction, continuous IK selection, full collision checks, approach/retreat,
fixed joint splines, TOPPRA, and dynamics validation. The path-kind identifiers
continue to describe continuous regional geometry. Planning mode is a separate
configuration field and is saved in request, geometry, reference and reports.
Old artifacts without the field load as `continuous`.

Saved references contain the simple mode's input points and planes, not trusted
precomputed tool rotations. Loading rejects mode changes, missing waypoints and
inconsistent metadata. Playback and control restore the saved mode without
changing a validated trajectory. Changing planning mode requires replanning.

## Verification

Test the original numerical rotations, nearest-plane switches and holds,
TXT parsing, rigid-frame transforms, saved-reference reload and mode
tampering, actual FK orientation audits, and propagation through CLI/JSON and
pipeline execution (including temporary outputs). Run existing planning and
pipeline regression tests, then try a real headless regional plan with the
current parameter set. The old TCP's physical axis definition is absent, so
old angles are not assumed feasible for the current tool frame. Record any
geometric infeasibility without weakening validation thresholds.
