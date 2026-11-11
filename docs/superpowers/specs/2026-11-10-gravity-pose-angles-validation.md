# Gravity Pose Angle Validation

Historical validation for the v2 zero reference. The later user-requested
Y-down zero reference supersedes this convention and requires replanning;
the real-data run below is not validation of v3.

The new pose convention uses fixed projected-gravity Z, fixed inward-normal
cross Z for Y, and a final rotation about tool +X. The zero pose aligns tool
+X with the inward normal and tool +Z with projected gravity. Positive angles
follow the right-hand rule. The second rotation uses the original fixed Y,
as confirmed by the user.

## Automated Checks

- Planning discovery: 130 tests passed using
  `.venv/Scripts/python.exe -m unittest discover -s planning -t . -p 'test_*.py'`.
- Pipeline, geometry-only, planning v2, reachability and parameter suites:
  87 tests passed.
- After adding the gravity metadata rejection case and updating CLI help,
  43 focused tests passed. This includes the new case, bringing unique test
  coverage to 218 tests.
- Python and browser rotations agree to 1e-12 across 378 angle tuples,
  including zero, signed angles, +/-90-degree singularities and +/-180 wraps.
- Checks cover fixed-axis order/sign, flip preserving +X, normal/gravity
  coordinate transformations, regional inverse-angle branches, the hard
  +/-5-degree FK constraint, and changed-gravity metadata rejection.
- Pipeline dry-run with `--planning-mode simple --angle-deg 0
  --azimuth-deg -20 --flip-deg -120` resolves to `[0, -20, -120]`.

## Real Registered Geometry

Reused the frozen request from
`tmp/regional_simple_geometry_verified_20261110_160619/request.json` with
the actual registration, jaw opening 20 degrees, middle region, 0.5 mm offset,
2 mm endpoint trims, and a 90-second timing target. Pose parameters were
`[15, 0, -120]` under the new convention.

Output: `tmp/gravity_pose_geometry_20261110_163636`.

The nominal regional orientation succeeded without corrections: 1215 path
poses, one continuous cleaning block, all four selected teeth, no internal
retreat or transfer. Total duration is 90 seconds. Timed FK auditing used
10926 samples; maximum position error was 0.0000496 mm, full rotation error
was below 1e-8 degrees, and all recovered regional angle deviations were
below 1e-8 degrees. The actual normal angle varies along the curved region,
as expected when world orientation is fixed.

This run used `check_mode=geometry_only` and diagnostic orientation policy;
its measured orientation checks and mandatory regional bounds passed.
It did not evaluate clearance or dynamics and has `execution_ready=false`.

## Browser Verification

Playwright checked Chrome at 1440x1000 and 390x844, including angle changes,
flip preserving the shaft, 90-degree singular poses, surface contact picking,
reset, and full upper/lower dentition loading. No page or asset errors and
no horizontal/control overflow occurred. Screenshots were visually inspected.

Canvas pixel checks found 31157 foreground pixels on desktop and 18568 on
mobile. Changing angles changed 75161 and 28778 pixels respectively, confirming
nonblank, responsive renders.

Artifacts are in `tmp/gravity_pose_viewer_desktop.png`,
`tmp/gravity_pose_viewer_mobile.png`, and
`tmp/gravity_pose_browser_validation.json`. The viewer runs at
`http://127.0.0.1:8767/` and uses default-scene gravity in scan coordinates;
planning uses the actual registration transform.

## Y-Down Follow-Up (v3)

The user requested that tool +Y point down by default. The zero reference is
now `[n, z, n cross z]`, with z the downward gravity projection. Fixed rotation
axes and shaft directions are preserved; the zero-flip reference changes by
+90 degrees about tool +X relative to v2. Forward pose construction, regional
angle recovery, independent auditing, and the browser all use the new frame.
The v3 axis/policy IDs reject the earlier saved pose convention.

132 planning tests and 87 pipeline/integration tests passed (219 total),
including the new zero-flip/downward-Y tests and retired-v2-reference check.
Playwright verified default and selected-contact Y alignment, tool flip,
angle changes, singular poses, dentition assets, and desktop/mobile layout.
Maximum zero-pose Y alignment component error was 2.23e-16. Canvas checks found
37146/13200 foreground pixels and 113643/25755 changed pixels after angle
adjustment on desktop/mobile. Screenshots were visually inspected.

Current browser artifacts use the `tmp/gravity_pose_y_down_` prefix. The
viewer was restarted at the same localhost port, 8767. The earlier real-data
v2 plan above was not regenerated in this follow-up.
