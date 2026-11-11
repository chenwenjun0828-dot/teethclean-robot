# D405 registration software validation

Date: 2026-11-09 (Asia/Shanghai).

## Implemented

- Native depth-optical D405 frame source with measured scale, SDK geometry and
  color mapping, device selection, timeout handling and pipeline cleanup.
- Raw, ROI and filtered artifacts with fingerprints, preview and batch capture.
- Offline recropping with original acquisition identity preserved.
- Automatic and manual replay using the existing registration algorithms.
- Camera-only results without calibration; optional checked eye-to-hand
  calibration, base-metre transforms and gingival line export.
- Withheld correspondences, explicit quality limits, result integrity checks,
  and static acquisition/algorithm repeatability reporting.
- Chinese operating guide and real-time architecture/limitations discussion.

## Checks completed

The final independent perception run passed 42 tests: 22 new tests in
`tests/test_real_registration.py`, plus existing registration, RGB-D, frame and
paired-landmark tests. Coverage includes SDK doubles, scale, native grids,
extrinsic matrix layout, camera lifecycle, capture/replay/recrop, hashes,
manual and automatic registration, base composition, withheld-point provenance,
quality failure exit codes, calibration fitting and held-out poses.

The new modules compile, CLI help works, and tracked-file diff whitespace checks
pass. RealSense SDK 2.59.1.11053 was installed in the project virtual environment.
`python -m perception.real devices` returned `No RealSense devices connected`.

An end-to-end synthetic software check was retained under
`outputs/real_registration_software_check_20261109_001/`, including source data,
function and CLI registration runs, repeatability output and the registration
overlay. The overlay was visually inspected. These artifacts are explicitly
labelled synthetic; they are not physical D405 captures or accuracy evidence.

## Integration boundary

An earlier combined run passed 50 tests including the automatic simulation
pipeline. After the final additional tests were added, rerunning the combined
suite encountered a concurrent planning refactor: importing
`planning.pose_search.policy` failed to import `RollPolicy` from
`planning.config`. The final independent perception tests passed. Planning
changes were left intact; the complete simulation integration is not claimed
verified against the final concurrently changing planning state.

## Not measured

No attached D405 was available. Actual camera streaming, exposure behavior,
interactive ROI/checkpoint windows, physical tooth localization accuracy,
camera-to-base calibration accuracy and real-time latency require hardware
validation. Robot transport and board detection/PnP are not implemented by this
work; offline calibration accepts measured robot/board pose records.
