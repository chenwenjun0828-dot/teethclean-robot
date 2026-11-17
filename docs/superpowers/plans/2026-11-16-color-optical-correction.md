# Color Optical Hand-eye Correction Implementation Plan

> Execution: inline in this session, as authorized by the user. No delegation or Git publication is required.

**Goal:** Use the user-confirmed color-optical interpretation consistently in conversion code, the existing compact planning session, provenance and handoff documentation.

**Architecture:** Keep original calibration YAML/SDK reports and scan registration immutable. Introduce one frame-selection function shared by conversion and a replayable session correction command. Recompute derived Base geometry from saved scan-local geometry, validate in a staging directory, and back up the old session locally before installing the correction.

**Tech Stack:** Python, NumPy, SciPy, existing PreparedRegion and unittest interfaces.

## Tasks

- [x] Add `perception/coordinate_conversion/handeye_frames.py`: `select_handeye_chain(handeye, sdk, mode='color_optical')` returns the numeric handeye, depth-to-selected-camera transform, and explicit declared/effective frame metadata. `declared` retains the historical standard-ROS chain. Invalid modes raise `ValueError`.
- [x] Test that identity handeye plus a forward optical point remains forward by default, whereas `declared` maps optical Z to ROS X; assert the two rotations differ by 120 degrees. Test invalid mode rejection.
- [x] Update `convert_points.py` with `--handeye-frame {color_optical,declared}`, default color_optical. Export matrices under accurate effective frame names and record mode and formula in conversion reports.
- [x] Add `correct_session.py` with `--session` and `--backup-dir`. Validate manifest hashes and PreparedRegions, verify original/current matrix chain, then stage the correction. Derive both jaw matrices from unchanged T_depth_optical_scan, preserve existing rigid-normalization policy, update world positions and directions through `export_regions`, and refresh all hashes. Retain region counts, source geometry, shared meshes, calibration numeric values and robot joints.
- [x] Test the tracked asset chain against default optical composition; verify CSV/NPZ/PreparedRegion equality, normal and tangent rotation, original calibration hashes, and rejection of corrupted input. Run the correction twice to verify no double application.
- [x] Propagate frame interpretation in `publish_session.py` and region metadata. Preserve original declarations as declarations, without relabeling raw YAML or SDK evidence.
- [x] Back up the existing session under ignored `outputs/perception/frame_correction/`, install the validated optical result at the existing stable path, and update calibration interpretation/provenance without changing raw calibration inputs.
- [x] Update main entry points, session/calibration README, coordinate-conversion README, handoff and workflow documents. Explain the removed ROS axis conversion, obsolete historical outputs, and data revision. Remove any claim that the original ROS frame definition was physically verified.
- [x] Validate with `D:\python\python.exe -m unittest tests.test_coordinate_conversion tests.test_perception_session_assets tests.test_camera_extrinsics tests.test_region_portability`, plus correction tests. Check links and source hashes. Do not connect to or move hardware. Do not commit or push without a new request.

## Acceptance

Default chain is `T_B_E @ T_E_color_optical @ T_color_optical_depth_optical @ T_depth_optical_scan`. Both jaws and all 12 region exports agree with that chain. Re-running correction does not move the data again. Raw calibration, scan-local geometry and registration are unchanged. Existing users can load the same stable paths, with explicit corrected-frame revision metadata.

Validation: 18 focused tests passed; links, raw calibration bytes and source asset stability verified. No commit or push performed.
