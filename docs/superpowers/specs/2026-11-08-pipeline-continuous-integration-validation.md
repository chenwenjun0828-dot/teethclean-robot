# Continuous pipeline integration validation

Date: 2026-11-08 (Asia/Shanghai).

## Interface and execution checks

The pipeline defaults to continuous mode and a 1.5 mm non-contact offset.
`--planning-mode segmented --offset-mm 0.8` selects the previous segmented setup.
Continuous failure stops before control; there is no segmented fallback. Before
execution, the pipeline checks the saved reference fingerprint and requires one
complete trace visiting the four configured middle teeth, without internal
retreats, transfers, or re-entries.

Existing geometry, IK, pose, timing, collision and execution guards remain in use.
The simulation retains the 15 mm/s nominal trace speed, 2.2 rad/s joint speed,
12 rad/s^2 joint acceleration, 200 deg/s orientation bound, and 150 mm/s free-space
transfer setting. These limits do not certify hardware operation.

## Successful runs

The lower-jaw full run was saved under
`tmp/pipeline_continuous_integration_lower_20261108/`. Its tracking directory
contains the planning/reference/audit and execution records. The same validated
trajectory also completed a GUI replay in `tracking/gui_replay/`, with identical
dynamics metrics to headless execution.

| Wall-clock stage | Lower jaw (s) | Upper jaw (s) |
| --- | ---: | ---: |
| Scene setup | 0.84 | 0.98 |
| Gingival extraction | 3.65 | 3.91 |
| Capture | 0.81 | 0.92 |
| Automatic registration | 1.53 | 1.95 |
| Geometry generation | 0.79 | 0.81 |
| Planning setup | 1.23 | 1.29 |
| Pose search and motion planning | 40.90 | 22.50 |
| Independent trajectory validation | 94.97 | 43.46 |
| Headless dynamics execution | 22.88 | 10.06 |
| Total | 167.68 | 85.97 |
| Trajectory simulation duration | 107.48 | 60.13 |

These are observed runs, not a controlled benchmark. Tests ran alongside part of
lower-jaw planning; the upper run overlapped the lower GUI replay. Headless wall
time is not the duration of real-time GUI playback.

Lower-jaw source coverage is 63.1154%: 34.0934 mm retained original source plus
8.7094 mm synthetic connectors, for a 42.8027 mm reconstructed sweep. The four
middle teeth have motion. The contract reports one trace block, zero internal
retreats, and zero internal transfers. The initial approach occupies roughly
4.28 seconds before the trace begins.

The successful lower run's maximum actual position error was 0.5869 mm, maximum
rotation error 0.1345 degrees, and minimum actual clearance 0.1711 mm. Independent
reference audit error was approximately 0.020 mm and 0.210 degrees. Upper-jaw
execution completed with `--no-output`; its temporary run directory was removed.

## Boundaries and failed runs

At 0.8 mm offset, the lower-jaw continuous candidate failed collision/path
connectivity checks; larger endpoint trims then failed minimum source coverage.
This setting must not silently select segmented motion.

One earlier 1.5 mm GUI pipeline candidate passed static audits but stopped on the
existing 2 mm dynamics guard (observed error 2.033 mm). Replanning produced the
successful lower trajectory above. The identical saved successful trajectory
passed both GUI and headless execution, so this failure is not evidence that GUI
playback changes the dynamics. Registration/search outcomes can differ between
runs; a successful candidate does not certify every future trajectory.

Both successful plans required the existing validator's final zero waypoint
velocity option after higher velocity scales exceeded the 1-degree reference
error bound. Geometric continuity and absence of internal transfers do not imply
nonzero speed at every interpolated waypoint. Timing/pose algorithms were not
changed by this integration. Independent validation is the main measured delay
before the viewer opens. Force control and full-jaw coverage remain outside this
non-contact middle-region simulation.

## Commands

```powershell
.\.venv\Scripts\python.exe -m pipeline.run --jaw mandibular --registration-method automatic --headless --output tmp/pipeline_continuous_integration_lower_20261108
.\.venv\Scripts\python.exe -m pipeline.run --jaw maxillary --registration-method automatic --headless --no-output
.\.venv\Scripts\python.exe -m unittest discover -s tests -q
.\.venv\Scripts\python.exe -m unittest discover -s planning -t . -q
```

Use a new empty output directory to repeat the persistent run.
