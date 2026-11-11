# Perception

Perception converts a scene point cloud or colored oral scan into a registered
scene, ordered gingival topology, and optional tooth-instance labels. The four
submodules are `registration`, `gingival_extraction`, `tooth_segmentation`, and
`regions`; each keeps its own intermediate files and can be run or
tested independently.

The main workflow is `perception.workflow.perceive_gingival_line`. Its output
is a `PerceptionResult` containing the scan-to-world transform, registration
directory, topology path, capture metadata, and prepared continuous regions.
`regions` owns tooth assignment, surface planes, curve fitting, endpoint trims
and source continuity. Planning receives `PreparedRegion` and never rebuilds these
curves. Distances in mesh and topology
files are millimetres; transforms used by planning are metres.

The [project guide](../README.md#3-单独运行-perception) contains independently runnable
extraction, simulated acquisition, registration and region-export commands.

![Simulated RGB, depth and dental point cloud](../docs/images/simulation_capture.png)

![Registration overlay from the same simulation](../docs/images/registration.png)

For a fixed external D405, use `python -m perception.real`: `capture` saves
native depth-optical RGB-D artifacts, `recrop` reprocesses saved geometry, and
`register` replays the existing automatic/manual registration without MuJoCo.
Checkpoint evaluation, static repeatability and offline eye-to-hand calibration
are also available. Uncalibrated runs only produce camera coordinates.
See [D405 capture, validation and real-time roadmap](REAL_CAMERA.md) for commands,
file formats, measurement limits and the remaining work for live tracking.

Useful isolated commands include `python -m perception.registration --help` and
`python -m perception.tooth_segmentation --help`. The gingival extractor is
also directly callable as `extract_gingival_line(scan, output_dir, jaw)`.

Export the merged mandibular six-region snapshot independently:

```powershell
.\.venv\Scripts\python.exe -m perception.regions --out outputs/perception/my_regions
.\.venv\Scripts\python.exe -m planning --perception-result outputs/perception/my_regions/outer_front --out outputs/planning/my_front
```

The second command uses the chosen planning angles and still requires all motion
checks to pass. Six-region input is mandatory and must match the jaw and scan hash;
there is no extracted-topology fallback. Only complete regional endpoints are trimmed,
with no per-tooth trimming or synthetic bridges. `--world-from-scan` attaches a
registered transform in metres. See [current cleanup notes](../docs/integration/2026-11-11-six-region-cleanup.md).
