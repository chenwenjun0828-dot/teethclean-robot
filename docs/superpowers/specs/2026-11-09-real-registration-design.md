# D405 capture and registration validation

Approved scope: fixed external D405, capture first, register saved data, report
validation evidence, and document the path to live registration. Robot execution
is not part of this workflow.

## Contracts

- A context-managed camera source returns an in-memory RGBDFrame. Capture and
  disk persistence are separate so a future consumer can use frames directly.
- Geometry stays in the native depth optical frame (CV axes); SDK point-cloud
  generation handles distortion. Color is sampled with SDK texture coordinates.
- Raw Z16, measured depth scale, depth metres, native RGB, point maps, masks,
  raw/ROI/filtered clouds, timestamps, device identity and hashes are preserved.
- Registration returns oral-scan millimetres to depth-camera millimetres. It
  supports automatic and manual initialization and optional gingival lines.
- Real captures require neither simulation segmentation nor MuJoCo poses.
  Uncalibrated results have no world/base outputs or invented identity extrinsic.
- Optional fixed-camera calibration uses camera serial and optical-frame checks;
  exported base transforms use metres. Calibration fitting error is distinguished
  from independent validation.
- Checkpoints are bound to capture ID and scan hash, withheld from optimization,
  and checked against reused manual fitting points. Human point identification
  and independence remain user-declared measurement properties.
- Quality limits default to unset. Reports distinguish numerical fitting,
  checkpoints, configured limits, repeatability and physical accuracy. All results
  retain robot_execution_allowed=false.

## Implementation and validation

1. Add optional RealSense SDK source, snapshot persistence and capture CLI.
2. Extend existing registration workflow for real captures without changing
   simulation contracts; add replay CLI, checkpoints and optional base outputs.
3. Add offline eye-to-hand calibration and static-scene repeatability reports.
4. Test source lifecycle and units with SDK doubles; test integrity, transforms,
   provenance, withheld points, calibration and replay with synthetic geometry.
5. Run existing perception tests, smoke-test CLI and inspect an exported overlay.
6. Document exact commands, data formats, experimental protocol, current limits
   and future initialization/tracking/relocalization architecture.

Hardware acquisition and accuracy claims require an attached D405 and a matching
physical scan. Synthetic tests verify software behavior, not device precision.

Implementation steps 1-6 are complete. Final software checks and the unavailable
hardware/integration checks are recorded in
`2026-11-09-real-registration-validation.md`.
