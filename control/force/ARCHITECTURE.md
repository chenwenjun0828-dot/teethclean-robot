# Architecture

The current simulation development path regulates signed compensated normal
force, retaining resultant force for all overload guards. Offline contact assets
and explicit uncalibrated solver/material settings are loaded separately from
the immutable position-plan source. Hold, prefix and regional experiments have
separate acceptance: a successful prefix never certifies the full region.
See README.md for geometry gates, legacy resultant mode and current commands.

`local_assets` uses `local_surface` to build connected, inward extruded patches
of original labelled teeth and the actual tool tip. `contact_model` recomputes
surface, ray and coverage gates before injection. `scan_guard` checks both the
executed and privately predicted states against complete original meshes and
rejects side/bottom contacts. `stages` and `stage_gate` enforce hold -> short ->
regional with matching provenance. Local acceptance never certifies an entire
tooth body, soft tissue or a hardware experiment.

The force loop reads six-axis sensor samples, applies the configured target and
hard-stop limits, and lets `contact_path` schedule only validated trajectory
segments. A report records contact state, force peaks, stop reasons, and the
retained path fraction. The current MuJoCo sensor source is replaceable with a
`HardwareBackend` without changing the force policy.
