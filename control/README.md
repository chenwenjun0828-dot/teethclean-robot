# Control

Control consumes a planning output only after its trajectory, collision and dynamics checks have passed.

## Position execution

`control.position` runs the verified MuJoCo position servo. The main pipeline calls it automatically after
planning; it does not reconstruct or alter the perception path.

## Force validation

`control.force` is an independent experiment for sensor-feedback force control. It defaults to the current
regional trajectory, records the compensated force and raw six-axis signals, and applies predictive force
and collision guards:

```powershell
.\.venv\Scripts\python.exe -m control.force `
  --source outputs/pipeline/my_run/tracking `
  --target-force-n 1 --out outputs/contact_1111/my_force_run
```

`--source` is required and must name a verified planning output. Explicit `--mode trajectory` remains
supported. The retired segmented-plan probe CLI (`--mode probe`, `--sweep-mm`) has been removed.

Force tracking success and true scanned-surface contact are reported separately. The current MuJoCo dental
model uses convex collision geometry, so a force signal can appear before the tool reaches the original scan
surface. See `docs/integration/2026-11-11-contact-force-validation.md`.

The force implementation is contained in `control/force/`; the old root-level forwarding modules have been
removed. Hardware backends remain under `control/hardware/`.

ROS 1 UR7e saved-trajectory export, read-only checks and protected action execution
also live under `control/hardware/`. See [hardware setup](hardware/README.md).
Simulation readiness does not authorize hardware motion.

The standalone two-file workstation delivery is in
`control/employment/2026_11_16/`. Its developer-approved geometry replay policy
is separate from the original project CLI, which retains full execution gates.
The current JSON contains only joint names and timed joint points. The paired
script uses the workstation's existing ROS 1/MoveIt environment, with defaults
`manipulator`, `base_link` and `tool0`. No TCP reference or deployment dictionary
is embedded. This minimal delivery omits the protected backend's TCP/force/watchdog
monitoring; see the hardware README for startup, speed limits and start/stop commands.
