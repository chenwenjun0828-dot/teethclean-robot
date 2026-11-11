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
