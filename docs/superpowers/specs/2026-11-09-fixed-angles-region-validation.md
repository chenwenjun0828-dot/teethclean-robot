# Fixed Angles And Region Validation

## Automated Checks

Using the repository `.venv` on 2026-11-09:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s planning -t . -p 'test_*.py' -q
# 100 tests passed
.\.venv\Scripts\python.exe -m unittest discover -s tests -t . -q
# 147 tests passed
git diff --check
# passed
```

Regional tests cover CLI/API propagation, fixed-angle search, selected tooth
ordering and coverage, both side-region FK audits, collision rejection,
saved-plan metadata checks, execution gating, and continuous preview dispatch.
The synthetic feasible paths verify the regional solver contract; they are
not evidence of real-model feasibility for any chosen angle tuple.

## Real Scan Checks

Geometry-only runs used the lower jaw and example local angles `(55, -20,
-141)` degrees. Artifacts are under `tmp/region_fixed_angles_20261109_v1/`.

| Region | Selected instance IDs | Strokes | Plane validation |
| --- | --- | --- | --- |
| left | 11, 12, 13, 14, 15 | 14 | all valid |
| middle | 10, 9, 1, 2 | 15 | all valid |
| right | 3, 4, 5, 6, 7 | 14 | IDs 5 and 6 exceed the 0.8 mm RMS limit |

The saved geometry contains exactly the selected teeth. Middle strokes follow
positive scan X; side strokes follow negative scan Z. A left geometry preview
was rendered and inspected with all five selected teeth visible.

Full fixed-angle planning was attempted for both side regions:

- `left_fixed_plan`: `no_continuous_left_sweep`, with
  `no_reachable_principal_source` at stroke 2 / tooth 11.
- `right_fixed_plan`: `no_continuous_right_sweep`, with
  `invalid_tooth_plane` at tooth 5.

Neither run produced a complete executable trajectory. The solver preserved
the requested angles and the existing residual/collision thresholds, retained
the failure diagnostics, and stopped before execution. These example angles
must not be described as a validated side-region motion.
