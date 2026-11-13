# Regional fixed-pose contact validation, 2026-11-10

The updated 2026-11-11 continuous-region controller, independent contact audit,
and current results are documented in [the current validation](../docs/integration/2026-11-11-contact-force-validation.md).
The historical per-arc reproduction below now explicitly selects `segments`.

The controller completed the retained trajectory in the existing convex-hull
physics model. Contact cleaning against the original scanned tooth surface was
NOT verified: all independently audited force-present states had a positive
tool-to-scan gap of 0.561-1.596 mm.

## Source and Execution

- Source: `outputs/integration_1111/pipeline_registered/tracking`.
- Configuration: regional fixed pose `[6.37, 13.26, -0.4]` degrees, 20-degree jaw
  opening, 1.8 mm nominal standoff. No force-run roll corrections were supplied.
- All five retained source arcs were traversed, totaling 23.3501 mm. These
  represent 43.23% of the original 54.0175 mm middle-region source line.
- Synthetic bridges were traversed after release. They are not counted as
  contact-cleaning length. Contact was intentionally released between arcs.
- The original source joint interpolation, approach, bridges and final retreat
  were retained. There was one initial home-state setup, without per-arc resets.
- Source trajectory and planning-report fingerprints remained unchanged.
- Target force: 1 N; nominal segment speed: 0.5 mm/s; simulation budget: 240 s.
  This is a simulation test setting, not a clinical force prescription.
- Existing 2 N unloading, 2.5 N predictive rejection, 3 N hard stop and
  non-working/opposite-jaw contact protections remained enabled.

## Convex-Hull Physics Result

Output: `outputs/contact_1111/regional_contact_new_run`.

| Metric | Result |
| --- | --- |
| Execution including final retreat | Completed |
| Simulation duration | 138.912 s |
| Retained source progress | 100% |
| Along-line force range | 0.879627-1.096096 N |
| Along-line mean force | 0.998419 N |
| Along-line force RMSE | 0.035976 N |
| Along-line samples within 1 +/- 0.2 N | 100% |
| Along-line contact loss | 0 s |
| Whole-run compensated force peak | 1.946566 N |
| Unexpected contact samples | 0 |
| Maximum tangential tracking error | 0.225339 mm |
| Maximum orientation tracking error | 0.031263 degrees |
| Controller criterion in this physics model | Passed |
| One-minute completion | Failed |

The earlier trial in `outputs/contact/20261110_surface15_force_1N` completed all
source arcs but stopped during final retreat. Its normal correction faded before
the nominal retreat had withdrawn the tool. Removal now follows the same quintic
progress as the source retreat. The rejected prediction was not executed.

## Independent Scanned-Surface Result

Actual measured joint positions were replayed through independent FK and FCL
distance queries between the tool triangle mesh and the original scanned jaw
triangle mesh, using the verified source transforms and jaw opening.

| Metric | Result |
| --- | --- |
| Replay rate | 20 Hz, plus boundaries and force extrema |
| Total replay states | 2,801 |
| Force-present replay states | 1,804 |
| Force-present states with scan gap above 0.1 mm | 1,804 / 1,804 |
| Tool-to-scan gap in force-present states | 0.561225-1.596348 mm |
| Triangle-mesh intersection states | 0 |
| Replayed vs logged TCP position error | 0 mm |
| Contact cleaning against the scanned surface | Not verified |

The force solver contacts convex proxies while the actual meshes remain apart.
This discrepancy is much larger than the configured 0.05 mm per-geometry
activation margin. Reducing that margin cannot restore the missing concave tooth
geometry. MuJoCo documents this mesh behavior in its
[mesh reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh).

The mesh audit is a sampled geometric check, not a continuous contact certificate.
It nevertheless disproves interpreting this force run as contact with the scanned
tooth surface. A surface-faithful contact model and independent contact-location
validation are required before reporting successful contact cleaning. The prior
15-degree working-tip audit applies to the non-contact source trajectory and is
not a certification of clinical contact in this experiment.

## Reproduction and Files

```powershell
.\.venv\Scripts\python.exe -m control.force --mode trajectory `
  --contact-schedule segments `
  --source outputs/integration_1111/pipeline_registered/tracking `
  --out outputs/contact_1111/regional_contact_new_run `
  --target-force-n 1 --trace-speed-mm-s 0.5 --max-duration-s 240
```

The output contains `force_trajectory_report.json`, `trajectory_force.csv`,
`trajectory_contacts.csv`, `force_tracking.png`, `scan_contact_audit.json`,
`scan_contact_audit.csv` and `scan_contact_audit.png`. The latter plot shows the
first-to-last force-present time window; the CSV retains approach and retreat.

The current implementation is in `control/force/contact_experiment.py`,
`control/force/contact_path.py` and `control/force/force_trajectory.py`. The
force runner restores source jaw opening,
adapts continuous source metadata without modifying joint motion, and includes
the original final retreat. Focused verification passed all 27 tests:

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_contact_path `
  tests.test_contact_sweep tests.test_compliance tests.test_regional_force_validation
```
