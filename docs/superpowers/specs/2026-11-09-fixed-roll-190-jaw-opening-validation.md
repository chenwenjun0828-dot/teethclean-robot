# Fixed Roll 190 Degrees: Jaw Opening Validation

Date: 2026-11-09

## Scope

The user requested several images of rejected collision/clearance states, then
requested opening the mouth and repeating simulation checks. This experiment
uses the existing `JawSettings.opening_deg` parameter, first at 20 degrees and
then at 30 degrees. These are runtime overrides; the checked-in oral settings
are not changed by the experiment.

The robot remains 150 mm horizontally closer to the original lower-jaw target,
at `[0.6959828478597841, -0.9912957693238451, 0.77]` metres. Roll remains strictly
190 degrees (equivalent to -170 degrees). Tilt is 15 degrees, TCP standoff is
1.5 mm, full-tool clearance is 0.15 mm, and adjacent joint steps are limited to
0.15 rad. No collision or continuity requirement is relaxed.

## Coordinate Handling

Frozen perception comes from
`tmp/pipeline_fixed_roll_10_lower_20261108_trial01/`. Original geometry,
segmentation, extracted gingival topology and registration are reused.
Perception is not rerun from new camera observations.

The lower jaw is rotated using the existing hinge approximation, with center
`[0, 10, -40]` mm in the scan frame; the upper jaw remains fixed. The new scan
registration is computed as:

```text
world_from_scan_new = world_from_lower_new
                      @ inverse(world_from_lower_old)
                      @ world_from_scan_old
```

This preserves the original registration residual and moves all lower-jaw TCP
targets and orientation frames consistently with the actual lower-jaw mesh.
Each run uses a fresh MuJoCo model and a fresh FCL collision checker. Model,
robot and oral configuration file fingerprints are checked against the frozen
source. Render helpers assert that restored jaw transforms match the planning
report to an absolute tolerance of 1e-8.

## Results

Each opened-jaw experiment runs the complete continuous planner with eight
endpoint trim candidates, from 0.05 through 0.40 mm in 0.05 mm increments.
Neither run produces an executable trajectory. Independent trajectory audit
and execution are therefore not reached.

Representative candidate: endpoint trim 0.10 mm (candidate 2).

| Opening | Samples | Samples with no safe IK branch | First graph break | Smallest joint step at that break | Limit |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0 deg | 291 | 79 | 68 | 0.2972 rad | 0.15 rad |
| 20 deg | 290 | 31 | 68 | 0.4799 rad | 0.15 rad |
| 30 deg | 290 | 31 | 68 | 0.5398 rad | 0.15 rad |

These sample counts belong to the stated candidate, not every tested path.
Small changes in reachable source windows cause the 291/290 sample difference.
Every representative target has at least one IK solution before collision
filtering. The graph break is on a synthetic bridge between tooth instances
10 and 9. Its orientation step is approximately 18.64 degrees (below the
20-degree limit); the adjacent joint step exceeds the configured limit.

The 31 remaining unsafe targets occur on bridge parts 3, 5 and 7. Rejected
branches include full-tool/lower-jaw mesh intersection and robot self contacts.
Opening the mouth reduces upper-jaw interference, but does not make the
existing path and tool directions feasible relative to the lower jaw.

Opened-jaw planning took approximately 6.0 s at 20 degrees and 6.2 s at 30
degrees, excluding separate diagnostics and image rendering. Total experiment
times were approximately 7.1 s and 7.4 s. These timings reuse perception and
do not include a fresh perception pipeline or trajectory execution.

## Images and Exact Mesh Evidence

Images are actual MuJoCo renders of the original mesh assets. They show static
targets being diagnosed, not accepted or executed robot motion. No mesh poses
are altered for camera framing, no teeth are hidden, and no transparency edits
are used. Screen-space annotations mark positions returned by FCL, including
occluded contacts. Contact queries cap returned triangle contacts at 64 per jaw.
Their count is not a penetration-depth measurement.

Three original reference points are replayed with the rigid lower-jaw transform
for a like-for-like comparison. They refer to the original candidate's sample
indices, not the newly generated 290-point candidate's indices.

| Original reference index | Jaw at 0 deg | Jaw at 20 deg |
| --- | --- | --- |
| 0, source curve | Upper intersects; lower clearance 0.509 mm | Upper clearance 21.127 mm; lower clearance 0.509 mm |
| 100, bridge part 3 | Both upper and lower intersect | Upper clearance 18.525 mm; lower still intersects |
| 160, bridge part 5 | Upper intersects; lower clearance 0.480 mm | Upper clearance 20.565 mm; lower clearance 0.480 mm |

At these original rejected states, returned contact positions are 1-7 mm from
the TCP on the working region near the curved tip, rather than the large
handpiece farther back. The 1.5 mm target offset applies to the TCP relative to
an approximate reference plane; it does not guarantee clearance of the whole
curved tool. A geometrically smooth connecting curve can still turn the curved
working region into a tooth. Fixed roll is relative to the changing local
tangent/normal frame, not a constant world orientation.

Opening the lower jaw moves both the target tooth and its tool target together,
so their relative geometry is unchanged for an identical scan-space target.
It increases space to the opposing upper jaw. Moving the robot base changes
IK and arm reach but likewise cannot eliminate a tool/tooth intersection at
an otherwise unchanged TCP pose.

Artifacts:

- `tmp/fixed_roll_190_collision_images_20261109/jaw_opening_overview.png`
- `tmp/fixed_roll_190_collision_images_20261109/collision_comparison_0.png`
- `tmp/fixed_roll_190_collision_images_20261109/collision_comparison_100.png`
- `tmp/fixed_roll_190_collision_images_20261109/collision_comparison_160.png`
- `tmp/fixed_roll_190_collision_images_20261109/contacts.json`
- `tmp/fixed_roll_190_collision_images_20261109/comparison_contacts.json`
- `tmp/pipeline_fixed_roll_190_mount150_open20_20261109/experiment.json`
- `tmp/pipeline_fixed_roll_190_mount150_open30_20261109/experiment.json`
- Each run's `candidate_2_graph_diagnosis.json` and `tracking/planning_report.json`.

## Reproduction

```powershell
.\.venv\Scripts\python.exe -m tmp.validate_fixed_roll_pipeline `
  --source tmp/pipeline_fixed_roll_10_lower_20261108_trial01 `
  --out tmp/pipeline_fixed_roll_190_mount150_open20_repeat `
  --mount-toward-mm 150 --opening-deg 20 `
  --trim-mm 0.05 --max-trim-mm 0.4 --trim-step-mm 0.05

.\.venv\Scripts\python.exe -m tmp.inspect_mount_sweep `
  --run tmp/pipeline_fixed_roll_190_mount150_open20_repeat --check-nodes
```

The normal pipeline also accepts `--jaw-opening-deg 20`. Its fixed-roll command
line argument currently accepts the canonical -170 degrees for the same 190
degree orientation. Passing this parameter is not a claim of execution
readiness: planning, collision, coverage, continuity and independent trajectory
audit must still succeed.
