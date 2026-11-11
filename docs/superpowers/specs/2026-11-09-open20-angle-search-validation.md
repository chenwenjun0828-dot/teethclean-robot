# Open-20 Angle Search Validation

## Scope and Outcome

The confirmed rigid TCP frame was retained. Jaw opening was exactly 20 degrees.
The experiments reused frozen perception from
`tmp/pipeline_fixed_roll_10_lower_20261108_trial01` and applied the measured rigid
jaw transform to its registration. They do not represent a fresh perception run.

All 13 full-planner experiments failed planning; none attempted execution. No
executable result has been approved by the completed experiments. An analytic
graph survivor is only a preliminary result. Full planning recomputes source
windows, geometry, exact FK, collisions and approach before independent audits
and servo execution. Failed planning never proceeds to execution.

The existing four-middle-tooth, noncontact workflow was retained: 1.5 mm normal
offset, 0.15 mm clearance, 0.15 rad joint step, 20 degree orientation step,
0.004 rad collision sweep, 0.1 mm / 1 degree interpolation tolerances. The speed
configuration remains 15 mm/s, 2.2 rad/s, 12 rad/s^2 and 200 degrees/s.
Production algorithms, models, TCP, default angles and default mount were not
changed by this search.

## Preliminary Searches

`tmp/search_open20_angles.py` screens an existing reference using the existing
analytic IK branches. It excludes obvious graph failures but does not check
collisions or independently verify FK. It cannot approve execution.

`tmp/search_open20_source_windows.py` additionally uses the existing exact
source-window search and rebuilds bridges for each angle. Source geometry is
shared at the original 0.25 mm spacing for this preliminary stage. Its denser
graph spacing is explicitly recorded. A full run rebuilds source geometry at
the actual requested spacing, which can change tooth assignments, plane support,
source windows and bridges. This is why a preliminary survivor can fail the
subsequent full run.

| Search Folder Under tmp | Attempts | Graph Survivors | Seconds |
| --- | ---: | ---: | ---: |
| open20_angle_search_strict_20261109 | 44 | 0 | 0.3 |
| open20_angle_search_broad_20261109 | 308 | 0 | 2.4 |
| open20_angle_search_dense_20261109 | 308 | 0 | 6.9 |
| open20_angle_search_fine_20261109 | 96 | 0 | 9.9 |
| open20_angle_search_ultrafine_20261109 | 76 | 4 | 53.4 |
| open20_source_search_20261109 | 250 | 0 | 73.7 |
| open20_source_search_high_tilt_20261109 | 176 | 0 | 72.1 |
| open20_source_search_near190_20261109 | 216 | 1 | 259.7 |
| open20_source_search_mount150_20261109 | 58 | 3 | 150.4 |

These are 1,532 preliminary evaluations, including repetitions at different
spacings. There are 868 distinct mount/angle combinations. Exact angle-dependent
source windows were searched for 684 combinations: 626 at the default mount
and 58 at the previously approved mount 150 mm closer. Those searches evaluated
1,505 trimmed reference candidates. The counts are finite grid-search results,
not a proof that no feasible angle exists anywhere in continuous parameter space.

Azimuths stay within 30 degrees of either in-plane normal (+/-90 degrees).
The tilt grids span 15 to 85 degrees; fixed roll grids span 130 to 220 degrees.
Not every combination in this bounding box was evaluated: working-end-angle
filters and early exits are recorded in each screen.json.

The estimated terminal working direction is
`[0.4876797803, 0.8729865484, 0.0079320947]` in TCP coordinates, measured from
the actual mesh. It differs from shaft X by 60.8118 degrees. Its angle to the
three-point reference plane is only a geometric proxy for inspecting poses;
it is not the angle to every point of the curved tooth surface or a clinical
validation. Initial shallow-angle filtering was followed by a wider diagnostic
search, up to 32 degrees, to distinguish physical obstruction from graph failure.

## Full Planner Experiments

Mount D is the original `[0.815, -0.9, 0.77]` m. Mount C is the approved closer
position `[0.6959828479, -0.9912957693, 0.77]` m. Roll 190 means CLI roll -170.

| Tilt / Azimuth / Roll (deg) | Mount | Spacing (mm) | Result |
| --- | --- | ---: | --- |
| 45 / +90 / 150 | D | 0.1 | No reachable principal source on tooth instance 10 |
| 45 / -90 / 150 | D | 0.1 | No reachable principal source on tooth instance 9 |
| 55 / 75 / 185 | D | 0.005 | Tooth instance 10 below minimum retained coverage |
| 55 / 75 / 190 | D | 0.005 | Analytic graph connected; collision rejection disconnects it |
| 55 / 60 / 195 | D | 0.025 | Disconnected graph at layer 630 with trim 2.5 mm |
| 55 / 75 / 190 | C | 0.01 | Disconnected graph at layer 1340 with trim 1 mm |
| 45 / 60 / 185 | C | 0.01 | Disconnected graph at layer 1391 with trim 2 mm |
| 55 / 75 / 195 | C | 0.005 | Analytic graph connected; self-collision rejection disconnects it at layer 2785 |

The 55/75/190 default-mount run took 133.4 seconds for full planning. Geometry
generation took 0.93 seconds, setup 1.28 seconds and source/pose/motion planning
131.19 seconds. The 0.005 mm sampling produces 10,018 reference targets. After
finding an analytic connection, lazy collision checking rejects 1,591 graph
nodes and loses connectivity at layer 1543. Recorded rejected nodes include
`ur_wrist_1_link / ur_end_scaler_link`. Larger symmetric trims fail minimum
coverage. The independent trajectory audit is never reached in this run.

The final closer-mount 55/75/195 run took 129.4 seconds for full planning.
At trim 2 mm its analytic graph connects, but collision checking rejects 1,560
nodes and disconnects the graph at layer 2785. Recorded rejected nodes again
include wrist/tool self-collision. At trim 2.5 mm tooth instance 2 falls below
the required retained coverage. Neither candidate reaches independent audits.

## Bounded Variable Roll

The existing RollPolicy was also tested with a center of 190 degrees and a
maximum deviation of 30 degrees. The implementation reserves the existing
1 degree audit margin inside that corridor. This is explicitly variable roll,
not a successful fixed-roll result.

| Tilt / Azimuth | Mount | Spacing / Roll Grid | Result |
| --- | --- | --- | --- |
| 35 / 60 | D | 0.1 mm / 10 | No continuous graph after five trim/bridge attempts |
| 45 / 75 | D | 0.1 mm / 10 | No continuous graph after five trim/bridge attempts |
| 55 / 75 | D | 0.1 mm / 10 | No continuous graph after five trim/bridge attempts |
| 35 / 60 | C | 0.025 mm / 5 | No continuous graph after five trim/bridge attempts |
| 55 / 75 | D | 0.025 mm / 5 | No continuous graph after five trim/bridge attempts |

The last two runs took 142.1 and 167.8 seconds. Individual parameters, local
bridge edits, source intervals, rejection reasons and timing are retained in
the corresponding `tmp/open20_corridor30*/experiment.json` and tracking reports.

## Visual Evidence

`tmp/open20_angle_search_evidence_20261109/angle_search_evidence.png` shows three
actual exact-IK configurations from the 55/75/190 default-mount reference.
They are static configurations from a failed planning run, not executed frames.

| Layer | Reference Part | Valid Local IK | Selected Configuration |
| ---: | ---: | ---: | --- |
| 594 | 0 | 8/8 | Valid; tool/lower-jaw mesh clearance 0.417 mm |
| 1543 | 1 | 5/8 | Valid local alternative; tool/lower-jaw clearance 0.819 mm |
| 5100 | 5 | 5/8 | Representative rejected wrist/tool self-collision branch |

The third point also has valid local alternatives. This does not imply they
connect continuously to the valid branches at earlier and later points under
the same joint-step and collision requirements. The depicted rejected branch
must not be described as the only solution at that point.

Mesh distances to the upper jaw at these three poses are 26.17, 29.58 and
27.58 mm. Opening the jaw solves the upper-jaw obstruction at these particular
poses; it does not resolve the full-path branch and self-collision constraints.
No negative FCL collision sentinel is reported as a penetration depth.

## Reproduction and Limits

For example, the full fixed-190 rejection can be reproduced with a new output
directory:

```powershell
.\.venv\Scripts\python.exe -m tmp.validate_fixed_roll_pipeline `
  --source tmp/pipeline_fixed_roll_10_lower_20261108_trial01 `
  --out tmp/open20_reproduce_new `
  --roll-deg 190 --tilt-deg 55 --tilt-azimuth-deg 75 `
  --opening-deg 20 --spacing-mm .005
```

All added experiment helpers passed Python compilation and were exercised on
the actual model. No production defaults or collision thresholds were loosened.
No fresh perception-to-servo pipeline success is claimed. Broader changes to
path construction or varying tilt/azimuth along the path have not been made;
those would exceed the fixed-parameter search and existing bounded-roll checks.
