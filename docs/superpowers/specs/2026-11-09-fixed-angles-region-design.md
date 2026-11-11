# Fixed Tool Angles And Selected Region

The user requested three prescribed local tool angles and a feasible motion
search within one selected left, middle, or right region. The confirmed selector
is the existing left/middle/right partition, not FDI numbers or curve intervals.

## Interface And Scope

- Add `--region left|middle|right` to planning and the complete pipeline.
- Add `--pose-angles-deg TILT AZIMUTH ROLL` as an explicit fixed-angle request.
  These are the current surface-frame angles, not world Euler angles.
- Reuse `Config` tilt/azimuth and `SweepOptions.fixed_roll_deg`; preserve the
  existing individual angle flags and roll-search mode when the tuple is absent.
- Reject conflicting tuple/individual angles and non-middle segmented requests.

Reusing the continuous solver keeps the current collision and interpolation
checks. A separate regional solver would duplicate them; using world Euler
angles would change the established tooth-relative orientation convention.

## Geometry And Search

Build planes and strokes for the selected region. Follow its configured tooth
order; traverse middle strokes left to right and side strokes anterior to
posterior. Source support, whole-path IK branches, edge refinement, and timed
auditing must all use the same three angles. A failed fixed-angle search must
never change the angles or execute a partial disconnected solution.

## Export And Validation

Use `continuous_<region>_sweep` for each region, retaining the existing middle
identifier. Save selected region/teeth, fixed-angle mode, original retained
length, synthetic connector length, and per-tooth coverage. Validate exactly
the selected teeth rather than a hard-coded count of four. Reject mismatched
region/configuration/reference/angle metadata before saved-plan execution.
Keep old middle reports readable with an implicit middle region.

## Verification

Cover angle validation and CLI/API propagation, side direction and tooth
selection, one/five-tooth contracts, fixed-angle IK without roll fallback,
collision/disconnected-path rejection, and independent FK auditing of both
side regions. Run the existing planning and pipeline tests. Use actual scan
geometry for all three regions and a real-model planning smoke test; report
infeasibility explicitly when the prescribed pose has no valid complete path.
