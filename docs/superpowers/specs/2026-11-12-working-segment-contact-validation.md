# Working-Segment Contact Validation

Date: 2026-11-12

These results describe the original v2 contact verifier. The subsequent repair
of overly broad local support is documented in
2026-11-12-local-contact-support-fix-design.md. The completed verification in
2026-11-12-local-contact-support-fix-validation.md supersedes the v2 folded-support
diagnoses; those historical reports are retained.

## Conditions

- Prepared source: outputs/tutorial/manual/regions/outer_front.
- Mode: simple, full collision checking; angles [50, 0, 50] degrees.
- Existing source curves and registered transform retained; jaw opening 20 deg.
- Working sphere radius 1 mm, contact tolerance 0.05 mm, forbidden clearance 0 mm.
- Existing +/-5 degree correction grid, approach/retreat and trajectory checks.
- The initial zero-standoff run allowed per-position local correction. At the
  user's subsequent request, both later full runs disabled it using
  local_angle_search=false. The 125 common-angle candidates remain enabled.

## Zero Standoff

Full run: outputs/planning/contact_front_50_0_50_zero.

Result: no_continuous_outer_front_sweep, execution_ready=false.
Failure: no_regional_pose_within_5_deg, layer 0, tooth 12, parameter 0.
The requested 1853-position path cannot be planned because its start has no
accepted candidate under the existing angle bounds.

At the nominal first pose all eight IK branches exist. Forbidden parts remain
separated by approximately 0.359917 mm, but the conservative directional working
gap is approximately -0.142083 mm, beyond the -0.05 mm allowed bound.

An independent work-only start diagnostic checked all 125 correction triples.
All had supported local sign; none met the tolerance. The best working gap was
-0.108075456 mm at correction [-5, -5, 5] degrees. This diagnostic is not an IK,
forbidden-part or trajectory certificate; full planning independently failed.

## Positive Standoff

The user subsequently requested 0.14 mm. Explicit contact mode now permits a
nonnegative target standoff while keeping the same contact policy; inference of
ordinary positive-offset noncontact mode remains unchanged.

Full run: outputs/planning/contact_front_50_0_50_offset014_global.

Result: no_continuous_outer_front_sweep, execution_ready=false.
Failure: no_common_angle_correction_within_5_deg; all 125 common-angle candidates
were tested, with local_angle_search=false. No executable trajectory was
generated and dynamics validation was not reached.

The work-only first-position diagnostic found 124 supported and tolerance-valid
corrections out of 125. The nominal gap was approximately +0.022979 mm. This
establishes improvement only at the start, not feasibility of the complete path.

## Increased Contact Tolerance

The user additionally requested zero standoff with a 0.2 mm working tolerance.
This is a run-specific override; default tolerance remains 0.05 mm. The original
125-pose start diagnostic already shows the nominal -0.142083 mm gap would meet
this larger numerical bound.

Full run: outputs/planning/contact_front_50_0_50_zero_tol020_global.

Result: no_continuous_outer_front_sweep, execution_ready=false.
Failure: no_common_angle_correction_within_5_deg; all 125 common-angle candidates
were tested, with local_angle_search=false. No executable trajectory was
generated and dynamics validation was not reached.

## Mandatory-Sample Diagnostic

At layer 115, curve parameter 4.053613 mm, tooth 12, an independent diagnostic
checked all 125 angle corrections and eight IK branches (1000 checks) for each
of the later configurations. Both produced:

- 973 contact_unverifiable:folded_or_conflicting_local_support rejections.
- 27 self-collision rejections (20 forearm/scaler and seven forearm/sensor).
- Zero accepted candidates.

The local jaw support does not permit a reliable working-gap certificate there;
this is not a confirmed penetration measurement. Increasing contact tolerance
alone does not resolve that rejection. These results concern the discrete
candidate grid and current conservative verifier, not physical impossibility or
all possible continuous angles.

Diagnostic artifacts:

- outputs/planning/contact_front_50_0_50_offset014_check/mandatory_probe_diagnostic.json
- outputs/planning/contact_front_50_0_50_zero_tol020/mandatory_probe_diagnostic.json

The earlier full runs in those diagnostic directories were interrupted when
local search was disabled. The completed full-run results are in the two
directories ending in _global listed above.

## Reproduction

The contact example disables per-position correction explicitly. The CLI can
override this with --local-angle-search or --no-local-angle-search. Existing
configurations that omit the field retain the previous enabled behavior.

```powershell
.venv/Scripts/python.exe -m planning --params config/planning_contact.json --no-local-angle-search --standoff-mm 0.14 --out outputs/planning/contact_front_50_0_50_offset014_global
.venv/Scripts/python.exe -m planning --params config/planning_contact.json --no-local-angle-search --contact-tolerance-mm 0.2 --out outputs/planning/contact_front_50_0_50_zero_tol020_global
```

## Regression

Final planning discovery: 116 tests passed. The external contact/configuration,
saved-plan, position-control and existing contact-workflow suite passed 104 tests,
including 32 geometry/backend tests. Total: 220 passed after the contact-query
performance changes and local-search option were implemented.

Tests cover the option's config and both CLI paths, legacy defaults, early exit
before per-position search, and complete node/edge checks for accepted common
angles. Contact-policy load, retime and runtime boundaries are covered by tests;
no successful full contact trajectory was available for end-to-end execution.
