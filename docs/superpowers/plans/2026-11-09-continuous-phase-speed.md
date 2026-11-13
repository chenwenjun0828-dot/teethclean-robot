# Implementation Plan

Approved design: `../specs/2026-11-09-continuous-phase-speed-design.md`.

1. [x] Default the standalone planning CLI to continuous; preserve explicit
   `--continuous`, add `--segmented`, and keep geometry-only mode available.
2. [x] Add trace downsampling that preserves non-trace nodes, tooth boundaries,
   joint/orientation thresholds, and original sweep parameter/roll mapping.
   Reinsert original nodes when independent interpolation/collision audits fail.
3. [x] Extend retiming with separate trace/free-space speeds, jaw-opening restore,
   phase durations, target-duration validation, and fresh exported references.
4. [x] Default `planning.execute` to a validated 90 s profile, with explicit original
   timing and reusable output only when source/profile/scene fingerprints match.
5. [x] Test routing, reduction invariants, timing limits and execution selection.
6. [x] Generate and dynamically validate the supplied lower-jaw plan at 20 degrees
   jaw opening. Record actual duration, phase times, rates, errors and clearance.

The original source plan stays unchanged. Default fast profile: 90 s target,
1.0 mm/s trace speed limit, 150 mm/s free-space nominal limit, 0.1 mm trace
sampling, and the existing simulation joint/acceleration/orientation bounds.

Validation adjustment: a 0.7 mm/s peak trace limit required over 100 s with
the actual local interpolation. A 1.0 mm/s peak limit is used to meet the
90 s target; spare time still stretches trace motion, keeping its mean speed
near 0.5 mm/s while free-space phases remain fast.
