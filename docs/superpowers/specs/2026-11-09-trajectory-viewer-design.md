# Planning Trajectory Viewer

The user approved the existing MuJoCo trajectory preview and requested that it
become a reusable part of `planning`. Preserve that demonstrated behavior while
extending the existing `python -m planning.view` command, rather than introducing
a second trajectory viewer. A separate command would duplicate scene loading and
controls; keeping the temporary script would leave hardcoded paths and jaw angles.

## Behavior

- Load the specified saved plan without retiming, replanning, or running control.
- For joint trajectories, use the existing verified loader, including scene,
  mounting pose, dental poses, and trajectory fingerprint checks.
- Infer the runtime jaw opening from a saved timing profile when available;
  `--jaw-opening-deg` explicitly selects an opening and must pass the same checks.
- Preserve previews of geometry-only planning outputs.
- Play the actual `Trajectory.sample(t)` interpolant at `--speed` (default 5).
  Space pauses/resumes, N/P pause and select adjacent waypoints, and R resets.
  `--paused`, `--time`, and the existing `--waypoint` select a debugging state.
- Keep the demonstrated tooth model, teal sweep, orange connectors, and red TCP
  marker. Cache bounded overlays instead of redrawing every dense source sample.
- Offer the demonstrated camera presets (0-11, default 10), automatic distance,
  and an optional explicit camera distance in meters.
- Preserve `--snapshot PATH` and add `--gif PATH` with configurable frame rate.
  Exports use the same camera, overlays, and interpolation as the live viewer.
  GIF labels show saved plan time, phase, and playback multiplier.
- Keep imports free of rendering side effects, make errors actionable, and
  document the reusable commands and keyboard controls.

## Implementation And Validation

Keep CLI orchestration and existing public functions in `visualization/view.py`.
Keep trajectory clock/stepping and GIF sampling in a small playback helper. Reuse
the existing trajectory, verification, reference-path, and MuJoCo drawing APIs.
The temporary preview and generated plan files stay available.

Focused tests cover timing/stepping, argument validation, jaw-profile inference,
strict trajectory loading, bounded overlays, and geometry-only compatibility.
Run the planning regression suite and render a snapshot plus a short GIF from
the current verified 90-second plan. Inspect the output and launch the packaged
viewer to confirm it opens successfully. No motor interface is involved.
