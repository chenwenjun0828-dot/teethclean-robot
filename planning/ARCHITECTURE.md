# Planning Architecture

Planning consumes a validated `perception.regions.PreparedRegion` and produces one continuous,
collision-audited robot trajectory.

```text
PreparedRegion
  -> pose_search (fixed regional pose and connected IK)
  -> motion_planning (approach, trace, retreat, timing, audit)
  -> planning_report.json + trajectory.npz
```

Curve fitting, tooth assignment, endpoint trimming and source continuity across tooth boundaries are perception
responsibilities. The planner never rebuilds those objects and rejects a missing or changed perception
artifact through its file fingerprints.

`simple` is the default pose policy: a regional fixed angle tuple followed by bounded ±5 degree corrections
when the nominal pose is blocked. `continuous` is an explicit local-frame alternative. Both policies use the
same IK graph, full-tool collision checker and trajectory validation.

All planning CLI commands accept `--params config/<command>.json` through the shared
`pipeline.parameters` loader. It derives validation from each command's argparse schema,
merges defaults < JSON < explicit CLI, and resolves JSON paths relative to the project root.
Required paths may come from JSON; cross-source conflicts are checked after merging.
This changes command configuration only, not the saved `PreparedRegion` or trajectory contracts.
