# Pipeline Run Parameters

The user requested a file in config/ to replace repetitive pipeline CLI options.
Add --params for a flat JSON object using CLI destination names (underscores).
Keep --config as the existing camera/registration configuration selector.

Precedence is existing defaults, then file values, then explicit CLI values.
Both paths and numeric/choice/boolean values are checked before a run; unknown
or duplicate keys and malformed JSON are rejected. Reuse argparse metadata to
avoid duplicating the CLI schema. Existing algorithm-level range and consistency
checks remain authoritative. Mutually exclusive options are checked after merging.

File path values are relative to the project root; absolute paths are accepted.
--params itself also resolves relative to the project root. CLI path behavior is
unchanged. JSON null restores an optional unset field, but is invalid for required
numeric, string or boolean values. Angle tuples remain mutually incompatible with
individual angle overrides, even when supplied from different sources.

The sample config/pipeline_middle_90s.json contains the current validated middle
region setup. It omits output so the existing unique timestamp directory is used
on every run. --output can select an explicit empty destination. --no-headless
overrides the sample's headless=true for a visible simulation window.

No parameter file is loaded implicitly. Existing CLI and Python entrypoints retain
their current defaults and motion algorithms. Dry runs display the resolved report.

Verify file-to-runtime forwarding, CLI precedence, input types, path resolution,
cross-source conflicts, old CLI behavior, real dry runs and the complete simulation.

## Validation

- 183 project tests passed, including 11 parameter-file tests.
- Two planning CLI/API compatibility tests passed.
- The real sample dry run accepted CLI duration=85 and created no output.
- Full file-based automatic pipeline completed at
  outputs/pipeline/params_file_90s_20261110/tracking.
- 1141 pose nodes, 90 seconds total motion, completed dynamics and execution_ready.
- Minimum actual clearance: 0.889480 mm; total pipeline wall time: 86.86 s.
- The headless pipeline reused its completed dynamics validation.
