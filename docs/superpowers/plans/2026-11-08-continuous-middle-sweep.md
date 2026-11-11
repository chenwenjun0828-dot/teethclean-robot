# Implementation plan

1. Add serializable composite source/connector geometry and bounded trimming options.
2. Add all-or-nothing global roll/IK search, trace refinement and single-sweep export.
3. Integrate a CLI option, reference-aware audit and guarded execution loading.
4. Test geometry, graph failure, structural validation and independent pose audit.
5. Run real jaw planning, diagnose failed connections, verify saved trajectories and
   headless dynamics, then document commands and measured results.

Use the existing scene, TCP, 15 degree tilt, 0.15 mm collision margin, 0.1 mm path
tolerance and 1 degree pose tolerance. No changes to the old segmented algorithm.
