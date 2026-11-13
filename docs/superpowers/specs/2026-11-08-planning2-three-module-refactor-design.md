# Three-module organization for planning2

The user requests organizing the existing implementation into gingival path
reconstruction, whole-path pose search, and robot motion planning. Preserve
numerical algorithms, CLI options, saved output schemas, and the existing verified
90 second trajectories.

`path_reconstruction` owns labelled sources, three-point planes and continuous
source/bridge geometry. `pose_search` owns target frames, directed Y/Z references,
reachability, roll/IK candidates and global branch selection. `motion_planning`
owns interpolation refinement, approach motion, timing, physical axial kinematics,
independent audits, structural contracts and execution loading.

Shared scalar parameters live in config.py. pipeline.py orchestrates the stages;
__main__.py parses arguments. The motion planner calls reachable_source_windows,
build_sweep and solve_sweep in order, retrying path edits when selection fails.
solve_sweep is the complete candidate-to-selected-pose interface: it encapsulates
the graph and node/edge collision callbacks. Timing calls audits in a refinement loop.

Move plotting to visualization and contact experiments to experiments. Keep old
module names as explicit, short forwarding modules so existing commands, tests
and control imports work. Core implementations use canonical module paths and
never depend on compatibility modules. Forwarders contain no algorithm copies.

Verify existing tests, module dependency direction, compatibility identities,
the solve_sweep interface with actual geometric targets and a deterministic robot,
CLI argument parsing, and saved upper/lower trajectory loading and full FK/rate
audits. Audit copies of metadata so original result files remain intact; compare
hashes of saved trajectories, reports and curve references before and after.
