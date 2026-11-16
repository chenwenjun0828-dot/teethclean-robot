# Force Control

当前真实 session 的几何轨迹、复现命令和力控接入说明见
[开发交接指南](../../docs/integration/2026-11-16-developer-handoff.md)。
当前 geometry_only 轨迹不能通过本模块的 `load_verified` 门槛；现有实验循环还会重新创建
非 session 仿真场景，且直接依赖 MuJoCo 传感器和 actuator，尚无真机恒力切换入口。
接入 session 场景或真实后端属于后续核心改动，需先按 `AGENTS.md` 提交具体方案审核。

The force stage contains the compliant contact configuration, predictive force
guard, contact-path runner, force history, and force-panel visualisation. The
implementations live together in this subpackage; `runner.py` is the stable
command-line boundary.

Run `python -m control.force --source <verified-tracking-directory> --out <empty-directory>`.
Trajectory execution is the default; explicit `--mode trajectory` is also accepted.
`--source` is required, so no historical output is selected implicitly. The old
segmented-plan probe CLI and `--sweep-mm` option have been removed. Shared collision
checks, signal logging, and the isolated stop-behavior test probe remain in
`contact_experiment.py`.

Continuous regional sources now default to one acquisition and one uninterrupted
regional force-controlled trace. `--contact-schedule segments` retains the older
per-arc acquire/release experiments. Regional execution preserves the saved
geometry and orientation policy; it does not reconstruct the perception path.

Every full-path run also audits the saved sensor log and replays measured joints
against the original scanned triangle meshes. Force tracking alone cannot pass
`force_control_passed` when the physical convex proxies are still separated from
the scanned tooth surface. See the [current validation](../../docs/integration/2026-11-11-contact-force-validation.md).

`python -m control.force.benchmark --out <empty-directory>` runs a separate 1 N
sphere/plane controller component test, not a dental trajectory certification.

![Current mandibular force attempt](../../docs/images/force_control.png)

The documented front-region attempt stopped at 7.141 s on the 2.5 N predictive
guard, before cleaning started. It did not pass dental contact validation.

![Independent controller component benchmark](../../docs/images/force_benchmark.png)

The sphere/plane benchmark passes independently. See the [project guide](../../README.md)
for commands, output fields, and the distinction between these acceptance results.
