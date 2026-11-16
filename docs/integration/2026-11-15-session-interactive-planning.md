# Session 交互式规划验收

- 日期：2026-11-15。
- 审核依据：[已批准方案及补充加载器修复](../superpowers/specs/2026-11-15-session-interactive-planning-design.md)。
- 完成范围：session 一致预览、页面选参、单任务规划/取消、固定参数快照、结果与日志、实际 FK TCP 路径、配置导出及已有回放命令。
- 实施文件：`planning/visualization/pose_angles.py`、`planning_jobs.py`、`planning_results.py`、`pose_angles_web/`，相关测试、浏览器工具和模块说明；最小加载修复位于 `planning/motion_planning/execution.py` 与 `planning/visualization/view.py`。
- 未新增依赖，未改变正式配置、标定资产、规划策略或安全阈值。保留已有工作区修改。

## 已通过的完整任务

页面提交任务：`outputs/interactive_planning/20261115_140144_7553461f/`。
`parameters.json` 保存提交输入，`planner.log` 保存真实 CLI 输出，`task.json` 记录 `succeeded`；
`plan/` 中保留来源记录、关节轨迹、完整审计和动力学产物。
此前加载失败任务未改写；修复后以新的页面任务独立验收。

| 参数 | 值 |
| --- | --- |
| Session | `assets/perception/sessions/20261113_220209` |
| 相机解释 | `color_optical` |
| 牙颌 / 区域 | `mandibular` / `outer_front` |
| 姿态模式 | `continuous`，局部严格角度 |
| 三角度 | `20 / -20 / 10` 度 |
| 离面距离 | `3 mm`，非接触 |
| 沿线 / 进出场速度 | `1 / 10 mm/s` |
| 姿态采样间距 | `0.25 mm` |
| 检查 / 执行 | `full` / `execute=false` |

结果：430 个姿态点连接成功，总时长 `108.4769534006265 s`，进出场 `45.832 s`，沿线 `62.645 s`。
报告最小几何间隙 `1.1444585365835338 mm`；覆盖、曲线审计、运动审计及动力学均通过。
页面独立调用 `load_verified` 成功；`load_scene` 能加载同一完整轨迹，保留原有全部来源和场景检查。
显式覆盖 session 张口角 `0` 或 `20` 度仍被拒绝；普通场景保存/覆盖张口角回归通过。

## 实际验证

```powershell
$py = ".\.venv\Scripts\python.exe"
& $py -m unittest planning.test_interactive_planning planning.test_pose_angles planning.test_view tests.test_perception_session tests.test_planning_parameters -q
& $py -m unittest discover -s planning -t . -p 'test_*.py' -q
& $py -m unittest discover -s tests -p 'test_*.py' -q
node planning/tools/check_interactive_planning.cjs http://127.0.0.1:8766/ outputs/interactive_planning/full_browser_validation --full
git diff --check
```

- 针对性回归：59 项通过；规划发现入口：129 项通过；跨模块发现入口：319 项通过。
- 原姿态浏览器回归通过，结果在 `outputs/interactive_planning/browser_validation_default/`。
- 新交互浏览器回归验证真实可达性任务、参数不受运行中编辑影响、配置导出、取消与无虚假通过，结果在 `outputs/interactive_planning/interactive_browser_validation/`。
- 完整规划浏览器回归通过，结果在 `outputs/interactive_planning/full_browser_validation/interactive_checks.json`；1440、390、320 像素宽度均检查截图、画布非空像素和控件溢出。
- 实际 TCP 显示通过保存轨迹的原插值和 `Robot.fk` 采样测试；预览在两种相机解释、两个区域、三个姿态模式及起中终点与权威规划接口一致。
- 无报告、部分轨迹、来源变化、非法参数/请求、并发提交及取消均有回归覆盖。`git diff --check` 通过。

## 当前限制

`color_optical` 解释仍需现场核实，`robot_execution_allowed=false`。本次仅证明指定下颌区域和参数在当前场景中通过非接触仿真验收，未进行真机验证或接触/力控验收。
当前上颌区域输入不完整；显示双颌不等于可以规划上颌。其他区域和参数需重新规划与验收。
页面重载可以读取本服务最近任务；服务重启不会恢复任务。重新打开时如默认输入不同于任务快照，页面会明确标记待重新规划。
