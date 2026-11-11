# Configuration

当前六区域主流程参数示例是 `pipeline_outer_front.json`：

```powershell
.\.venv\Scripts\python.exe -m pipeline.run `
  --params config/pipeline_outer_front.json --headless
```

其他配置文件分别负责：

- `global_camera_registration.json`：仿真相机、配准和扫描路径。
- `real_camera_registration.json`：RealSense 采集和实机配准参数。
- `pipeline_outer_front.example.md`：六区域参数含义与运行边界。
- `oral_settings.py`：口腔开口和下颌张口角。

## Planning 命令参数

每个命令均支持 `--params`；参数只属于表中对应命令，不能将 pipeline 的 JSON 直接混用。

| 命令 | 参数文件 | 用途 |
| --- | --- | --- |
| `python -m planning` | `planning_outer_front.json` | 读取感知结果，进行完整规划与验收 |
| `python -m planning.visualization.pose_angles` | `planning_pose_angles.json` | 三角度交互滑块选角 |
| `python -m planning.view` | `planning_view.json` | 轨迹回放、截图或 GIF |
| `python -m planning.preview` | `planning_preview.json` | 连续参考与 TCP 轨迹诊断图 |
| `python -m planning.sweep_preview` | `planning_preview.json` | 直接导出同一连续路径诊断图 |
| `python -m planning.segment_map` | `planning_segment_map.json` | 将区域诊断图导出至指定目录 |
| `python -m planning.retime` | `planning_retime.json` | 在保留路径的前提下重新计时并验收 |
| `python -m planning.execute` | `planning_execute.json` | 加载并执行已验证的仿真轨迹 |

这些命令对应的 `planning.visualization.*`、`planning.motion_planning.*` 实现入口也支持同样参数。
角度查看器默认使用附带数据，可在准备感知结果之前单独打开：

```powershell
$py = ".\.venv\Scripts\python.exe"
& $py -m planning.visualization.pose_angles --params config/planning_pose_angles.json
```

主规划参数文件默认读取 README 第 3 节生成的 `outputs/tutorial/manual/regions/outer_front`，
输出至 `outputs/planning/outer_front`。其余轨迹命令默认读取该输出目录。
使用自己的感知结果或输出路径时，修改 JSON 或通过 CLI 覆盖：

```powershell
& $py -m planning --params config/planning_outer_front.json --dry-run
& $py -m planning --params config/planning_outer_front.json --out outputs/planning/my_front
& $py -m planning.view --params config/planning_view.json --plan outputs/planning/my_front --no-paused
& $py -m planning.preview --params config/planning_preview.json --plan outputs/planning/my_front
```

`planning_retime.json` 指定目标总时长 90 s；改变 `duration` 时建议同时修改 `out` 的目录名。
`planning_execute.json` 默认 `original_timing=true`，直接使用保存的轨迹时序；
`--no-original-timing --duration 90` 才启用重计时，其速度上限与总时长仍须通过审计。
主规划默认 `execute=false`，仅生成并验收轨迹。每次规划或重计时均需使用新的空输出目录。

本轮干净环境中，主规划、回放、导图、原时序执行及 90 s 独立重计时均通过。
重计时保留源轨迹的关节速度、加速度和角速度限制，已修复原先自动放大限制引起的 TOPPRA 数值失败；
不能通过缩短时长自动放宽限制。执行重计时结果时，显式将 `--plan` 指向新输出目录。
修复及实测记录见 [README 命令验收](../docs/integration/2026-11-11-readme-command-audit.md)，
修复前记录保存在 [参数文件验证](../docs/integration/2026-11-11-planning-parameters.md)。

## 文件格式与覆盖

参数文件中的路径相对于项目根目录。命令行参数优先于 JSON；每次修改规划参数都要使用新的输出目录。
`--params` 的文件路径也相对于项目根目录；原有命令行路径继续相对于当前工作目录。
参数优先级为内置默认值 < JSON < 显式命令行；不指定 `--params` 时保持原用法。
`--config` 只选择 pipeline 的相机/配准配置，`--params` 选择对应命令的运行参数。

JSON 顶层为对象，键名使用 argparse 的参数名，通常将 CLI 中的 `-` 改为 `_`。
数组如 `pose_angles_deg: [6.37, 13.26, -0.4]`，布尔值使用 `true`/`false`，
可选未设置的参数可以用 `null`。不接受注释、重复键、未知键、非有限数字或不匹配的类型。
必填的 `plan`、`out` 可以只在文件中提供；遗漏时仍然报错。
`--headless/--no-headless`、`--paused/--no-paused` 等可以覆盖文件中的布尔值。
文件中的 `no_browser=true` 用 `--browser` 取消，`no_output=true` 用 `--save-output` 取消。
pipeline 使用 `offset_mm`/`output`；独立 planning 使用 `standoff_mm`/`out`。

六区域源数据及顺序由 `assets/regional_paths/` 的 manifest 决定。
旧三分区配置已删除；主流程和主规划示例不锁定总时长，也不扩大裁剪范围。
