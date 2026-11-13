# Pipeline

`pipeline.run` 是当前仿真项目的端到端入口。它只负责组织阶段，不在 planning 中重构牙龈线。

## 阶段

1. `perception.workflow.perceive_gingival_line` 提取并配准扫描，调用 `perception.regions` 输出连续分区牙龈线。
2. `pipeline.tracking` 校验 perception 输出和来源指纹，把 `PreparedRegion` 交给 planning。
3. `planning.pipeline.run` 完成固定姿态、IK 分支、碰撞检查、进场/退出运动连接、关节样条和动力学审计。
4. `control.position.execute` 执行已验证的轨迹并记录跟踪误差和碰撞状态。

阶段适配器位于 `pipeline/stages/`，完整流程默认通过 `pipeline.run` 调用。

## 当前运行

```powershell
.\.venv\Scripts\python.exe -m pipeline.run `
  --params config/pipeline_outer_front.json --headless
```

也可以覆盖单次运行参数：

```powershell
.\.venv\Scripts\python.exe -m pipeline.run `
  --params config/pipeline_outer_front.json `
  --headless `
  --output outputs/pipeline/outer_front_run
```

`--dry-run` 只验证路径、输入和配置；`--no-output` 使用临时目录执行完整流程；
`--check-mode geometry_only` 会跳过碰撞和动力学，只验证可达性，结果永远不可执行。

六区域输入是必需项，缺失或与扫描、分割预测不匹配时，在创建仿真前报错。
当前仅附下颌数据集；`--region` 默认 `outer_front`，只规划该区。
整区仅在起终点裁剪，不逐牙删段、不补线，也不会自动连接六个区域。
参数示例未锁定总时长；时间由物理约束决定。验证情况见
`../docs/integration/2026-11-11-six-region-cleanup.md`。

## 姿态策略

默认 `planning_mode=simple`。三角度的顺序固定为：

```text
fixed_z_angle, fixed_y_azimuth, tool_x_flip
```

规划器先使用区域统一姿态；碰撞或 IK 失败时，在每个角度 ±5° 的有界网格内搜索，必要时使用连续的小幅修正。
`planning_mode=continuous` 仅在需要局部牙面参考姿态时显式启用。

`offset_mm=0` 是几何目标，仍必须通过完整网格碰撞检查。主 pipeline 是位置控制；恒力实验请使用
`python -m control.force`，不会由此入口隐式开启力控。

## 输出

持久化运行目录包含：

- `pipeline_report.json`：阶段状态、输入指纹和总耗时。
- `extraction/`、`capture/`、`registration/`：perception 中间产物。
- `regions/`：perception 输出的 `PreparedRegion` 文件。
- `tracking/`：planning 报告、轨迹、连续路径和执行验证。

输出目录必须为空；复用参数时使用新的输出目录，避免覆盖已验证结果。
