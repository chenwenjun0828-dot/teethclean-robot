# Project Structure

当前运行时只有一条主链：`perception -> planning -> control`，由 `pipeline` 负责串接。

| 目录 | 职责 |
| --- | --- |
| `perception/registration/` | RGB-D/点云配准和配准质量检查 |
| `perception/gingival_extraction/` | 从彩色口扫提取牙龈线和清理中间结果 |
| `perception/tooth_segmentation/` | CrossTooth 预测验证、实例清理和牙齿标签 |
| `perception/regions/` | 六区域分区、逐牙平面、连续曲线、端点裁剪和牙间连续性保留 |
| `perception/regional_paths.py` | 固定六区域数据集读取和来源指纹检查 |
| `planning/pose_search/` | 固定姿态构造和连续 IK 分支搜索 |
| `planning/motion_planning/` | 进场、退出、时序、碰撞、轨迹和动力学审计 |
| `planning/infrastructure/` | 机器人、网格、轨迹、变换和文件 IO 基础设施 |
| `planning/visualization/` | 交互式角度滑块、已保存轨迹回放和诊断图，含离线网页依赖 |
| `control/position/` | 已验证轨迹的位置控制仿真 |
| `control/force/` | 独立恒力和接触轨迹验证 |
| `control/hardware/` | Mock、MuJoCo 和预留真机后端接口 |
| `pipeline/` | 完整 pipeline、阶段适配器、口腔设置和输出策略 |
| `assets/simulation/` | MuJoCo 场景、机器人、牙颌、口腔和工具网格 |
| `assets/source_cad/` | 英文命名的源 CAD 与扫描网格 |
| `assets/perception/` | 固定牙龈线、分割预测和来源证明 |
| `assets/regional_paths/` | 六区域有序路径和坐标变换 |
| `config/` | 配准、口腔和六区域主流程参数 |
| `docs/integration/` | 当前合并、力控和零 offset 验证记录 |

## 数据边界

Perception 输出 `PreparedRegion`，其中包含连续分区牙龈线、来源指纹、逐牙平面和 scan-to-world 变换。
Planning 只读取这些数据，生成碰撞审计后的关节轨迹。Control 只读取执行就绪的轨迹。

长度和扫描几何使用毫米，世界位置使用米，关节角使用弧度，时间使用秒，力使用牛顿。
每个阶段的报告都保存输入指纹，源网格或分区结果改变后必须重新运行。

## 常用命令

```text
python -m pipeline.run --dry-run
python -m pipeline.run --params config/pipeline_outer_front.json --headless
python -m planning.view --plan <run>/tracking
python -m control.force --help
```

六区域数据集为必需输入；当前只附下颌数据。每次选择一个区域，整区端点裁剪后保留源曲线连续性，
不按牙齿重建或补线。提取线仅继续作为配准中间数据使用。旧三分区流程已于 2026-11-11 移除。
历史设计与验收保留在 `docs/superpowers/` 和历史 integration 记录中，不代表当前运行入口。
