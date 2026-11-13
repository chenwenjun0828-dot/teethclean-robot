# 轴向 X / 径向 Y TCP 验证

日期：2026-11-07。模型 TCP 原点仍位于洁治器尖端，物理 +X 沿直杆轴向朝工作端，+Y 沿弯曲平面径向，+Z=+X×+Y。规划将这个物理 +X 严格对齐牙龈线切线；切线与每牙固定参考平面的夹角由几何决定，15°仅作为参考。

## 验证内容

- 从洁治器实际编译网格的 40 个圆形截面顶点拟合直杆轴线；截面拟合平面残差约 `2.18e-5 mm`。
- 修改 `models/universal_robots_ur5e/ur7e_with_tool.xml` 中 `scaler_tcp` 的四元数，TCP 位置、洁治器网格、力传感器和相机不变。
- Planning、audit、control 共用 `tcp_frame_id=shaft_axial_x_radial_y_v1`，旧尖端方向 TCP 的轨迹会被加载器拒绝。
- 三轴图保存在 `outputs/planning_v2/axis_confirmation_shaft_frame/`，包括完整模型、三组正投影和原 X-Y 视角对照图。

## 新帧规划和仿真

| 颌 | 规划目录 | 中区覆盖率 | 实测平面夹角 | 最大 X/切线误差 | 最大位置跟踪误差 | 最小实际净距 | 状态 |
|---|---|---:|---:|---:|---:|---:|---|
| 下颌 | `outputs/planning_v2/shaft_x_tangent_mandibular` | 6.59% | 17.08°–39.43° | 0.504° | 0.213 mm | 0.586 mm | completed |
| 上颌 | `outputs/planning_v2/shaft_x_tangent_maxillary` | 46.34% | 0.01°–41.60° | 0.611° | 0.365 mm | 0.436 mm | completed |

两条新轨迹均完成独立完整 FK 审计和 MuJoCo control 仿真，未触发碰撞或跟踪误差停止。轨迹仍是非接触、离面规划；中区覆盖率是通过 IK、碰撞、连续性和进出路径筛选后的保留弧长，不能解释为整区已完成。左右区仍只输出几何。

## 检查

- `python -m unittest discover -s tests -q`：82 项通过。
- `git diff --check`：通过。
- 下颌和上颌的 `trace_audit.json`：`passed=true`。
- 下颌和上颌的 `execution_report.json`：`status=completed`。

旧目录 `x_tangent_*` 属于修改前的 TCP 方向；新坐标系必须使用 `shaft_x_tangent_*` 目录或重新规划的新输出。
