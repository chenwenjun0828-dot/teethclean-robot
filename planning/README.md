# 牙齿点坐标系

`tooth_frames.py` 定义每个原始牙龈线点的局部右手坐标系，不改变点位置、顺序或编号：

- 原点：原始牙龈线点。
- +X：牙冠指向牙龈的方向在局部牙面上的单位投影。
- +Z：牙齿侧网格的外向单位表面法线。
- +Y：`Z × X`，牙面内的侧向切向；不是 CSV 中相邻点的连线。

当前 `2026615-gch` 口扫的默认牙龈方向为下颌 `scan -Y`、上颌 `scan +Y`，复原自备份算法。更换口扫朝向时须显式传入 `gingivaward` 或 `--gingiva-direction GX GY GZ`；旋转模型时，这个参考向量也必须跟着旋转。

## 计算与约束

优先传入当前提取生成的 `<jaw>_vertex_labels.ply`。程序核对其顶点及三角形和口扫一致；容许已有 ASCII PLY 导出的 0.0001 mm 坐标舍入误差。无标签时复用现有颜色分类、平滑和颜色区域清理。至少两个顶点标为牙齿的三角形构成牙齿侧子网格，通过 Open3D 最近三角形查询及重心坐标插值得到局部顶点法线。

外向符号由现有完整牙弓估计器判断，可纠正全局反向的三角形绕序。自动估计需要完整或较完整的牙弓；单颗牙可以直接调用 `define_tooth_frames`，显式提供外向法线。没有足够牙面支持（默认距离超过 2 mm）、零法线或牙龈方向平行于法线时会报出点编号，不生成替代法线。颜色标签属于几何估计，不是逐牙语义分割。

程序不沿 CSV 行顺序平滑，不连接断开的牙龈线，不生成缺口中的点。`point_index=250` 是当前输入 CSV 的零基索引；不同提取结果中的同一编号可能位于不同位置。可视化从外向法线侧观察，保持实际口扫坐标、牙面与箭头的深度关系；右侧三轴示意使用相同视角，单独展示被牙面遮住的轴。

## 单独运行

在项目根目录运行，无需安装新依赖。使用一次流水线的配准及对应提取结果，输出口扫、相机、世界三套坐标系：

```powershell
.\.venv\Scripts\python.exe -m planning.tooth_frames `
  --jaw mandibular `
  --registration-dir outputs\pipeline\20261001_223948_971512\registration `
  --labels outputs\pipeline\20261001_223948_971512\extraction\mandibular_vertex_labels.ply `
  --point-index 250 --out outputs\tooth_frames\mandibular_point_250
```

`--registration-dir` 会沿用报告中的口扫和被冻结的牙龈线，校验口扫/线文件哈希与配准一致；默认寻找该线同目录的 `gingival_line_segments.obj`，也可显式传 `--topology`。以后运行时替换时间戳目录。纯口扫模式：

```powershell
.\.venv\Scripts\python.exe -m planning.tooth_frames `
  --jaw maxillary `
  --scan 'models\2026615-gch\2026615-gch病例-maxillary.ply' `
  --line outputs\extraction_cleaned_maxillary\gingival_line_points.csv `
  --labels outputs\extraction_cleaned_maxillary\maxillary_vertex_labels.ply `
  --point-index 250 --out outputs\tooth_frames\maxillary_point_250
```

## 开发接口与单位

在以上命令末尾加 `--view`，导出后打开交互式三维窗口。拖动旋转、滚轮缩放；输入原始点编号或使用 Previous/Next 切换点。勾选 Local tooth surface 显示局部，取消勾选显示完整牙弓；Reset view 恢复默认视角。`--view-radius-mm 8` 可调整局部裁剪半径，默认 10 mm。

可视化函数位于 `tooth_frames.py` 后半部分：`show_tooth_frames(mesh_path, scan_frames, point_index=250, radius_mm=10.)` 打开窗口，`preview_tooth_frame(...)` 保存高清 PNG。二者都使用原始点与坐标轴，不会修改坐标系结果。牙齿与牙龈使用增强对比的展示色，青色点为原始牙龈点；显示色不是新的语义分类。静态预览通过隐藏的 OpenGL 窗口渲染，需要可用的桌面图形环境；当前 Windows 版 Open3D 不支持 EGL 离屏渲染。

牙龈边界可能受到牙龈遮挡。添加 `--tooth-surface-demo` 会从指定牙龈点沿 -X（朝牙冠）偏移 5 mm 的目标位置附近，选择最近的牙齿标签网格顶点，重新计算该牙面点的坐标系。生成单独的 `tooth_surface_preview.png`、`T_scan_tooth_surface_demo.npy`（牙齿局部→口扫，mm）和 `tooth_surface_selection.json`；它属于牙面示例点，不替换原始牙龈点。与 `--view` 同时使用时，交互窗口展示该牙面示例及原始牙龈点。

**指定点序号**：普通模式用 `--point-index 250` 选择牙龈 CSV 第 250 个点；牙面模式用 `--vertex-index 8848` 直接选择口扫 PLY 的原始网格顶点 8848。两种编号均从 0 开始，含义不同；`--vertex-index` 和 `--tooth-surface-demo` 二选一。

牙面模式的三维窗口支持输入 Mesh vertex index，或用 Previous/Next 在牙齿标签顶点间切换；普通模式仍切换牙龈点。牙面模式遇到牙龈区域或退化法线会在窗口提示，并保留上一次成功的视图；命令行输入非法顶点会报错。窗口中的选择只改变查看内容，PNG 和 NPY 保存的是启动命令指定的点，重新运行命令可保存其他点。

清晰牙面示例运行命令：

```powershell
.\.venv\Scripts\python.exe -m planning.tooth_frames `
  --jaw mandibular `
  --registration-dir outputs\pipeline\20261001_223948_971512\registration `
  --labels outputs\pipeline\20261001_223948_971512\extraction\mandibular_vertex_labels.ply `
  --out outputs\tooth_frames\mandibular_point_250 `
  --tooth-surface-demo --view
```

指定牙面点时，将最后一行改为 `--vertex-index 8848 --view`；可以更换 8848，运行后继续在窗口里切换。

```python
from planning.tooth_frames import registered_tooth_frames
from perception.gingival_line import load_registered_paths

paths = load_registered_paths(registration_dir, 'mandibular', topology_path)
frames = registered_tooth_frames(paths, mesh_path, labelled_mesh_path=labels_path)
R_world_tooth = frames.world.rotations[250]  # 列向量依次为 X、Y、Z
T_world_tooth = frames.world.transforms[250]
```

`define_tooth_frames(points_mm, outward_normals, jaw, gingivaward=None)` 可独立使用已知法线；`build_tooth_frames(...)` 从网格计算，返回 `(scan_frames, surface_distances_mm, report)`。`ToothFrames.transformed(T, coordinate_frame, units)` 同时变换原点与旋转：T 的平移必须使用**目标坐标系单位**，函数会转换输入原点的长度单位。

`T_target_tooth` 表示**牙齿局部 → target**，各轴位于旋转矩阵的列：

| 输出 | 方向 | 平移及局部点单位 |
| --- | --- | --- |
| `T_scan_tooth.npy` | 牙齿局部 → 口扫 | mm |
| `T_camera_tooth.npy` | 牙齿局部 → CV 相机 | mm |
| `T_world_tooth.npy` | 牙齿局部 → MuJoCo 世界 | m |

所有矩阵数组为 `(N,4,4)`。相机变换使用现有配准 `T_camera_scan`；世界变换复用 `RegisteredPaths.world_from_scan`。每个坐标系另导出包含原点和三轴的 CSV，`tooth_frames_report.json` 记录单位、方向、选定点矩阵及算法诊断，`surface_distances_mm.npy` 记录法线采样距离；PNG 展示指定点。纯口扫模式仅输出 scan 一套。

## TCP 与牙齿坐标系逐点对齐

`trajectory.py` 已将牙齿坐标系接入完整流水线。最终轨迹的每个采样点使用：

```text
R_world_tcp[i] = R_world_tooth[i]
p_world_tcp[i] = p_world_tooth[i] + offset_mm / 1000 * R_world_tooth[i, :, 2]
```

上述是未筛选的逐点目标；`offset_mm=0` 时 TCP 原点及 X/Y/Z 三轴均与牙齿坐标系重合。完整流水线现在默认启用 45° 姿态筛选：保留点维持上述精确对齐，跳过点替换姿态以完成运动，见下节。当前不叠加工作倾角；TCP 使用已有 `ur_scaler_tcp` 的标定位置与方向，IK 自动考虑 TCP 相对末端连杆的固定变换，无须重复乘 site 的局部旋转。

规划先选连续段、按当前 TCP 距离决定路径方向并重采样，再将**最终采样点**逆变换回口扫毫米坐标。复用 `build_tooth_frames` 计算牙面方向，并用配准 `world_from_scan` 变换三轴；不使用 CSV 行号匹配姿态，不跨断线插值。非零偏移施加后保持这些点与姿态一一对应，不再次重采样。口扫哈希须与配准一致，优先复用本次提取的 `<jaw>_vertex_labels.ply`；缺少标签时沿用牙齿坐标系模块的颜色分类流程。

主要接口：

```python
from planning.trajectory import registered_tracking_targets, plan_registered_tracking

targets = registered_tracking_targets(model, data, paths)
T_world_tooth = targets.tooth.transforms  # (N,4,4), m
T_world_tcp = targets.tcp.transforms      # offset=0 时与前者相同
plan = plan_registered_tracking(model, data, paths)
R_world_tcp = plan.rotations              # (N,3,3)，各轴为列向量
R_at_time = plan.sample_rotation(time_s)
```

`registered_tracking_targets` 只计算几何目标，不求 IK 或运动。配准目录为空时须显式提供 `mesh_path`，也可传 `labelled_mesh_path` 与口扫坐标下的 `gingivaward`。`plan_tooth_aligned_tracking(model, data, targets, speed_mm_s=4.)` 可复用几何结果进行 IK 与预检。

`plan_tracking` 兼容固定 `(3,3)` 旋转矩阵，也接受逐点 `(N,3,3)` 旋转。默认同时重采样位置与姿态，姿态用弧长 Slerp；`resample=False` 用于已经计算好最终牙齿坐标系的采样点。`plan.rotation` 仅为首点方向的兼容读取，逐点跟踪须使用 `plan.rotations` 或 `sample_rotation`。

接近与撤离分别采用首点/末点的实际目标姿态，沿各自 TCP +Z 留出 30 mm。逐点 IK 沿用上一点关节解，求解器采用整体步长限制和残差下降检查；任何 IK 或关节扫掠碰撞失败都停止执行。控制仍采用位置执行器及关节插值，点间的期望 TCP 姿态使用 Slerp 计算。规划为执行补充中间 IK 采样：按关节变化约 0.02 rad、笛卡尔间距不超过 0.25 mm 细分区间，沿原位置和 SO(3) 插值求解，以降低关节插值的 TCP 偏差；这些辅助采样不改变原始目标的位置和姿态。`plan.trace_q` 对应原始目标，`plan.command_q/command_arc` 为执行样本及其路径弧长，运动限速和碰撞预检使用全部执行样本。报告按变化的目标姿态计算误差，最终接受限为位置 RMS 1 mm、最大 2 mm、姿态最大 1°，跟踪时姿态误差超过 5° 会立即停止。

### 跳过姿态变化过大的约束

完整流水线和 `run_tracking` 默认 `max_orientation_step_deg=45.`。按执行顺序，计算当前点与**上一个保留点**的 SO(3) 旋转角；超过阈值时跳过该点的牙齿姿态约束，随后仍与上一个保留点比较。

全部空间轨迹点保留。被跳过位置的 TCP 姿态按相邻保留点的弧长进行 Slerp；超出最后保留点的位置使用最后有效姿态。因此末尾异常点不会截断空间路径，也不会直接连接远处保留点而抄近路。保留点三轴精确对齐牙齿；跳过点的三轴不再强制对齐。在只有一个保留姿态时，整段沿用该姿态。筛选不会自动跳过其他 IK 或碰撞失败。

```powershell
.\.venv\Scripts\python.exe -m pipeline.run --jaw mandibular `
  --registration-method automatic --max-orientation-step-deg 45 --record-gif
```

添加 `--headless` 可不打开执行 Viewer；添加 `--no-orientation-filter` 恢复严格逐点对齐。底层 `plan_registered_tracking` 默认仍是严格对齐，可显式传 `max_orientation_step_deg=45.`；`filter_tooth_orientation_targets(targets, 45.)` 可先得到筛选结果，再调用 `plan_tooth_aligned_tracking`。

筛选后报告 `tool_pose="tooth_aligned_filtered"`，`orientation_filter` 记录阈值、保留/跳过编号、角度、比较对象及替换方式。`orientation_selection.csv` 标记每个原始采样点是否保留牙齿姿态约束，编号指最终重采样轨迹的零基索引。

tracking 目录新增以下输出，**在 IK 前保存**，规划失败也能检查目标：

| 文件 | 内容 |
| --- | --- |
| `T_world_tooth.npy` | 最终路径点的牙齿局部→世界变换，平移 m |
| `T_world_tcp.npy` | TCP 局部→世界目标变换，平移 m |
| `requested_T_world_tcp.npy` | 筛选前的原始 TCP 目标；offset=0 时与牙齿矩阵一致 |
| `target_rotations_world.npy` | `(N,3,3)` TCP 旋转矩阵 |
| `target_poses_world_m_rad.csv` | 逐点 xyz 和 roll/pitch/yaw，m/rad；`Rz(yaw) Ry(pitch) Rx(roll)` |

`tracking_report.json` 标记 `fixed_orientation=false`，记录所有实际目标方向、牙面估计信息、筛选情况及实际执行误差；`tcp_tracking.csv` 增加 `orientation_error_deg` 列，仅 trace 阶段填写。`joint_waypoints_rad.csv` 包含全部辅助执行样本，`trace_joint_commands.csv` 额外记录各执行样本的 `arc_m`。

### 当前场景验证结果

当前下颌路径有 70 个最终采样点。45° 筛选保留 15 个原始牙齿姿态、跳过 55 个姿态约束，全部空间点及终点保留；加入 235 个执行 IK 样本后完成整段运动，位置 RMS 0.006965 mm、最大 0.155875 mm，姿态最大 0.091999°。仿真计划时长 276.188 s，最大实际关节速度 0.288051 rad/s，到原始注册牙龈线最大距离 0.127264 mm。输出、筛选明细、预览与动画位于 `outputs/tcp_tooth_skip45_complete/`，该运行使用 geometry 模式，不验证牙模接触。

关闭筛选后，offset=0 的 TCP 与牙齿目标矩阵逐点一致，但当前布置在索引 4 发生 IK 失败，不开始运动。严格对齐结果保存在 `outputs/tcp_tooth_alignment_actual_path/`，几何不可达原因见其中的 `ik_point_4_diagnosis.json`。

另外，可达的变化姿态测试夹具用于独立验证规划及位置执行器链路，严格对齐模式也可完成运动。

验证命令：`.\.venv\Scripts\python.exe -m unittest tests.test_tooth_frames -v`。

逐点规划与控制验证：`.\.venv\Scripts\python.exe -m unittest tests.test_tcp_tooth_alignment tests.test_planning_control tests.test_modular_pipeline tests.test_automatic_registration -v`。
