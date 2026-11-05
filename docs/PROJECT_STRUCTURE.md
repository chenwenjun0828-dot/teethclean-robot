# 洁牙机器人仿真项目结构与实现状态

> 更新日期：2026-10-02。主流程：`python -m pipeline.run`。重构前的完整目录备份位于桌面同级的 `teethclean robot_backup_2026-09-30`；移出的代码也存放在该目录的 `removed_after_refactor/`。

## 代码结构

```text
teethclean robot/
├─ perception/
│  ├─ gingival_extraction/extract.py  彩色口扫 PLY 提取入口、输出及三维叠加窗口
│  ├─ gingival_extraction/cleanup.py  孤立颜色区域、扫描边缘伪线清理
│  ├─ gingival_extraction/outer_surface.py  咬合平面/牙弓估计、外侧牙龈线自动筛选
│  ├─ gingival_line.py                OBJ 拓扑排序、配准结果读取、世界坐标转换
│  ├─ global_camera.py, frames.py     RGB-D 采集、点云及坐标系
│  ├─ registration/                  配准算法开发目录
│  │  ├─ __init__.py                 对外接口：register_capture、register_clouds 等
│  │  ├─ registration_workflow.py    输入检查、人工初值、ICP 和结果输出服务
│  │  ├─ algorithms.py               点云加载、预处理、人工初值与两阶段 ICP
│  │  ├─ automatic.py                无标志点的 FPFH、RANSAC、Point-to-Plane ICP
│  │  ├─ main.py, __main__.py         双点云自动配准、保存和可视化入口
│  │  ├─ registration_math.py        Kabsch 刚体估计、异常配对剔除、点坐标变换
│  │  └─ landmark_*.py               标志点配对窗口及选择状态
│  └─ workflow.py                     感知总接口：口扫+RGB-D→世界牙龈线
├─ planning/
│  ├─ tooth_frames.py                 牙齿点坐标系、口扫/相机/世界转换、导出与预览
│  ├─ trajectory.py                   六维 TCP 位姿与关节轨迹、阶段时序
│  ├─ ik.py                           六关节逆运动学
│  └─ validation.py                   碰撞配置与关节扫掠预检
├─ control/
│  └─ position.py                     MuJoCo 位置执行器指令与运行时检查
├─ pipeline/
│  ├─ run.py                          提取→采集→配准→规划→仿真执行入口
│  ├─ tracking.py                     跟踪运行、可视化、记录和报告
│  └─ simulation.py                   场景初始化与轨迹绘制
├─ models/, assets/, config/          场景、口扫模型及配置
├─ outputs/                           各阶段结果与报告
└─ tests/                             模块与集成测试
```

旧的手动/正弦演示、CrossTooth 离线流水线、兼容脚本及其旧测试已从活动目录移出。原 `C:\Users\Administrator\Desktop\视觉+力觉\stl-ply-obj\outputs\extract_gingival_line.py` 保持不变，算法副本已整理成可调用的感知函数。

## 模块输入与输出

| 模块 | 输入 | 输出 |
| --- | --- | --- |
| Perception | 带顶点颜色、以毫米计的口扫 PLY；全局相机 RGB-D；人工方法另需配对标志点 | 口扫坐标下 CSV 点/OBJ 连线；配准变换；世界坐标、米单位的有序连续段 |
| Planning | 选定的世界牙龈线连续段、当前 MuJoCo 机器人状态 | `[x_m,y_m,z_m,roll,pitch,yaw]` 位姿点、关节解、时序和预检结果 |
| Control | 规划轨迹及对应关节解、当前机器人状态 | MuJoCo 位置执行器 `data.ctrl` 指令、跟踪状态 |
| Pipeline | 颌别、口扫 PLY、场景配置、路径与速度参数 | 分阶段输出、跟踪结果和 `pipeline_report.json` |

位姿后三维为弧度，旋转顺序为 `Rz(yaw)·Ry(pitch)·Rx(roll)`。断开的牙龈线保留为不同连续段；默认选择长度排序后的路径 0，不跨缺口规划。提取依赖 `scikit-learn`，不依赖 CrossTooth 缓存。人工配准需至少六组配对标志点；自动配准使用 FPFH/RANSAC 初始化，无需选点。两种方法均不使用仿真真值作为配准算法输入。新采集与执行共用同一 MuJoCo 模型和状态。

## 已实现与待实现

| 功能 | 状态 |
| --- | --- |
| 彩色口扫 PLY 牙龈线提取与 CSV/OBJ/预览输出 | 已实现；颜色启发式结果需人工检查 |
| 外侧牙龈线自动筛选，排除舌侧/腭侧 | 已实现，默认启用；`--surface all` 提取内外侧 |
| 离群颜色区域及扫描边缘伪线清理 | 已实现，默认启用；`--no-cleanup` 关闭 |
| 单独运行 `extract.py` 时显示牙模与牙龈线三维叠加窗口 | 已实现；`--no-view` 可关闭，pipeline 调用不弹窗 |
| 全局相机 RGB-D 采集、点云生成 | 已实现 |
| 人工标志点初值、ICP 配准、世界坐标转换 | 已实现 |
| 无人工选点的 FPFH + RANSAC + Point-to-Plane ICP | 已实现；输出仍为口扫模型 → 相机，`--registration-method automatic` 启用 |
| 连续段排序、逐点牙齿姿态对齐、六维位姿轨迹 | 已实现；默认 45° 跳变筛选，保留点对齐、跳过姿态插值，全部空间点保留 |
| 牙龈点的右手牙齿坐标系及三坐标系转换 | 已实现于 `planning/tooth_frames.py`；+X 牙冠→牙龈、+Y 侧向切向、+Z 外向法线 |
| 六关节 IK、关节范围、扫掠碰撞预检 | 已实现；`geometry` 默认不验证牙模碰撞 |
| MuJoCo 位置执行器跟踪及误差报告 | 已实现，仅仿真 |
| 一次调用串联感知、规划、执行 | 已实现；人工方法需点选，自动方法无需交互 |
| 根据牙面几何逐点计算洁治器角度 | 已实现三轴直接对齐和 45° 跳变筛选；额外工作倾角待实现，筛选后完成当前下颌路径运动 |
| 导纳、力矩或接触力闭环控制 | 待实现 |
| 真实机器人硬件执行 | 待实现 |

运行失败会停止后续阶段，在 `pipeline_report.json` 记录阶段和原因。跟踪阶段另存 `target_poses_world_m_rad.csv`、`joint_waypoints_rad.csv`、`tcp_tracking.csv` 和 `tracking_report.json`。默认 `geometry` 模式会在内存中关闭牙模凸包碰撞；报告中的成功只说明当前几何演示与其余已启用碰撞检查通过。

## 验证

牙齿坐标系的运行方式、默认颌别方向、矩阵方向与单位见 [planning 开发说明](../planning/README.md)。独立导出保留原始牙龈点编号；规划在路径反向和重采样后重新计算各点牙齿坐标系，并将其作为初始 TCP 姿态。默认跳过相对上一个保留点超过 45° 的姿态约束，沿全部原空间路径插值姿态；加入辅助 IK 样本后完成整段运动。目标和筛选记录在 IK 前导出，不可达或碰撞时停止。关闭筛选的严格模式仍在当前下颌路径索引 4 求解失败。

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

模块测试包含 `surface='all', cleanup=False` 下上下颌提取结果与既有资产逐点逐边比对、默认外侧输出与拓扑一致性、扫描边缘长条和颜色斑点剔除、真实牙齿靠近边缘时的保留、封闭网格、已知内外侧的合成牙弓筛选、旋转/平移/面方向反转、跨侧连线拒绝、坐标及六维姿态、失败时不执行控制，以及使用仿真标志点**仅作为测试夹具**的全流程无窗口验证。自动配准另验证未知大角度刚体变换、部分重叠、离群点、变换方向、PLY/PCD 独立入口、无选点的完整流水线和配准失败时停止执行；仿真真值仅用于运行后的独立误差评价。

外侧筛选在原有颜色等值线提取之后、焊接与短段过滤之前运行：使用牙齿颜色区域 PCA 估计咬合平面，在该平面拟合牙弓中心，并估计局部径向范围。候选线段的两端和中点均需位于牙弓内侧边界之外，且表面法线向外。没有新增点，也不连接被剔除的线段。筛选参数及统计写入 `gingival_surface_filter.json`；独立运行和 pipeline 的 CSV、OBJ、PLY、预览均使用同一份外侧结果。算法适用于完整或较完整牙弓，仍可能保留原颜色算法产生的外侧误检线。

颜色区域清理先于等值线与外侧筛选：根据网格邻接划分牙齿/牙龈颜色连通区域，用表面积和到开放扫描边界的网格测地距离识别无牙齿支持的伪区域。它能移除密集但位于扫描切边的整条误检曲线，默认不直接裁剪所有靠近边缘的点。清理报告存于 `gingival_cleanup.json`，牙齿/牙龈标签 PLY 使用清理后的标签；原始彩色牙模保持原样。`--surface all --no-cleanup` 可复现最初的完整颜色等值线。部分缺损和异常颜色仍可能导致误删或漏删。
