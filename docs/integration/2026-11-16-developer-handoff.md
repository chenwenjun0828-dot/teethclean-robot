# 几何规划、力控与真机接入交接说明

更新日期：2026-11-16。面向接手当前几何轨迹并开发力控的开发者。
本文依据当前源码、配置与本地实际产物整理；本次仅补充文档，不修改核心行为或执行门槛。

## 1. 当前交接状态

| 能力 | 当前状态 |
| --- | --- |
| 真实 session 加载、双颌摆放、单区域姿态/IK/几何轨迹 | 已实现；当前工作基线是下颌 `outer_front` |
| `35/-20/15°`、离面 `0.2 mm` 的直线往返轨迹 | `geometry_only_verified`；几何可达、实际插值和速率检查通过 |
| 该轨迹的完整碰撞、动力学、牙面恒力验收 | 未通过完整执行验收，`execution_ready=false` |
| MuJoCo 恒力实验 | 有独立入口；要求已验证 full 源，不能直接执行本次 geometry_only 源 |
| ROS 1 导出、只读核对、受保护位置回放代码 | 已实现；离线/Mock 可验证，Linux ROS 和实机尚未验收 |
| ROS 真机恒力控制 | 尚未实现；现有桥接是位置回放及力超限停止 |
| 当前 session 真机执行 | 禁止：`robot_execution_allowed=false`、`simulation_only=true`，相机解释未实测 |

**当前轨迹是恒力开发的名义参考，不是可以直接启动的真机洁治任务。**
`motion_mode=contact` 只选择规划接触策略，不建立力闭环；正离面 `0.2 mm` 不保证接触。
历史 full 尝试曾出现局部牙面支持不能确认的问题，转换 JSON 不能消除原验证失败。

## 2. 当前使用的数据

### 2.1 当前工作入口：真实采集 session

入口：[`assets/perception/sessions/20261113_220209/`](../../assets/perception/sessions/20261113_220209/README.md)。
来自 2026-11-13 同一次 D405 RGB-D 采集，序列号 `335122273144`；
下颌六区共 2711 点，上颌六区候选共 1984 点，各区域独立，不跨区连接。

| 文件/目录 | 用途与约定 |
| --- | --- |
| `planning_input_manifest.json` | 相对路径、文件 SHA256、来源与执行限制，数据追溯入口 |
| `robot_joint_observation.json`、`fk_report.json` | 拍摄关节与 UR7e 标称 FK；没有工厂运动学修正，也没有独立关节测量时间戳 |
| `transforms/` | 拍摄位姿、手眼和相机帧组合变换，平移 m |
| `<jaw>/regions/<region>/perception_region.json` | `PreparedRegion` 清单，局部曲线、方向、来源及 scan-to-base 关系 |
| 同区 `points_controller_base_nominal_m.csv` | 发布版 controller_base 中 N×3 位置，m；不再乘 scan-to-base |
| 同区 `region_controller_base_nominal.npz` | 发布版 `points_m`、单位 `normals/tangents` 等 |
| `<jaw>/shared/region_mesh.npz` | 同颌共享扫描网格，由区域清单引用 |
| [`assets/calibration/ur7e_d405/20261113/`](../../assets/calibration/ur7e_d405/20261113/README.md) | `_25` 手眼、D405 内部外参与来源；不包含工具 TCP 标定 |

区域名称：`outer_left / outer_front / outer_right / inner_right / inner_front / inner_left`。
上颌边界尚未批准，当前配套分割/下游输入不足；显示双颌不代表上颌可完整规划。
本次交接仅覆盖下颌前区；清单的 `region_boundaries_approved=false` 也应保留。
关节记录以拍摄后机器人未移动为时间对齐条件，执行前需独立核实。

原始 RGB-D、ROI、选点/配准实验保存在原机器 `outputs/perception/real/`，未随 Git 发布；
加载正式 session 包不依赖这些实验，重新处理原始采集时需要另外交接。
必须保留整个仓库的匹配 `assets/`，单独复制一个区域目录不足以通过来源校验。

### 2.2 另一条链：原全仿真感知 pipeline

这条链用于仿真感知开发和旧教程，**不会自动读取上述真实 session**。

| 输入 | 路径 |
| --- | --- |
| MuJoCo 场景 | `assets/simulation/scene_ur7e_dental.xml` |
| 机器人、器械、力传感器、牙颌网格 | `assets/simulation/robot/` 及场景相对引用资产 |
| 彩色下颌扫描 | `assets/source_cad/teeth/mandibular_scan.ply` |
| 已附 CrossTooth 预测 | `assets/perception/tooth_segmentation/crosstooth_mandibular/` |
| 有序六区域源线 | `assets/regional_paths/mandibular/` |
| 仿真相机/配准 | `config/global_camera_registration.json` |
| 流程参数 | `config/pipeline_outer_front.json` |

流程为仿真采集 -> 配准 -> `PreparedRegion` -> 规划 -> 位置动力学。
旧教程是 `simple`、`6.37/13.26/-0.4°`、离面 `1.8 mm`、张口 `20°`；
本次真实 session 是 `continuous`、`35/-20/15°`、离面 `0.2 mm`，不能混用验收结论。
README 的旧图片和历史验收不表示当前工具安装和当前 session 又通过了一次验收。

## 3. 新机器复现当前轨迹

Windows x64、Python 3.11.9 是已记录的安装环境；TOPPRA 需要 C++ Build Tools。
在仓库根目录 PowerShell 中运行；完整环境说明见[根 README](../../README.md)。

```powershell
git clone --branch cwj https://github.com/chenwenjun0828-dot/teethclean-robot.git
Set-Location teethclean-robot
py -3.11 -m venv .venv
$py = ".\.venv\Scripts\python.exe"
& $py -m pip install --upgrade pip
& $py -m pip install -r requirements.txt -c requirements-perception-lock.txt
& $py -m pip check
```

常规 `outputs/`、虚拟环境和缓存继续忽略；依开发者要求，本次定向发布以下两个完整输出目录。
克隆 `cwj` 后即可取得它们，不需要从原机器另外复制，也不需要先运行规划。

| 已发布目录 | 内容 |
| --- | --- |
| [`outputs/developer_handoff_20261116/session_plan/`](../../outputs/developer_handoff_20261116/session_plan/) | 本轮重建并与原基线逐元素核对的关节轨迹、TCP/姿态数组、几何参考、规划报告和采样审计 |
| [`outputs/developer_handoff_20261116/ros1_export/`](../../outputs/developer_handoff_20261116/ros1_export/) | 对同一轨迹导出的完整可搬运 ROS JSON bundle，带来源限制及插值审计 |

两目录合计约 28.2 MB。它们是固定交接快照；不要原地修改或覆盖。
`.gitattributes` 对这两个目录禁用换行转换以保持来源哈希；`.gitignore` 仍忽略其他运行输出。
快照内含原开发机器的来源绝对路径，仅用于追溯，不是新机器默认配置。

直接读取已发布轨迹（只做离线分析，不连接 ROS，不要求源机器路径存在）：

```powershell
@'
from planning.infrastructure.trajectory import Trajectory
from control.hardware.trajectory_contract import load_bundle

plan = Trajectory.load("outputs/developer_handoff_20261116/session_plan/trajectory.npz")
state = plan.sample(60.0)
print(plan.duration, plan.joint_names, state["phase"], state["q"])
curve, report, manifest = load_bundle(
    "outputs/developer_handoff_20261116/ros1_export/trajectory.json")
q, dq, ddq, tcp, phase = curve.sample(60.0)
print("ROS nominal sample:", phase, q, "TCP:", tcp)
print("hardware_execution_ready:", report["hardware_execution_ready"])
'@ | & $py -
& $py -m control.hardware inspect `
  --trajectory outputs/developer_handoff_20261116/ros1_export/trajectory.json
```

上述直接读取不构成执行校验；inspect 应显示 blocked 并非零退出。
需要恢复 MuJoCo 场景或开发力控时，推荐按下面命令在自己的路径重建，自动生成有效的本机来源路径。
所有生成命令使用新的空目录，重复实验时更换目录名，不需要重新采集或配准。

### 3.1 当前几何轨迹的完整命令

原机器基线：`outputs/planning/session_straight_return_35_m20_15_20261116/`。

```powershell
$plan = "outputs/my_handoff_reproduction/session_plan"
& $py -m planning `
  --perception-session assets/perception/sessions/20261113_220209 `
  --session-camera-frame color_optical --jaw mandibular --region outer_front `
  --planning-mode continuous --pose-angles-deg 35 -20 15 `
  --motion-mode contact --standoff-mm 0.2 `
  --working-segment-radius-mm 2 --contact-tolerance-mm 0.5 `
  --nonworking-clearance-mm 0 --pose-spacing-mm 0.05 --sweep-trim-mm 1 `
  --speed-mm-s 1 --transfer-speed-mm-s 150 `
  --check-mode geometry_only --orientation-audit strict `
  --headless --out $plan
# 只回放，不执行控制或连接机器人；关闭窗口后返回。
& $py -m planning.view --plan $plan --speed 1
```

半径/容差只是忠实记录当前实验输入；geometry_only 不调用碰撞/接触检查，
不表示这些参数适合真机。不要加 `--execute`、覆盖 `--jaw-opening-deg`，
或混入 `--perception-result`/原始扫描参数。
Session 恢复实测双颌位姿和拍摄关节；报告的张口标量 `0` 不表示把双颌重新合拢。

基线：2084 个关节路点、1853 个沿线位姿点，总时长 `124.879636567 s`；
进场 `55.324405916 s`、沿线 `61.117710855 s`、返回 `8.437519795 s`，
源线保留率约 `96.3361%`。状态应为 `geometry_only_verified`，`execution_ready=false`。
进退场策略 `straight_tcp_home_return_v1`：拍摄 home -> 沿线起点 -> 连续沿线 -> 原 home；
进退场 TCP 为 world 中直线，方向为最短 Slerp，最终六关节与初始完全相同。
最终插值的检查是有限采样，不是连续解析直线或碰撞证明。

### 3.2 交互选角和旧仿真入口

```powershell
# 本地服务打印 URL；Ctrl+C 停止。
& $py -m planning.visualization.pose_angles `
  --perception-session assets/perception/sessions/20261113_220209 `
  --session-camera-frame color_optical --jaw mandibular --region outer_front `
  --model-scope both
# 另一条全仿真 pipeline。
& $py -m pipeline.run --params config/pipeline_outer_front.json --dry-run
& $py -m pipeline.run --params config/pipeline_outer_front.json `
  --headless --output outputs/my_simulation_front
```

页面目前只提交非接触规划；复现本次 contact 参数应用 3.1 CLI。
`config/planning_session_optical_preview.json` 使用 `geometry_only=true`，仅导出几何，
不求姿态、IK 或时序；与 **`--check-mode geometry_only`** 不同，不能生成本次关节轨迹。

## 4. 力控开发要读取什么

### 4.1 交接整个规划目录

| 产物 | 用途 |
| --- | --- |
| `request.json`、`planning_report.json` | 参数、状态、世界变换、场景、session 限制、来源指纹 |
| `trajectory.npz` | 关节时序、阶段、牙位、自由段多项式、沿线样条及时间律 |
| `sweep_reference.json`、`perception_reference.json` | 已保存的连续几何和姿态参考，不在控制侧重新拟合 |
| `transfer_reference.json` | 直线进退场、初末关节与端点参考，受指纹约束 |
| `geometry.json/.npz`、`tooth_planes.json` | 局部扫描几何、逐牙平面与法向 |
| `pose_targets_world_m.npy`、`pose_joints_rad.npy`、`pose_parameters_m.npy` | 沿线离散目标、IK 节点及参考参数，不替代实际定时插值 |
| full 源额外的审计/动力学产物 | 经 `load_verified` 校验；本次几何源没有 full 执行许可 |

使用 `Trajectory.load()` / `sample(t)`；不能只线性插值关节数组。
沿线是固定关节样条叠加 TOPPRA 时间律，自由段是 C2 多项式。
`sample()` 返回 `q/dq/ddq/phase/tooth/interval`；阶段是 `transfer/trace/retreat`，
`phases/teeth` 长度为路点数减一。顺序读取 `joint_names`，不可静默按 `2*pi` 归一化。

下例在仓库根目录离线读取名义轨迹、TCP 和法向，不启动控制：

```powershell
@'
from pathlib import Path
import json
import numpy as np
from planning.infrastructure.trajectory import Trajectory
from planning.session import rebuild_session_simulation
from planning.infrastructure.robot import Robot
from planning.motion_planning.contract import load_sweep_reference
from control.force.continuous_path import RegionalContactPath
from control.force.compliance import ForceControlConfig
from control.force.contact_path import ContactPathConfig

root = Path("outputs/my_handoff_reproduction/session_plan")
report = json.loads((root / "planning_report.json").read_text(encoding="utf-8"))
request = json.loads((root / "request.json").read_text(encoding="utf-8"))
plan = Trajectory.load(root / "trajectory.npz")
_, model, data, _, _, _ = rebuild_session_simulation(
    request["perception_session"],
    robot_mount_position_m=report["robot_mount_position_m"])
robot = Robot(model, data)
reference = load_sweep_reference(root, report)
world = np.asarray(report["world_from_scan_m"])
block = report["motion_segments"][0]
a, b = block["trace_start_waypoint"], block["trace_end_waypoint"]
t = (plan.times[a] + plan.times[b]) / 2
state = plan.sample(t)
tcp_world = robot.fk(state["q"])
# Reuse the saved clock-to-parameter mapping; do not run a control loop.
path = RegionalContactPath(root, plan, report, world,
                           ForceControlConfig(), ContactPathConfig())
u = path.parameter_at(t)
normal_world = world[:3, :3] @ reference.normal(u)
print("duration_s:", plan.duration, "joint_names:", plan.joint_names)
print("phase:", state["phase"], "q_rad:", state["q"])
print("tcp_world_m:\n", tcp_world, "normal_world:", normal_world)
print("execution_ready:", report["execution_ready"])
'@ | & $py -
```

这是结构读取和 session 重建，**不替代 `load_verified`，不可接到伺服发送循环**。
规划报告目前含原机器绝对来源路径，跨机器复制原规划目录可能校验失败；
优先在新 checkout 按 3.1 重建，不能替换哈希或删除校验。
Linux 消费的是第 5 节整体传输的 ROS bundle，不是裸 NPZ。

### 4.2 坐标、单位与 TCP

| 数据 | 约定 |
| --- | --- |
| 原始扫描/点云 | mm；连续曲线 API 的位置以 m 表达在扫描局部系 |
| world/base TCP 和变换平移 | m；`T_A_B` 表示把 B 坐标变换到 A |
| 关节、速度、加速度 | rad、rad/s、rad/s²；CLI 三角度使用度 |
| 时间、力、力矩 | s、N、N*m |
| 法向 | 只乘旋转，不乘平移；必须验证法向符号 |
| TCP | `tcp_frame_id=shaft_axial_x_radial_y_v1`，不能直接用未经核对的 ROS `tool0` |

发布 CSV/NPZ 对应归档 controller_base 链。当前 CLI 默认 `color_optical`，
由 `perception.session` 推导有效链，不能混入发布版 CSV；`declared` 才是归档帧解释。
`color_optical` 仍标记 `optical_frame_hypothesis_live_tf_unverified`，需现场 TF 与多位姿 FK 核实。
规划使用 `T_world_controller_base = T_world_ur_base @ Rz(pi)`，
`T_world_scan = T_world_controller_base @ T_controller_base_scan`。
复用 `load_session_region()`、`build_session_simulation()` / `rebuild_session_simulation()`，
不手动重复旋转或把 controller_base 当 world。
工具采用当前 `ur7e_with_tool.xml` 安装；旧安装轨迹必须重算。
三个角度是项目的 `fixed_z_angle/fixed_y_azimuth/tool_x_flip`，不是任意 RPY 或直接牙面夹角。

### 4.3 现有力控代码及接入缺口

| 职责 | 代码 |
| --- | --- |
| CLI | [`control/force/runner.py`](../../control/force/runner.py) |
| MuJoCo 主循环、日志 | [`force_trajectory.py`](../../control/force/force_trajectory.py) |
| 单次贴合/连续沿线/卸力、时钟映射 | [`continuous_path.py`](../../control/force/continuous_path.py)、[`contact_path.py`](../../control/force/contact_path.py) |
| 法向导纳与仿真六轴补偿 | [`compliance.py`](../../control/force/compliance.py)：`NormalAdmittance`、`ForceControlConfig`、`SensorContactForce` |
| 名义姿态/切向位置保持、法向修正 | [`force_servo.py`](../../control/force/force_servo.py)：`CartesianServo`，依赖 MuJoCo |
| 预测保护和独立接触复查 | [`force_guard.py`](../../control/force/force_guard.py)、[`validation.py`](../../control/force/validation.py) |
| 硬件抽象/预留逐点后端 | [`protocol.py`](../../control/hardware/protocol.py)、[`ur_robot.py`](../../control/hardware/ur_robot.py) |

现有力控调节**补偿外力的模长**，不默认调节法向投影；日志另存法向分量。
源轨迹保留为名义参考，反馈可以减慢源时间并叠加有界法向位移；
实际 q/TCP 会改变，不能同时宣称恒力和实际路径完全等于源位置回放。

两个接入缺口必须先处理：

1. `force_trajectory._run()` 先调用 `load_verified`，本次几何源会被拒绝。
   即使后续取得 full 源，它也丢弃加载器返回的场景，重新 `create_simulation()`，
   没有恢复 session 双颌实测变换，因此不能认为传入 session 的 `--source` 就已兼容。
2. 该循环直接读 MuJoCo 传感器、设置 actuator 并 `mj_step`；
   `SensorContactForce` 使用同状态无接触逆动力学补偿，不等于真机零偏/重力补偿。
   `HardwareBackend` 仍是接口预留，没有真机恒力切换 CLI。

下一位开发者应按 [AGENTS.md](../../AGENTS.md) 先写具体接入方案并审核，
明确 session 场景复用、力定义/坐标/时间戳、贴合/卸力、法向位移边界、闭环频率、
修正轨迹的可达/碰撞/速率检查、停止机制与命令唯一所有者。
策略归 `control/force/`，传感器/通信归 `control/hardware/`；不在可视化中重建感知曲线或实现控制。

旧 MuJoCo 实验只能用于**匹配当前场景且通过 full 加载校验**的独立源：

```powershell
# $verifiedPlan 另行指定；不能用本次 $plan，也不假定 session full 源已兼容。
& $py -m control.force --source $verifiedPlan --out outputs/my_force_experiment `
  --target-force-n 1 --contact-schedule regional --trace-speed-mm-s 1 --max-duration-s 180
# 独立球-平面控制组件测试，不读取牙面轨迹。
& $py -m control.force.benchmark --out outputs/my_force_benchmark --target-force-n 1
```

`1 N` 和时限是仿真示例。验收应看 `force_tracking_passed`、
`scanned_surface_contact_passed`、`force_control_passed` 和全路径进度，不能只看平均力。
历史牙面实验因预测力保护停止，球-平面 benchmark 通过不能替代牙面恒力验收。

## 5. 已有 ROS 1 接入流程

详见 [`control/hardware/README.md`](../../control/hardware/README.md)。

| 文件 | 职责 |
| --- | --- |
| `ros_export.py` | 规划环境离线导出、关节映射、源/ROS 插值与 full 曲线审计 |
| `trajectory_contract.py` | JSON 契约、五次插值、bundle 哈希、源执行限制 |
| `ros_runner.py` | 部署门槛、状态/力监控、action 生命周期、停止和日志 |
| `ros1_trajectory.py` | ROS topic/action/service/TF 适配 |
| `__main__.py` | `export / inspect / check / execute` 入口 |

### 5.1 Windows 离线导出

```powershell
# 沿用 3.1 重建的 $plan，不连接 ROS。
$bundle = "outputs/my_handoff_reproduction/ros1_export"
& $py -m control.hardware export --plan $plan --out $bundle
& $py -m control.hardware inspect --trajectory "$bundle/trajectory.json"
```

当前源 export 可以成功，inspect **应显示 blocked 且非零退出**、`motion_sent=false`。
原机器最终包：`outputs/hardware/ur7e_ros1_20261116/export_final/`，
16006 点、`124.879636567 s`。整体传输导出目录，包含 `trajectory.json`、
`export_report.json`、`source_manifest.json`、请求/源报告/参考及 ROS 审计，不能只复制 JSON。
本次已将同参数重建的完整包发布在 `outputs/developer_handoff_20261116/ros1_export/`，
可以直接下载/离线 inspect；以上命令用于重建另一份包，原历史输出目录不上传。

源 `ur_*_joint` 显式映射为 ROS 无前缀六关节名；不改符号、偏置或归一化，
目标顺序必须与部署一致。q/dq/ddq 和时间共同定义五次插值，不可只取 positions 重插值。
`--slowdown k` 仅允许 `k>=1` 并重审计，不放宽保护。
不要运行旧 `smooth_path_executor_1111_json2real.py`，其额外运动与本包执行契约不同。

### 5.2 Linux 部署和只读核对

需要 NumPy、ROS 1 与现场 UR7e 驱动，以及 `rospy/actionlib/tf2_ros`、
`control_msgs/trajectory_msgs/controller_manager_msgs/ur_dashboard_msgs/std_srvs`。
消费 bundle 不需要 MuJoCo/Open3D/TOPPRA。从可导入本项目 `control` 的 checkout 根目录运行，
先 source 现场 ROS/Catkin。UR7e 实际 roslaunch/IP/标定未知，不照搬旧 UR3e 配置。

从 [`ur7e_deployment.template.json`](../../control/hardware/ur7e_deployment.template.json) 复制独立现场配置。
必填：设备/工厂标定、TCP/安装/负载、多位姿 FK/配准、力单位/补偿、
控制器/action/topics/services/TF、阈值、停止试验、命令互斥和独立看门狗证据。
控制器须为已核实的 `position_controllers/ScaledJointTrajectoryController`、五次插值；
核实 `controller_state.actual.time_from_start` 来自当前轨迹缩放时钟，不能用墙钟替代。
`planning_world_from_base_m` 是 ROS base -> 规划 world；`tip_frame` 为已标定器械 TCP。
真实力用有有效时间戳的 `WrenchStamped`，单位 N/N*m；不猜测旧坤维读数 Kg/N 换算。

```bash
python3 -m control.hardware inspect --trajectory /data/export/trajectory.json --deployment /data/deployment.json
python3 -m control.hardware check --trajectory /data/export/trajectory.json --deployment /data/deployment.json --out /data/check_001
```

`inspect` 离线；`check` 只读关节、力、TF、控制器和机器人状态，不发 goal/心跳/stop，
不启动 External Control、解除保护或自动到起点。部署字段完整后才能完成只读核对；
它不会解除源限制，也不是恒力测试。

### 5.3 后续具备条件时的位置回放

当前没有把此 session 合法提升为硬件就绪的流程或开关。
需先用现场证据并经单独审核解决帧解释、TCP/机器人/牙模一致性与来源限制，
重做 full 碰撞/动力学和 ROS 曲线审计，绑定 bundle 三个哈希并填写有失效时间的授权。
不能手改 `execution_ready/simulation_only/robot_execution_allowed/camera_verified` 绕过门槛。

以下仅记录未来的显式入口，**当前源会在创建 ROS 后端前被阻止**：

```bash
python3 -m control.hardware execute --trajectory /data/export/trajectory.json --deployment /data/deployment.json --out /data/execute_001
```

它发送一次完整 FollowJointTrajectory，不自动到起点、不重新规划或切控制器。
只允许一个命令所有者，不能同时发送位置 action 和导纳/MoveIt Servo 修正。
故障取消 goal、请求已验证停止服务并确认实际速度，不自动重试或返回 home。
提交进程退出后 action 可能继续，接触运行必须实测独立看门狗与停止路径；软件 stop 不是硬件急停。
完成要求真实进度、最终关节/TCP 和持续停止反馈一致，不能只看 action 成功。
后续真机恒力会改变实际指令曲线，需另行审核闭环命令通道与保护。

## 6. 本次验证及下一步

本次使用项目 `.venv` 检查，命令如下：

```powershell
& $py -m unittest discover -s tests -p 'test_*.py' -q
& $py -m unittest discover -s planning -t . -p 'test_*.py' -q
& $py -m pipeline.run --params config/pipeline_outer_front.json --dry-run
git diff --check
```

- 规划回归：142 项通过。
- 跨模块回归：345 项通过；与规划回归合计 487 项。
- 按 3.1 在 `outputs/developer_handoff_20261116/session_plan/` 实际重建通过：
  时间、关节、阶段、牙位、多项式系数及三个沿线目标数组与原基线逐元素相同，首末关节完全相同。
- 4.1 读取示例直接从本文提取执行通过，能读取名义 TCP、法向和 `execution_ready=false`。
- 3.1 同参数 session dry-run 与原 pipeline dry-run 通过；仅为输入检查。
- 原最终 ROS 包 inspect：blocked、16006 点、`motion_sent=false`，符合限制。
- 本次交接快照的 ROS 导出同样为 16006 点、124.879636567 s，插值审计通过；
  新包 inspect 仍正确阻止执行。快照保留全部来源限制，不因上传而成为硬件可执行数据。
- 57 个快照文件均核对工作区与 Git 暂存字节一致，来源哈希未受换行转换影响。
- 从 Git 暂存树 `git archive` 导出仓库外干净副本，直接读取已发布 NPZ/ROS bundle、
  离线 inspect、按 3.1 在 `outputs/my_handoff_reproduction/session_plan/` 重建及 4.1 示例均通过。
  新轨迹的时间、关节、阶段、牙位、插值系数与发布快照逐元素一致。
  此检查复用本机 `.venv` 依赖，不是重新安装环境或 Linux/ROS 验收。
- `git diff --check`、`git diff --cached --check` 通过；未新增依赖或一次性源码。
- 本次不连接机器人；Linux ROS、真实力补偿、独立看门狗/停止距离与牙面恒力未验收。

接手顺序：复现几何轨迹 -> 理解参考/坐标/时序 -> 提交 session 力控方案审核 ->
仿真接触与故障验证 -> 现场只读核对 -> 独立停止验收 -> 审核具体真机实验。
本文不构成后续核心功能修改或实机运动批准。
