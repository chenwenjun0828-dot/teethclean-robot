# Hardware Interface

Controllers depend on `HardwareBackend`, which exposes lifecycle, state and
wrench reads, joint commands, and an emergency stop. `MockHardwareBackend`
supports deterministic tests; `MujocoHardwareBackend` adapts a simulation;
`URRobotBackend` reserves the real-robot transport boundary and refuses to
connect until an explicit transport object is supplied.

## UR7e ROS 1 接入

`ros_export.py` 负责离线导出，`trajectory_contract.py` 定义 JSON/五次插值与哈希，
`ros_runner.py` 负责门槛/监控/停止，`ros1_trajectory.py` 适配 ROS。
模式为位置轨迹回放与力超限停止，不是恒力控制；正离面 0.2 mm 不保证接触。
批准方案见 `docs/superpowers/specs/2026-11-16-ur7e-ros1-trajectory-bridge-design.md`。

### 离线导出

在仓库根目录运行，输出必须为新目录：

```powershell
$py = ".\.venv\Scripts\python.exe"
& $py -m control.hardware export `
  --plan outputs/planning/session_straight_return_35_m20_15_20261116 `
  --out outputs/hardware/my_ur7e_export
& $py -m control.hardware inspect --trajectory outputs/hardware/my_ur7e_export/trajectory.json
```

两个命令不连接 ROS；`inspect` 有阻断项时退出码为 2。导出成功不代表可执行。
默认显式映射六个 `ur_*_joint` 到参考工程六个无前缀关节名。
`--mapping <json>` 可指定 `mapping`（源到目标字典）与有序 `joint_names`，必须一对一覆盖六关节。
不改变符号、偏置或做 2*pi 归一化。`--slowdown k` 仅接受 k>=1，缩放时间和导数后重新审计。

导出合入原路点/阶段/时间律边界、间隔<=0.01 s、携带 q/dq/ddq。
时间量化为 ns，相距不足 1 us 的时间律边界合并并优先保留原路点，避免纳秒级小段。
检查每段 1/4、1/2、3/4 的 FK/TCP 参考，以 Bernstein 界检查关节/速率，
调用原独立沿线及直线进退场审计核对 ROS 曲线。
full 源须通过 `load_verified`，转换后的曲线另做完整运动/碰撞审计。
这是有明确采样范围的检查，不是连续碰撞的解析证明。

必须整体传输导出目录：trajectory、export_report、source_manifest、源报告/请求/参考快照及 ROS 审计。
不要交给旧 `json2real`，它忽略执行限制且额外运动。

### ROS 部署

Linux 端需要 NumPy、ROS 1 UR 驱动和 `rospy/actionlib/tf2_ros`、
`control_msgs/trajectory_msgs/controller_manager_msgs/ur_dashboard_msgs/std_srvs`。
消费 JSON 不需要 MuJoCo、TOPPRA、Open3D。从能导入 `control` 的目录运行，先 source 现场 ROS/Catkin。
本机未运行 Linux ROS 集成测试，消息/版本需现场验收；不能用旧 UR3e launch/标定代替 UR7e。

以 `ur7e_deployment.template.json` 为基础创建独立 `deployment.json`。模板按要求留空，不填推测值。

- `robot_model=ur7e`、`joint_names` 与导出顺序一致、`controller_interpolation=quintic`、
  `command_owner=exclusive_follow_joint_trajectory`。
- `identity` 填设备、工具 TCP/安装、负载、配准/live TF、多姿态 FK、力单位/补偿、
  停止测试、命令互斥与独立看门狗的现场证据定位信息。
- `planning_world_from_base_m` 为 ROS base_frame 到规划世界的 4x4 变换，平移 m。
  `tip_frame` 必须是已标定器械 TCP，不默认使用 tool0。
- `ros` 填实际 action、控制器、topic、Dashboard 查询和停止服务。仅支持已确认的
  `position_controllers/ScaledJointTrajectoryController` 五次插值。
- 力为真实、有效时间戳、已验证补偿的 `WrenchStamped`，N/N*m，帧为 `wrench_frame`。
  旧坤维原始读数不能直接代入，程序不猜测 Kg 到 N 的换算。
- 所有 `limits` 与各 `phase_limits` 必填，按现场审核选择，无通用接触力默认值。
- `authorization` 三个 SHA256 绑定 trajectory、manifest、export_report，填写负责人和带时区失效时间。
  该审核记录不能替代 full、来源限制或独立硬件保护。

```bash
python3 -m control.hardware inspect --trajectory /data/export/trajectory.json --deployment /data/deployment.json
python3 -m control.hardware check --trajectory /data/export/trajectory.json --deployment /data/deployment.json --out /data/check_001
```

`check` 不发送 goal/心跳/stop，不启动 External Control 或解除保护。
读取关节、力、TCP TF、速度缩放、控制器及机器人/安全/程序状态，核对起点，输出 run_report 和 states.jsonl。
Dashboard 使用只读服务轮询，避免把“仅变化时发布”的 topic 当作周期心跳。
无有效消息的零值、旧时间戳、断流或过期 TF 均无效；报告保留源执行限制。

### 显式执行

仅在来源限制解除、full 和现场验收完成后使用：

```bash
python3 -m control.hardware execute --trajectory /data/export/trajectory.json --deployment /data/deployment.json --out /data/execute_001
```

只发送一次完整 FollowJointTrajectory，起点必须已经吻合；不自动移到起点、MoveIt 重规划、切控制器、
故障后返回或重试，禁止其他关节命令控制器争用。
进度取 `controller_state_topic` 的 `actual.time_from_start` 与同帧 desired.positions，
监控速度缩放/停滞，不按墙钟追赶；完成后的持续状态仍须有效，不能仅依赖停止发布的 action feedback。
`controller_progress_evidence` 必须证明此字段来自当前轨迹的缩放时钟。
部分旧 UR scaled 控制器不填轨迹时间，该版本不能直接接触运行；必须先完成现场驱动接口验收。
依据：[ros_control 状态实现](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/joint_trajectory_controller_impl.h)、
[UR scaled 控制器实现](https://github.com/UniversalRobots/Universal_Robots_ROS_scaled_controllers/blob/master/scaled_joint_trajectory_controller/include/scaled_joint_trajectory_controller/scaled_joint_trajectory_controller.h)。
完成需 action 成功、进度完整、最终关节/TCP 到位且实际速度持续低于停止阈值。
故障取消 goal、调用已核实的 `std_srvs/Trigger` 停止服务并确认速度，记录响应时间和可用的 TCP 位移。
软件停止不是硬件急停，真实停止距离须现场测量。

外部独立看门狗须订阅递增 UInt64 心跳，断流后独立停止，并周期发布 Bool 就绪信号。
桥接只在执行监控周期发心跳，进程阻塞/退出/崩溃时不能假定客户端还在保护。
本模块不提供已认证的硬实时看门狗；其力/UR 联锁、心跳与停止响应须现场验收。
填写证据字符串或发布 true 不等于验收，未落实时不得接触运行。

当前源为 geometry_only、相机解释未实测且硬件禁止。允许诊断导出/核对，execute 在构造 ROS 后端前拒绝。
上句指项目原 CLI。以下独立交付使用开发者另行授权的几何回放策略。

### 当前交付：MoveIt 两文件

`control/employment/2026_11_16/` 当前使用已批准的精简 MoveIt 客户端：

- `trajectory.json`：全部 16006 点，顶层仅有 `joint_names/points`；位置 rad、速度 rad/s、加速度 rad/s²、时间 s。
- `ur7e_saved_trajectory_executor.py`：控制逻辑与少量默认选项，不包含 TCP/阶段/审计数组，也没有 `SITE_DEPLOYMENT` 或 `PLAIN_REFERENCE`。

不再需要逐项填写上一版完整部署参数。默认沿用旧 json2real 的 `manipulator / base_link / tool0`，
分别可由 `--group / --base-frame / --tip-frame` 覆盖。机器人 IP 由现场驱动配置，脚本不连接机器人 IP。
脚本只保留一个与 JSON 配对的 SHA256 字符串；修改轨迹后须重新核验并构建。

需要 Linux 上的 Python 3、NumPy、ROS 1、`moveit_commander` 和 ROS 消息包。
先启动**正确的 UR7e 驱动、机器人描述与 MoveIt move_group**，并 source 对应 ROS/Catkin 环境；
仅启动驱动还不够。参考工程主要是 UR3e，不能直接以其 launch 代替 UR7e 配置。
本仓库尚无已核验的现场 UR7e 启动命令，不填写猜测的命令或 IP。

将两个文件放在工作站同一目录，在该目录运行：

```bash
python3 ur7e_saved_trajectory_executor.py inspect --trajectory trajectory.json
python3 ur7e_saved_trajectory_executor.py check --trajectory trajectory.json --out check_001
python3 ur7e_saved_trajectory_executor.py execute --trajectory trajectory.json --keyboard --out run_001
```

`inspect` 完全离线，不加载 MoveIt；`check` 只读核对关节组、起点及静止状态，不发送轨迹或 stop。
机械臂必须预先处于 JSON 首点，偏差超过默认 0.01 rad 会拒绝；脚本不会自动规划到首点，
也不会执行旧程序的硬编码初始位姿或额外返回运动。
默认监听 `/joint_states`，要求六关节位置、速度及有效源时间戳，组内关节必须与 JSON 一致。

执行时看到 `Ready. Press S to start` 后，按 S/s 开始，按空格取消/停止且不续跑。
空格停止会取消当前 MoveIt 执行 goal 并调用 MoveIt stop；还需 action 结束和新的关节状态确认停稳。
默认低速阈值 0.01 rad/s、连续停稳 0.2 s、状态超时 1 s、停止等待 5 s、监测周期 0.02 s。
这些是本次新增的可覆盖检查参数，不冒称为旧程序的现场参数，详见 `execute --help`。
Linux 前台终端须保持输入焦点；无 TTY 或监听故障不会自动开始。未加 `--keyboard` 则预检后立即发送。
停止失败明确报告 `fault_latched`，不会因为 stop 已调用而报告停稳；空格不能代替实体急停。

倍速与上一版一样在发送前修改内存中的 `t/s`、`dq*s`、`ddq*s²`，JSON 不变：

```bash
python3 ur7e_saved_trajectory_executor.py execute --trajectory trajectory.json --speed-factor 2 --keyboard --out run_2x_001
```

大于 1 倍速时从现场 `robot_description` 的 URDF 和 `robot_description_planning/joint_limits`
读取有效上限，并对整段五次插值检查速度/加速度。缺失上限时拒绝加速，可用
`--joint-velocity-limits` 与 `--joint-acceleration-limits` 分别提供六个现场允许值，顺序与 JSON 关节名一致，
单位 rad/s 和 rad/s²；明确提供的值不能放宽已知模型上限。
`inspect --speed-factor 2` 不读取 ROS，未显式提供限值时会报告缺项；`check/execute` 才读取现场模型。
旧程序的 0.1/0.075 是规划倍率，不能据此填写绝对上限，也不再复制其未用到的规划器/重规划选项。
非原速 full 策略仍须重新审计；当前交付是显式批准的 geometry_only 回放。

该精简版本仅有输入/起点、关节状态、执行结果和停止确认，**不包含上一版 TCP 跟踪、分阶段力阈值、
独立看门狗或完整现场授权门槛**。报告始终保留 `hardware_validated=false` 与
`collision_dynamics_verified=false`，不是恒力或接触验收。现场 MoveIt/机器人自身保护仍需正确部署。
正常完成、人工取消、已确认人工停止返回 0；必须看报告状态区分，不能以退出码 0 代替完整执行成功。
输出目录须为新目录，日志写到交付目录外。

本机从原始已核验导出包重建到新目录：

```powershell
& .\.venv\Scripts\python.exe -m control.hardware standalone-pack `
  --trajectory outputs/hardware/ur7e_ros1_20261116/export_final/trajectory.json `
  --backend moveit --format plain --allow-geometry-only --out outputs/ur7e_moveit_build
```

构建时检查原始 bundle/快照和几何回放门槛，运行时验证 JSON 配对哈希及其未被更改，不假称重验原始快照。
生成脚本从权威模块构建，不手工维护另一份实现。
审核方案：`docs/superpowers/specs/2026-11-16-ur7e-minimal-moveit-executor-design.md`。
MoveIt 通道依据：[MoveGroupCommander.execute](https://github.com/moveit/moveit/blob/noetic-devel/moveit_commander/src/moveit_commander/move_group.py)、
[ExecuteTrajectory 与 stop 实现](https://github.com/moveit/moveit/blob/noetic-devel/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp)。

### 保护后端的两文件构建

以下保留原保护客户端的使用方法，适用于缺省 `--backend protected` 的构建，
不适用于当前日期目录的 MoveIt 精简脚本。保护后端同样生成两份文件：

- `ur7e_saved_trajectory_executor.py`：完整普通 Python 源文件，运行时不导入本项目。
- `trajectory.json`：普通缩进 JSON，顶层只有 `joint_names`、`points`；每点只有
  `positions`、`velocities`、`accelerations`、`time_from_start`，单位依次为 rad、rad/s、rad/s²、s。
  与旧 `dict_to_trajectory` 的读取格式一致，完整保留 16006 点，不抽稀、不重新规划。

现场配置集中在脚本开头的 `SITE_DEPLOYMENT`（JSON 文本，保留 `null/true/false` 写法）。
后面的 `PLAIN_REFERENCE` 是固定数据：包含原来源、阶段、逐点 TCP 与审计参考，不能作为现场配置修改。
JSON 不含 Base64、部署参数或报告；仍只需迁移这两个文件。

独立脚本依赖 Python 3、NumPy 和上述 ROS 1 通信/消息包，无需克隆本项目或安装 MoveIt、规划和仿真依赖。
需先启动现场正确的 UR7e 驱动并 source 其 ROS/Catkin 环境。

本机重新构建到**新目录**：

```powershell
& .\.venv\Scripts\python.exe -m control.hardware standalone-pack `
  --trajectory outputs/hardware/ur7e_ros1_20261116/export_final/trajectory.json `
  --deployment control/hardware/ur7e_deployment.template.json `
  --allow-geometry-only --format plain --out outputs/ur7e_standalone_build
```

默认 `--format portable` 仍生成旧 `ur7e_standalone_v1` 两文件包，部署参数位于其 JSON 的 `deployment`，
原文件字节保存在 Base64 `files`；新脚本继续支持读取该旧包。`--format plain` 生成当前明文格式，
必须配合本次生成的脚本使用；原项目 CLI 不直接接收该明文格式。
旧 json2real 虽可读取新的点格式，但还包含自己的 MoveIt 和自动移动逻辑，不建议直接用它执行本轨迹。

没有 `--allow-geometry-only` 时仍使用 full 门槛。显式指定时在独立配置写入
`execution_policy.mode=geometry_only` 和 `acknowledge_unverified_motion=true`，允许原几何轨迹回放，
不再以项目 source/full、execution_ready 或仿真/硬件禁止标记直接拒绝；保留来源和插值校验，
不把原失败/未验证记录改为通过。报告显示原 `source`、`source_warnings` 和
`project_source_restrictions_waived`，`collision_dynamics_verified=false`；这不是完整避碰或动力学验收。
项目原 `python -m control.hardware execute` 继续按 full 规则拒绝当前源。

保护后端构建自动填好型号、关节顺序、插值模式、命令模式与原始三文件哈希。
其余现场未知信息仍为 null。修改脚本 `SITE_DEPLOYMENT` 中的实际接口、base-to-world 变换、
证据记录、阈值及 `authorization.approved_by/expires_utc`。
原始三个哈希作为来源历史保留，不要替换为明文 JSON 哈希。
有效期使用带时区 ISO 8601。证据字段只是记录，不会代替现场标定与测试。
力反馈、独立看门狗和停止服务仍须实际部署；单文件执行客户端并未提供这些独立现场组件。

明文交付改变了指纹校验契约：生成时以 `load_bundle` 核验原始导出包与来源快照；
工作站运行时验证明文 JSON、固定参考和配套脚本，工作站不再持有或重验原始快照。
报告明确记录 `source_validation=original_bundle_verified_at_build_time`，不将生成时检查冒称为运行时检查。
新绑定使用 `authorization.input_contract=ur7e_plain_points_v1`、`plain_trajectory_sha256`、
`reference_sha256`。尚无批准人的配置可自动填已知绑定事实，批准人与有效期仍需现场填写。
已有旧格式批准不会自动迁移：重新核验后显式将这三个字段绑定为脚本固定参考中的输入契约、
`PLAIN_REFERENCE.plain_trajectory_sha256` 和 `PLAIN_REFERENCE_SHA256`，再记录本次批准与有效期。
`inspect` 会列出缺失或不匹配项。手改轨迹点后必须从核验过的原导出包重新生成交付文件。

将这两个文件复制到工作站任意目录（也可放 `marm_planning/scripts/`），运行：

```bash
python3 ur7e_saved_trajectory_executor.py inspect --trajectory trajectory.json
python3 ur7e_saved_trajectory_executor.py check --trajectory trajectory.json --out check_001
python3 ur7e_saved_trajectory_executor.py execute --trajectory trajectory.json --out run_001
```

`inspect` 离线列出尚缺的现场参数；`check` 只读。`execute` 现场配置未完成时拒绝发送；
现场就绪后可发送 geometry_only 轨迹，仍要求已在首点、有效力/TF/状态、缩放进度、控制器互斥和看门狗。
不自动移到起点、重新规划或故障返回。JSON、脚本、配置或固定参考在预检期间改变会拒绝发送。
运行输出目录必须新建，不要将日志写进正式交付目录。

#### 运动速度倍率

独立脚本的 `inspect/check/execute` 均支持 `--speed-factor`，默认 `1`；`2` 为两倍指令速度，
`0.5` 为半速。只在启动前选定倍率，不在运动中切换：

```bash
python3 ur7e_saved_trajectory_executor.py inspect --trajectory trajectory.json --speed-factor 2
python3 ur7e_saved_trajectory_executor.py check --trajectory trajectory.json --speed-factor 2 --out check_2x_001
python3 ur7e_saved_trajectory_executor.py execute --trajectory trajectory.json --speed-factor 2 --out run_2x_001
```

加速（倍率大于 1）还需在脚本 `SITE_DEPLOYMENT.motion_limits` 填写：

```json
{
  "joint_velocity_rad_s": null,
  "joint_acceleration_rad_s2": null
}
```

将两个 null 分别替换为六个现场允许值组成的正数数组，顺序与 `SITE_DEPLOYMENT.joint_names` 一致，
单位为 rad/s 和 rad/s²。这些值由实际机器人、负载、工具与工况确定，交付不猜测允许上限。
独立脚本用整段五次插值的 Bernstein 保守界检查速度和加速度，缺失或超限会在连接 ROS 前拒绝，
报告列出超限关节。该检查不等于完整碰撞、负载动力学或接触运行验收。原速与减速不新增此必填项。

在内存中缩放时间 `t/s`、速度 `dq*s`、加速度 `ddq*s²`，保持输入文件、授权哈希与关节点不变。
阶段时间同步缩放，消息时刻取最近纳秒；发送和监控使用同一量化曲线。倍率须有限且大于 0，
时间溢出或纳秒时刻重合会拒绝。力、跟踪、看门狗、新鲜度及所有墙钟超时不随倍率放宽。
报告新增 `speed_factor`、`original_duration_s`、`commanded_duration_s`、`execution_curve_sha256`、
`max_clock_rounding_s` 和 `speed_limit_audit`；`duration_s` 继续表示原时长。
非原速时 `original_dynamics_audit_applies=false`，不继承原时序的动力学验收。

本次几何回放交付原时长约 124.88 s，2 倍速指令时长约 62.44 s、加速度变为约 4 倍。
示教器速度滑块、UR speed scaling 与保护减速仍会影响实际墙钟时长，完成判断依据控制器实际进度。
独立 full 策略下非 1 倍速须重新审计并导出，当前会明确拒绝；项目原 CLI 不增加此倍率选项。
审核方案：`docs/superpowers/specs/2026-11-16-ur7e-standalone-speed-factor-design.md`。

#### 按键启动与停止

独立 `execute` 可加 `--keyboard`，与速度倍率同时使用：

```bash
python3 ur7e_saved_trajectory_executor.py execute --trajectory trajectory.json --speed-factor 2 --keyboard --out run_keyboard_001
```

需在 Linux 前台交互终端运行，终端必须获得键盘输入焦点；无需回车。
看到 `Ready. Press S to start` 后按 `S` 或 `s` 开始整条轨迹，按空格终止本次执行。
准备好之前的 S 不排队启动；空格始终锁存且优先于启动。未发送轨迹时空格取消启动，
不发送 goal/cancel/stop；开始发送后空格走原取消、停止服务及实际停稳确认流程。
重复 S 不重复发送，停止后不能按 S 续跑，也不自动返回起点。

等待 S 期间持续检查起点、静止、力、状态新鲜度与看门狗，并维持执行心跳。
收到 S 后重新检查授权有效期、输入及配套参考指纹和实时状态；等待耗时不计入运动墙钟超时。
无交互 TTY、后台进程、输入 EOF/监听故障不会回退为自动运行，检测到输入故障会取消或停止。
`Ctrl+C` 保留原故障停止处理，退出时恢复终端设置。强制终止进程或终端被关闭的情形仍依赖独立看门狗。

按键监听不改变原轨迹时间、位置、速度、加速度或速度倍率，只控制发送时机和终止。
键盘检测、监控周期、网络与制动均有延迟；空格是软件停止，不能代替实体急停。
切换焦点后按键可能收不到，监听器无法据此证明操作员仍能控制本终端。

报告的 `keyboard_control=true` 标明该模式，`operator_wait_s` 记录接受 S 前的等待时间，
`operator_stop_reason` 记录空格或输入故障；状态 `cancelled_before_start` 表示未发送时取消，
`operator_stopped` 表示人工终止且确认停稳，不代表完整轨迹完成。
停稳未确认或输入故障为 `fault_latched`，停止日志保留确认及错误信息。
正常完成、人工取消及已确认人工停止返回 0；阻断或故障返回 2，不能只靠退出码判断轨迹完成。
未指定 `--keyboard` 时保持原启动方式，`inspect/check` 不接受该参数。
审核方案：`docs/superpowers/specs/2026-11-16-ur7e-standalone-keyboard-control-design.md`。

代码由 `standalone_pack.py` 静态合并权威运行模块和 `standalone_runtime.py`，保留源码哈希注释，
生成脚本只编辑 `SITE_DEPLOYMENT`，运行逻辑与固定参考应重新构建，避免出现第二套实现。
按已审核的单文件方案生成，正式交付目录是用户指定的产物例外。
明文方案：`docs/superpowers/specs/2026-11-16-ur7e-plain-trajectory-json-design.md`。

### 测试

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_ros1_hardware -v
.\.venv\Scripts\python.exe -m unittest tests.test_standalone_hardware -v
.\.venv\Scripts\python.exe -m unittest tests.test_keyboard_hardware -v
.\.venv\Scripts\python.exe -m unittest tests.test_plain_hardware -v
.\.venv\Scripts\python.exe -m unittest tests.test_moveit_hardware -v
```

测试用隔离文件和 Mock，不连接设备。Linux ROS、UR7e、真实力补偿、看门狗和停止试验均另需验收。
如需核对外部旧 `dict_to_trajectory`，设置环境变量 `UR7E_LEGACY_READER` 为旧脚本路径再运行
`tests.test_plain_hardware`；不设置时仅跳过这项外部参考测试，其余契约测试正常运行。
