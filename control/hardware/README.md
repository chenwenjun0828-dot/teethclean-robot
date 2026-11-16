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

### 测试

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_ros1_hardware -v
```

测试用隔离文件和 Mock，不连接设备。Linux ROS、UR7e、真实力补偿、看门狗和停止试验均另需验收。
