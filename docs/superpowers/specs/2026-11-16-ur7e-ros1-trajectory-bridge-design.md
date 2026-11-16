# UR7e 固定牙模轨迹 ROS 1 接入

- 状态：已批准；本机离线接入与 Mock 验证完成，现场验收未完成
- 问题与目标：开发者希望在固定牙模上接触运行当前直线进场、沿线、直线返回轨迹，并指定 `robot_clean/src/marm_planning/scripts/smooth_path_executor_1111_sim2json.py` 及相关代码作为参考。本方案建立现有规划产物到 ROS 1 UR 驱动的导出、只读核对和受保护的位置轨迹执行链路；不把转换成功认定为接触或真机验收通过。
- 本次运行边界：在本机完成实现和 Mock/离线验证，不自动连接、启动或驱动真实机器人。实际发送运动须在完成部署检查后由操作者通过独立执行入口启动。

## 调查证据

1. 参考工作空间的 `README.md` 声明 ROS 1/Catkin/UR3e；`ur3e_moveit_config/config/ros_controllers.yaml` 使用 `scaled_pos_joint_traj_controller/follow_joint_trajectory`，六个 ROS 关节名没有 `ur_` 前缀。该信息仅说明参考工程，不能证明现场 UR7e 使用相同模型、控制器或标定。
2. `smooth_path_executor_1111_sim2json.py` 的 `trajectory_to_dict` 保存 `joint_names` 及 `positions/velocities/accelerations/time_from_start`。脚本还会运动到 named target、预设位姿，并调用笛卡尔重规划及执行，因此不能导入或直接运行它来做离线转换。
3. `smooth_path_executor_1111_json2real.py:471` 将 JSON 重建为 `RobotTrajectory` 并执行；初始化还会运动到硬编码位姿，执行前另行规划到轨迹起点。本次只参考其消息字段，不继承这些额外运动、路径重规划或硬编码 TCP/位置。
4. `force_control/kunwei_sensor.py` 的接口只返回最近六轴值，无采样时间戳；初值为零，断流后可能继续读到旧值。示例打印使用 Kg，而部分调用方按 N 解释。`force_safe_demo.py` 的固定零偏和 `move_group.stop()` 不能直接作为本次完整轨迹接触执行的已验证保护。
5. 当前源目录 `outputs/planning/session_straight_return_35_m20_15_20261116` 是 schema 8、`geometry_only_verified`、`execution_ready=false`，session 为 `robot_execution_allowed=false`、`optical_frame_hypothesis_live_tf_unverified`。当前保存轨迹 2084 路点、124.879636567 s；首末关节完全一致。以前同参数 full 检查在沿线局部支持验证处失败，转换格式不能消除此失败。
6. 本仓库 `control/hardware/ur_robot.py` 只有注入式预留接口；`control/position/controller.py` 使用 MuJoCo 执行与逆动力学，不是 ROS 真机执行器。参考工程已具备 UR ROS 驱动，因此无需为本目标再并行建设一套 RTDE 运动链路。
7. 2026-11-16 开发者要求 UR7e 当前 roslaunch/型号/标定信息“先留空”。这些字段保持未配置，禁止根据 UR3e 文件名或历史 IP 推定现场信息；缺失时允许离线处理，禁止执行。

## 范围与选择

- 推荐：本项目导出带来源与插值审计的 JSON，Linux ROS 1 端通过官方 UR 驱动的 FollowJointTrajectory action 消费；保留整段路径和时序，不重新求 IK。
- 备选：新建 RTDE/servoj 执行器，需要另一套通信周期、驱动互斥、时间缩放和停止验证；现有 ROS 资产可复用，本次不采用。
- 备选：直接复用旧 json2real，接入较快，但它会额外运动且缺少本轨迹的来源、直线插值与力数据有效性检查，不采用。
- 主要文件：按开发者批准时指定，导出、契约、运行监控、ROS 适配和 CLI 均放在 `control/hardware/`，不接入现有 MuJoCo 位置控制循环。更新所属 README/ARCHITECTURE，回归放现有 `tests/`。
- `robot_clean` 只读参考，不修改其中业务代码、第三方驱动或配置；Linux 运行端使用本次交付的桥接入口。ROS 依赖按需导入，不引入默认安装要求；真机端消费 JSON，不依赖 MuJoCo、TOPPRA、Open3D。
- 不修改感知线、角度、0.2 mm 偏置、工作半径、负间隙阈值、机器人模型或标定。不放宽原 full/来源/执行门槛，不实现恒力控制、超声器械启停或真人实验。

## 数据流与接口

### 1. 离线导出

输入为规划目录；使用已有 `Trajectory.load/sample` 和场景/参考加载能力核实来源、单位和实际插值。输出到新的 `outputs/<run>/ros1_export/`：`trajectory.json`、导出报告及来源清单。源产物保持不变。

JSON 保留参考脚本的核心字段：`joint_names`、`points[].positions/velocities/accelerations/time_from_start`；另保存版本、单位、阶段边界、首末关节、源文件哈希、原检查模式、执行就绪状态、session 限制、映射和导出审计。旧脚本可能忽略这些元数据，因此只支持新桥接执行器，不宣称文件可直接交给旧 json2real。

关节名显式一对一映射，如 `ur_shoulder_pan_joint -> shoulder_pan_joint`，按目标关节名排列所有位置/速度/加速度；无隐式去前缀、符号翻转、关节偏置或 2*pi 归一化。现场名称不符、重复、缺失或多余时拒绝。关节使用 rad、rad/s、rad/s^2，时间 s；位置和 TCP 使用 m，显示误差 mm。关节目标本身不做 world/base 坐标变换，但 FK 核对必须区分 `controller_base/base_link/tool0/洁治器 TCP`，结合已标定变换，不能把 tool0 当器械尖端。

导出采样以实际时间曲线为准，不只导出原关节节点。默认候选间隔 0.01 s，合入原路点、阶段边界及源时间律分段边界；无界面或运动副作用。携带 q/dq/ddq，按已确认控制器的五次 Hermite 插值重建 ROS 端参考，检查区间内部 FK、原曲线偏差、直线进退场及速率/关节界限。沿线 TOPPRA 与 ROS 插值一般不解析等价，必要时仅细化导出采样，不能重拟合感知线或隐藏重新计时。位置/方向容差沿用原 0.1 mm/1 度；导出近似误差单独报告，不把它与原规划误差混为一项。加密上限与失败诊断显式记录。

geometry_only 可导出诊断文件，必须原样保留不可执行标记；执行器拒绝它。不能通过 JSON 字段、命令开关或复制文件提升为执行就绪。

### 2. ROS 1 只读核对

入口默认仅检查 JSON 和订阅状态，不发送 action goal、不调用 MoveIt go/execute，不启动 External Control、不解除保护停止。读取 `/joint_states`、控制器运行状态、UR 机器人/安全状态、TF/已标定工具信息和真实力数据，输出核对报告。

机器人型号、序列号/模型来源、PolyScope/驱动版本、控制器/action 名称、工厂运动学标定、工具 TCP/安装方向、负载、现场牙模配准与有效期均由部署配置给出。UR7e 的启动命令和对应字段当前为空；不得填入旧工程 UR3e 标定或 `192.168.0.14` 作为默认。

现场需核对初始六关节与保存起点、工具尖端在多组位姿下的 FK、工具/相机/牙模是否移动以及 live TF。阈值来自经过审核的部署参数，不自行把 0.01 rad 视为适合 0.2 mm 离面的容差。信息不足时报告缺项，不启动运动；也不自动移动到起点。起点姿态准备属于操作者独立流程，不能新增路径混入本次轨迹。

### 3. 执行门槛与运行

执行为独立显式入口。必需：完整 full 规划审计和执行来源指纹一致；现场机器人/模型/坐标/工具/牙模核对有效；真实力和机器人反馈有效；部署停止与监控链已验证。保留 `load_verified` 的既有拒绝条件，并额外检查硬件授权与现场证据，不把仿真 `execution_ready` 当作硬件授权。

当前 session 的硬件限制和相机解释状态是实际阻断项。本次接入开发不会把它们改为通过，也不提供跳过开关。解除这些限制或修复 contact_unverifiable 需要真实核对证据及针对具体来源的后续审核；在此之前当前轨迹只能导出、核对和 Mock 执行，不能真实发送。

执行器核实 scaled 控制器已运行、其他命令源未占用，再发送有完整首末点的 FollowJointTrajectory。不调用笛卡尔规划、不套固定四元数、不自动切控制器，不并行运行 MoveIt Servo 导纳节点抢占相同命令通道。正常成功才报告完成；action 返回、实际关节停止与最终关节/TCP 误差必须同时检查。

本次模式是“保存位置轨迹回放 + 力超限停止”，不承诺恒力，也不保证正离面 0.2 mm 的路径一定产生接触。接触是否发生由真实力日志判断，不能通过偷偷取消偏置或向牙面压入实现。若目标是持续目标力跟随，需要另行设计获取接触、法向柔顺、最大偏移和释放策略，且会改变实际路径，不能与本次精确路径回放混称。

可显式增加总时长的统一缩放 k>=1：t'=k*t、dq'=dq/k、ddq'=ddq/k^2，重做导出插值审计；禁止加速。UR speed scaling 属于运行时反馈，超时/进度按控制器反馈处理，不能用固定墙钟时间认定结束或强行追赶。所用限速、关节/笛卡尔误差阈值需在部署记录中明确。

### 4. 力保护与故障

初期消费配置的、带时间戳的真实六轴力/力矩接口，SI 单位 N/N*m，记录帧、单位验证、零偏、工具重力补偿及有效范围。若采用坤维 TCP 接口，只能在验证协议、单位后在硬件模块实现一次权威适配，必须携带实际收包单调时间和连接状态；不得复制旧的“最近零值/旧值即有效”行为。

各阶段力/力矩阈值、数据新鲜度、跟踪阈值、最大停止响应时间均为部署必填项，无适用于牙模接触的臆定默认力值。转姿态时固定初始零偏不能替代重力补偿；无法确认补偿有效时拒绝接触执行。力超限、传感器/关节状态超时、非有限值、保护停机、action 异常、取消或中断均进入停止状态，锁存原因，禁止自动恢复或自动沿返回段退回。

故障时取消当前 goal，并通过已核实的 UR 停止接口请求停止，等待真实速度反馈确认。普通 ROS cancel/软件 stop 不是硬件急停，不能声称零延迟停车。必须测量从触发到实际停止的延迟及移动距离，并相对于固定牙模场景验收。

特别风险：整个 action 已提交后，监控进程崩溃不必然使控制器停止。接触执行必须有独立于提交进程的监控/看门狗和可验证的停止路径；不能把客户端 finally、ROS shutdown 回调或驱动连接存在当成保证。本次实现需检查该部署证据；若现有驱动不能满足，不开放真实接触执行，另行审核下层保护设计。保留并现场测试示教器停止与硬件急停。

## 实施步骤

1. 本方案经开发者明确审核后，先补 JSON 字段/映射、源状态传递、插值等价性与拒绝执行回归。
2. 实现离线导出和控制器插值审计，使用当前轨迹生成明确不可执行的诊断 JSON，记录大小、样本量、误差和阶段边界。
3. 在 `control/hardware/` 实现 ROS 端契约解析、只读核对、运行监控和 Mock action 测试；不改 MuJoCo 控制器。
4. 实现受门槛约束的位置回放、真实状态/力监控和日志；未配置现场条件时测试必须拒绝发 goal。现场参数模板保持 UR7e 未确认项为空；文档区分配置检查、Mock、ROS 仿真和实机验收。
5. 提供 Linux ROS 环境运行说明及只读采集命令。审核与本机测试均不连接设备；后续现场部署和启动不作为本轮已完成结果。

## 验证与验收

- 导出往返：有限数值、严格递增时间、首末及阶段边界精确保留；含 q/dq/ddq，重排后恢复原始关节序列；不做 2*pi 改写。
- 几何：ROS 插值区间内部与源时间曲线对比，真实 MuJoCo FK 检查进场/沿线/返回；超差细化或失败，不能只检查导出点。
- 来源：任一源/导出数据/部署绑定改变即拒绝；geometry_only、simulation_only、硬件禁止、相机未核实、缺 full 审计均不能变成可执行。
- 运行：Mock 证明无自动起点运动、无二次 IK/重规划、无并行命令源、无故障自动退回；全阶段监控且只有正常终态确认才成功。
- 故障注入：力超限、力/状态断流、初值零但无有效帧、单位错误、错误关节映射、起点不符、action 拒绝/超时、速度缩放、进程异常与停止未确认。缺部署看门狗证据时不发目标。
- 回归命令：项目 `.venv` 下 `python -m unittest discover -s tests -p 'test_*.py' -q`、`python -m unittest discover -s planning -t . -p 'test_*.py' -q`；ROS 专项仅在有相应依赖的隔离 ROS 环境执行，未运行需明确记录。最终 `git diff --check` 和工作区检查。
- 当前阶段通过条件：诊断 JSON 和离线/Mock 验证通过；真实运动保持被正确阻止。固定牙模接触验收另需现场数据、full 及停止试验，不能以本机单测替代。

## 兼容性、风险与回退

- 现有 MuJoCo、trajectory.npz、规划默认参数和旧 JSON 脚本不修改；新增桥接契约只约束新入口。不升级 ROS 或规划依赖。
- 控制器插值、工厂标定、工具尖端与 tool0 差异、实时速度缩放、传感器单位/补偿以及进程失效后的继续运动是主要风险，必须按上述检查逐项验证。
- 回退仅移除本轮新增桥接入口及对应文档/测试，保留原始轨迹与新 outputs 诊断数据，不修改其他已有工作区改动。

## 审核记录

- 2026-11-16，本会话负责开发者在收到本方案后回复“接入 放在control的hardware里”，批准本方案的离线导出、只读核对、受保护 ROS 1 接入及 Mock 验证，并指定全部接入代码放在 `control/hardware/`。本次不连接或驱动实机。
- 待现场补充：UR7e 实际驱动/型号/标定与启动配置、工具 TCP、现场坐标核对、真实力接口/单位/补偿、阈值及独立停止监控验证。依开发者要求暂不填写推测值，不阻塞离线开发，阻止真实运动。

## 实施与验证记录

- 2026-11-16：新增 CLI、JSON 契约、导出与插值审计、运行监控和 ROS transport，全部位于 `control/hardware/`；使用独立 action 生命周期，不接入 MuJoCo 位置控制循环。硬件包改为延迟导入 MuJoCo 后端，保留原导入接口。
- 运行说明在 `control/hardware/README.md`，现场模板为 `ur7e_deployment.template.json`，UR7e 未知信息仍为 null；参考工作空间未修改。
- 控制器进度实现细化：订阅 `JointTrajectoryControllerState`，以同帧的 `actual.time_from_start` 和 desired.positions 对照保存的曲线。ros_control 的 desired.time_from_start 可仅表示分段边界；部分旧 UR scaled 实现不填时间。现场必须提供已验证的缩放进度接口证据，旧版缺失时不可接触执行。模板含 `controller_state_topic` 和 `controller_progress_evidence`，未通过时不提供墙钟替代。
- 最终诊断 bundle：`outputs/hardware/ur7e_ros1_20261116/export_final/`。16006 点、124.879636567 s、JSON 15091374 bytes；首末关节逐元素完全一致，保留原 home 的 -4.774522701755688 rad 腕关节角，不归一化。
- ROS 插值与源 FK 最大位置差 1.1786e-9 mm、方向差 3.4151e-6 度；在线 TCP 参考插值的最大位置误差 0.00047109 mm。速率 Bernstein 界为 0.12664354 rad/s、0.65189443 rad/s^2。上述值对应导出报告声明的采样范围，不是连续避碰证明。
- 原独立沿线审计 16668 个样本通过；直线进场与返回审计通过，最大位置误差分别 0.000113733 mm、0.000004534 mm。导出后核对原规划目录 20 个文件哈希全部不变。
- `python -m unittest tests.test_ros1_hardware -q`：26 项通过，包括旧段边界时间与连续采样时间的区分、缩放进度、力/力矩超限、断流、非有限值、起点不符、发送异常、KeyboardInterrupt、指纹变化、停止未确认与只读零命令。隔离进程确认运行端不导入 MuJoCo/TOPPRA/Open3D/rospy。
- `python -m unittest discover -s tests -p 'test_*.py' -q`：345 项通过；`python -m unittest discover -s planning -t . -p 'test_*.py' -q`：142 项通过；`git diff --check` 通过。没有新增依赖。
- 对最终 bundle 调用执行门槛入口，产物 `outputs/hardware/ur7e_ros1_20261116/final_gate_check/run_report.json`：blocked、motion_send_attempted=false、motion_sent=false；在构造 ROS 后端前拒绝，未连接真实设备。
- 当前轨迹仍为 geometry_only、simulation_only、硬件禁止及相机解释未实测，未生成 full 碰撞或动力学通过结论。新 full ROS 曲线运动审计分支尚无执行就绪 full 源的端到端验收；Linux ROS、UR7e、真实力补偿和独立看门狗/停止试验均未运行。
- 本轮中间导出目录的清理被自动审批策略拦截，保留在同一 outputs 任务目录；交付只使用 `export_final/`。未删除来源不明文件或覆盖其他工作区修改。
