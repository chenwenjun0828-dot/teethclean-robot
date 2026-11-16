# UR7e 单文件轨迹执行端

- 状态：已批准；两文件交付及离线/Mock 验证完成，现场验收未运行
- 问题与目标：开发者明确希望工作站不依赖本项目，只复制一个控制 Python 文件和一个 trajectory.json，在已有 ROS 1 UR 驱动环境中执行保存的完整轨迹。减少交付文件和安装依赖，不通过删去来源核验或接触运行保护实现精简。
- 审核范围：独立文件打包和运行入口；项目原入口保留全部门槛，独立真机入口允许经操作者授权的 geometry_only 回放，保留现场检查和保护。以下为批准时开发者对原方案的明确调整。

## 现状与证据

1. 当前 Git 基线为 5ae1570，调查开始时工作区干净。现有桥接已包含离线 JSON 契约、执行检查和 ROS 1 FollowJointTrajectory 适配。
2. 参考 smooth_path_executor_1111_json2real.py 从 JSON 恢复关节位置、速度、加速度和时间，再调用 MoveIt execute。构造对象时额外运动到固定 Cartesian 位姿，执行前另行规划到首点；这些行为不属于本次独立执行端。
3. 参考工作空间配置 scaled_pos_joint_traj_controller/follow_joint_trajectory，但声明机器人为 UR3e。现场 UR7e 的启动、标定和接口依然未确认，不能据此填写默认值。
4. 当前 trajectory.json 不是自足的执行输入：load_bundle 还核对 export_report、source_manifest 及来源快照，部署参数由另一个 JSON 提供。直接只复制现有 trajectory.json 会丢失这些检查所需的数据。
5. 当前现成轨迹为 geometry_only，execution_ready=false、simulation_only=true、robot_execution_allowed=false、camera_verified=false。单文件交付不改变这些事实。

## 范围与选择

- 推荐：生成一个自足的 Python 执行文件，并把完整原始导出及 deployment 配置封装成单个 JSON。工作站只需要这两个输入文件，不需要项目源码、规划资产或 MuJoCo 环境。
- 备选：手写一个只恢复 JointTrajectory 并执行的短脚本。虽然代码更短，但绕过现有来源、反馈和接触保护，不采用。
- 备选：复制薄入口和若干运行模块。复用最直接，但不满足开发者指定的单代码文件部署，不采用。
- 新增离线打包入口归属 control/hardware/；开发者明确指定正式交付目录 control/employment/2026_11_16/，只含 ur7e_saved_trajectory_executor.py 和 trajectory.json。生成脚本是正式可重建交付，不是实验副本。
- 更新 control/hardware/README.md、ARCHITECTURE.md；可复用跨模块回归放 tests/。不修改 robot_clean，交付后可由开发者把生成的执行文件放入其 marm_planning/scripts/。
- 不新增 RTDE、MoveIt 规划、自动进场到首点、恒力算法、驱动启动或控制器切换，不修改机器人/工具/牙模配置，不改现有正式导出文件。

## 两文件契约

1. ur7e_saved_trajectory_executor.py 为普通可读 Python 源文件，运行时不导入 control、planning、perception、pipeline，也不定位或下载本项目。标准库、NumPy 和现有 ROS 1 消息/通信包为外部依赖；不需要 MoveIt、SciPy、MuJoCo、TOPPRA、Open3D。
2. 单文件由离线打包器从现有 trajectory_contract.py、ros_runner.py、ros1_trajectory.py 生成，去掉项目相对导入并接入独立 CLI。保留同一份运行逻辑作为权威实现，避免手工维护另一套简化控制器；构建时检查顶层名称冲突和不允许的导入。
3. 新 trajectory.json 使用独立封装版本 ur7e_standalone_v1。包含 files（原始导出文件名到 Base64 原始字节的映射）及 deployment（现有部署配置对象）。只接受既有契约所需的明确 JSON 文件；文件名不得包含目录、绝对路径或重复项。配置不含密钥。
4. 封装保留原 trajectory.json、报告、清单和快照的精确原始字节，因此不需要重写它们的哈希。authorization 继续绑定内部原始轨迹、清单及报告，不能把外层封装哈希误填入这些字段。
5. 内部已有轨迹结构、关节顺序、q/dq/ddq、阶段、来源标记和单位原样保留。新封装只供新独立脚本消费；现有 CLI 和旧 json2real 不消费此封装，原有导出格式保持兼容。
6. 独立脚本先将已验证名称的文件恢复到系统临时目录，恢复 deployment，再运行相同检查和执行逻辑，结束后清理自身临时数据。日志写入操作者指定的新输出目录，不写入 robot_clean 源码目录。
7. 加载后记录外层输入文件 SHA256，发送前复核外层输入未变；内部指纹和部署授权仍按现有逻辑复核。哈希用于一致性检查，不宣称能证明恶意修改不存在或代替现场审核。

## 操作与失败处理

拟交付 CLI：

```bash
# 不连接 ROS，展示封装、配置和轨迹阻塞项
python3 ur7e_saved_trajectory_executor.py inspect --trajectory trajectory.json

# 连接已启动驱动，只读现场状态；不发送 goal、心跳或停止命令
python3 ur7e_saved_trajectory_executor.py check --trajectory trajectory.json --out check_001

# 现场和来源门槛通过后，显式发送完整轨迹
python3 ur7e_saved_trajectory_executor.py execute --trajectory trajectory.json --out run_001
```

- 三种模式均无自动起点运动；execute 只发送一次完整 FollowJointTrajectory。关节 rad、时间 s、TCP m、力 N、力矩 N*m；不换符号、偏置、归一化或重新计时。
- 项目原入口的 source/full 和 ROS 运动审计门槛不变。独立脚本使用单独明确的 execution_policy：full 仍沿用旧门槛；geometry_only 允许在来源字节和插值审计有效、配置中明确 acknowledge_unverified_motion=true 且现场授权有效时回放。原 execution_ready/simulation_only/robot_execution_allowed/camera_verified 与未通过 full 的事实不改写为通过，作为 source_warnings 和原 source 写入执行报告。对它们的例外仅适用于该独立几何回放；现场机器人、配准、工具、力补偿等证据与实时核对仍必须满足，不能由本次打包虚构。
- 复用原运行器时注入显式 source_gate 回调；默认仍为旧 execution_blockers，项目原 CLI 不注入例外。独立脚本不以项目仿真通过证明真机就绪，也不把取消 full 断言描述为完整避碰或动力学验证通过。
- 插值审计、来源指纹、现场配置、授权、起点、TF、关节/力新鲜度、控制器互斥、缩放进度、停止和独立看门狗要求保留。
- 力反馈、独立看门狗和真实停止路径仍须工作站提供；单文件客户端不能独立保证自身崩溃后已提交轨迹停止。既有驱动无法提供有效进度时仍拒绝，不能改用墙钟追赶。
- 未确认的 IP、型号标定、接口和阈值保持空值。允许打包含未配置项的诊断产物，必须明确报告不可执行；不能虚构证据或把配置齐全视作现场验收。
- 当前 geometry_only 产物可按独立策略回放；现场未知参数和授权仍为空，因此本机交付 execute 应因现场缺项而拒绝，不再以 geometry_only 本身拒绝。缺字段的旧普通 JSON 不提升为可执行。
- 故障取消目标、调用已核实的停止服务、确认速度并记录；不重试、不自动沿退场段返回、不自动解除保护。软件停止不等于硬件急停。

## 实施步骤

1. 获批后实现离线两文件打包及确定性的单文件代码生成，保留原多文件入口。
2. 实现封装校验、临时恢复及 inspect/check/execute 独立参数入口，复用现有生命周期。
3. 用当前真实导出和空部署模板生成正式两文件交付，自动填写已知的关节顺序、固定模式和三个原始文件哈希；未知现场参数及负责人/有效期不虚填。明确当前现场未配置，记录两个文件的位置及哈希，不覆盖现有导出。
4. 补独立启动、故障拒绝及与原执行端等价性回归，更新部署说明和验收记录。

## 验证与验收

- 从仓库外隔离目录运行生成脚本，只放生成脚本和封装 JSON，清除项目 PYTHONPATH。help/inspect 不要求 ROS，且不导入项目或规划/仿真包。
- 打包往返：内部文件字节与 SHA256 一致、配置一致；首末点、全部时间和 q/dq/ddq 与原始轨迹不变；原输入文件哈希保持不变。
- 拒绝缺失文件、损坏 Base64、非法/重复文件名、错误哈希、版本不符、缺失现场参数和旧无来源限制的 JSON。
- 项目旧入口继续拒绝 geometry_only。独立入口以有效 Mock 现场配置证明 geometry_only 可正常单次发送并完成；当前真实封装因现场未配置而拒绝且不构造 ROS 后端。Mock 验证只读零运动、起点不符零 goal、反馈/力异常停止、取消/中断、停止未确认，沿用原生命周期。
- 回归运行 tests.test_ros1_hardware 及新增单文件专项；这是跨执行契约改动，再运行根 tests 回归和 git diff --check。规划算法未改，不据此扩大到无关仿真验收。
- 本机仅离线与 Mock 验证。Linux ROS、UR7e、现场力补偿、缩放进度和独立停止仍需工作站验收；不自动连接或驱动真实设备。

## 兼容性、风险与回退

- 新封装必须与旧普通轨迹文件清楚区分；文件名相同不意味着格式通用。README 明确新旧入口，防止交给旧 json2real。
- 保留完整执行保护意味着生成文件可能有数百行；目标是只部署一个代码文件，不承诺几十行完成接触运行保护。
- Base64 会增大交付大小，但仅使用标准库且保留精确字节，避免另引压缩格式和解包路径逻辑。
- 生成器变更需证明生成文件与权威实现一致，不能在产物上直接修补并造成第二实现。停止现场风险不因单文件交付降低。
- 回退仅移除本次新增打包入口、文档和自己生成的交付目录；保留旧桥接、规划源数据及其他开发者修改。不迁移或覆盖原工作站脚本。

## 审核记录

- 2026-11-16，本会话开发者在看到具体方案后回复“当前 geometry_only 轨迹仍保留禁止执行的限制。解除这个限制，然后实施”，批准实施并要求调整独立执行策略。
- 随后明确“项目里可以有这些限制，我的意思是到真机上去之后，可以正常执行”：本次仅在独立交付入口采用几何回放策略，项目原入口保留限制。
- 随后指定“把json（轨迹）和控制代码，放在control文件夹下的employment里把，用日期2026_11_16来命名子文件夹，包含轨迹和控制代码”：交付改为 control/employment/2026_11_16/，覆盖上文原 outputs 位置建议。

## 实施与验证记录

- 2026-11-16：新增 standalone_pack.py、standalone_runtime.py 和 standalone-pack CLI，生成正式交付 control/employment/2026_11_16/ur7e_saved_trajectory_executor.py 与 trajectory.json。工作站不导入本项目；JSON 顶层 deployment 可填写现场参数。
- ros_runner.run 仅增加 source_gate 和 before_send 可选回调，默认保留原 source/full 拒绝条件；原入口拒绝同一 geometry_only 数据的回归通过。独立入口明确 geometry_only 策略，真实源的七项项目限制作为警告保留，几何策略阻塞项为空。
- 当前交付 16006 点、124.879636567 s，首末关节逐元素相同，腕关节保持 -4.774522701755688 rad。恢复后的 10 个原导出 JSON 与原文件逐字节一致；未改写原来源、审计或规划文件。
- 生成脚本 42918 bytes，SHA256 036fdc0c83c2e79f20d89930d6f4494db094790a935e503bc1c1bf7c3d5e0fc9；封装 JSON 20705684 bytes，SHA256 ef94a03af1fafef9e4e32af5e9be4ed0c1249d991ce1470e03ff666e265a0386。
- python -m unittest tests.test_standalone_hardware tests.test_ros1_hardware -q：48 项通过，其中新增独立交付测试 22 项；覆盖无项目/ROS 离线启动、项目原门槛保留、独立几何单次发送完成、full 入口、起点不符、力超限、力断流、停滞、停止失败、KeyboardInterrupt、授权失效、指纹损坏、路径/重复键、文件变化和字节恢复。
- python -m unittest discover -s tests -p 'test_*.py' -q：367 项通过，95.505 s。未修改规划算法，不额外运行完整规划仿真。
- 将真实交付两文件复制到仓库外 TemporaryDirectory，以 python -I 启动 execute。结果 blocked、motion_send_attempted=false、motion_sent=false、退出码 2；仅缺现场配置/授权，不以 source/full 标记拒绝，也未导入 ROS。验收记录 outputs/ur7e_standalone_20261116/acceptance.json。
- 本机未连接机器人，未运行 Linux ROS、真实 UR7e、力补偿或停止距离验收。配置 evidence 是记录而不是校准证明，独立看门狗仍需现场部署；本次不声称接触轨迹已完成避碰或动力学验证。
