# 真机 session 接触轨迹可行性规划
- 状态：已按批准范围运行；本次参数未生成完整可行轨迹
- 问题与目标：开发者希望针对真机相机 session 规划一条完整下颌前牙轨迹，指定姿态角 `35/-20/15` 度、TCP 离面 `0.2 mm`、接触工作区半径 `2 mm`，并允许工作区最低有向间隙为 `-0.5 mm`。当前规划器将允许的负间隙幅度表示为非负 `contact_tolerance_mm`，因此应传 `0.5 mm`，其判据为 `gap >= -0.5 mm`。确认指定条件下能否通过仿真规划验收。
- 范围：只运行现有 planning 入口，不改算法、默认值、配置、标定、session 输入或硬件。输入采用 `assets/perception/sessions/20261113_220209`、`mandibular/outer_front` 和 `color_optical` session 解释；使用新的 `outputs/planning/real_session_contact_20261115_<run>/` 目录保存本次请求、完整轨迹及审计。规划模式使用 `continuous`，确保三个角度严格固定，不做逐点角度修正。明确 `execute=false`。不规划其他区域，不连接或驱动真实机器人，也不执行力控。
- 设计：调用 `python -m planning`，显式提供 session、牙颌与区域、`--planning-mode continuous`、`--pose-angles-deg 35 -20 15`、`--motion-mode contact`、`--standoff-mm 0.2`、`--working-segment-radius-mm 2` 和 `--contact-tolerance-mm 0.5`；使用 `--check-mode full`、严格方向审计和 headless 模式。保持 `--nonworking-clearance-mm 0` 默认值以及现有关节、采样、速度和 TOPPRA 参数。命令不包含 `--execute`；session 入口本身也拒绝执行。
- 选择依据：现有 planning session 入口会用拍摄 session 的坐标链、当前工具/机器人模型和牙颌网格建立 MuJoCo 场景，并可完成完整轨迹检查。`contact_tolerance_mm` 是非负容差幅度，负有向间隙阈值 `-0.5 mm` 对应 `0.5 mm` 容差。正 `0.2 mm` standoff 与显式 `contact` 模式兼容。使用 `continuous` 避免 ±5 度搜索改变请求姿态。
- 兼容性与风险：工作区允许最多 `0.5 mm` 的局部有向负间隙，是本次明确请求的较宽松数值接触容差；其他工具部位、牙颌、机器人自碰撞和动力学保护保持开启。接触容差不代表有意穿入牙体，也不提供力控。即使所有仿真检查通过，产物仍是 `simulation_only` 且 `robot_execution_allowed=false`；session 的 `color_optical` 解释仍是未现场核验的光学帧假设，不能据此宣称真机可执行或标定绝对精度。
- 实施步骤：1. 审核并批准本方案。2. 确认输出目录是新的空目录。3. 用上述参数运行 full planning。4. 检查 `planning_report.json`、`trajectory.npz`、碰撞/接触审计、TOPPRA 时序及动力学产物；若规划失败，保留失败报告和原因，不自动改变姿态、容差或安全参数。5. 记录实际命令、退出状态和验收结论。
- 验证与验收：报告须确认保存的角度为 `35/-20/15`、standoff 为 `0.2 mm`、接触半径为 `2 mm`、接触容差为 `0.5 mm`、模式为 `contact` 且 `check_mode=full`。只有 IK 全路径连通、进场/沿线/退出完整，工作区间隙判据、非工作区与自碰撞检查、轨迹插值/时序/动力学审计均通过，且 `execution_ready=true`，才判为“当前仿真模型下生成完整仿真可行轨迹”。同时检查 `simulation_only=true`、`robot_execution_allowed=false`；不得将结果表述为实机执行验证。
- 回退方式：不更改正式输入或源码。若运行失败，保留本次专属输出用于诊断；若输出本身不完整，只清理本次新建目录，不触碰其他输出。
- 审核记录：2026-11-15，开发者回复“确认”，批准本方案所列 session、下颌 outer_front 区域、固定姿态、离面距离、工作区半径、负间隙容差映射及 full 仿真验收范围；不包含真实机器人执行。
- 实施与验证记录：使用 `.venv` 运行参数 `--session-camera-frame color_optical --jaw mandibular --region outer_front --planning-mode continuous --pose-angles-deg 35 -20 15 --motion-mode contact --standoff-mm 0.2 --working-segment-radius-mm 2 --contact-tolerance-mm 0.5 --check-mode full --orientation-audit strict --headless`，不含 `--execute`。dry-run 输入验证通过。完整规划退出码为 1；输出 `outputs/planning/session_contact_35_m20_15_20261115/` 报告状态 `no_continuous_outer_front_sweep`、`execution_ready=false`、`simulation_only=true`。报告确认请求角度 `35/-20/15`、standoff `0.2 mm`、工作区半径 `2 mm`、容差 `0.5 mm`、完整碰撞检查开启。
- 失败诊断：连续 sweep 1,853 个位置的姿态候选均生成，但接触/碰撞验证阻塞 74 个候选节点，连续 IK 搜索在 layer 1745 返回 `no_continuous_ik_path`。报告列出的尾段 layer 1833–1852 中，工作区对下颌的局部支持因 `folded_or_conflicting_local_support` 无法验证。没有生成完整轨迹、TOPPRA 时序或动力学验收产物；不调整容差或其他参数重试。

## 2026-11-16 可达性诊断补充

- 开发者明确要求：“能先不考虑碰撞，生成一条这样的路径吗？”本次按该指示调用已有 `--check-mode geometry_only` 诊断入口，仅生成并检查同一 session/区域与姿态参数的几何轨迹；不修改源码、正式配置、安全策略或原 full 运行报告。
- 参数保留 `continuous`、`35/-20/15` 度、离面 `0.2 mm`、工作区半径 `2 mm`、接触容差 `0.5 mm`、姿态采样 `0.05 mm`，其余速度与关节限制沿用上次请求。关闭碰撞后，半径和容差只作请求记录，不用于验证接触安全。
- 诊断输出：新目录 `outputs/planning/session_geometry_35_m20_15_20261116/`。验收检查完整进场/沿线/退出轨迹、连续 IK、时序、几何与运动速率，以及 `geometry_only_verified`、`collision_checking_enabled=false`、`execution_ready=false`。不执行动力学、控制或真实机器人。
- 原 full 规划结论保持失败；本次诊断通过不能替代碰撞或执行验收。
- 运行结果：现有 CLI 按上述参数及显式 `--no-execute` 运行完成，退出码 0，状态 `geometry_only_verified`。1,853 个沿线姿态形成连续 IK；保存的 `trajectory.npz` 共 1,855 个关节路点，包含 `transfer/trace/retreat` 三阶段。总时长 `107.6943706534585 s`，进场 `45.50341146159407 s`、沿线 `61.11771085501081 s`、退出 `1.0732483368536236 s`。
- 验证结果：轨迹几何及运动速率审计均通过，沿线最大位置误差约 `0.00003821 mm`；沿线 TCP 速度上界约 `0.99222 mm/s`。`collision_checking_enabled=false`、`clearance_measured=false`、`fast_dynamics_verified=false`、`execution_ready=false`，session 仍为 `robot_execution_allowed=false`。现有 `load_scene` 成功重建预览场景并加载同一轨迹；101 个时刻的 `q/dq/ddq` 有限值检查通过。未验证碰撞、接触间隙或动力学，未运行控制。
