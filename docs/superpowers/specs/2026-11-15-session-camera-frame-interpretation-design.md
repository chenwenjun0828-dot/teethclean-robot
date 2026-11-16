# 真机 session 光学相机帧解释与仿真摆位

- 状态：已批准并实施，仿真坐标与回放验收完成。
- 问题与目标：当前 session 按手眼 YAML 声明的 `camera_color_frame` 使用 ROS stream-frame 轴转换，渲染中的两颌整体高于洁治器尖端，与开发者提供的拍摄照片和实际安装说明不符。目标是提供显式、可追溯的光学帧解释，用同一变换定位牙模、规划几何和回放，同时保留原始数据及既有解释。该选择用于仿真核对，不代表已完成现场 TF 或标定绝对精度验收。
- 范围：`perception/session.py`、`planning/session.py`、`planning/__main__.py`、`tests/test_perception_session.py`，必要的 CLI/回放集成测试，以及 `planning/README.md`、`planning/ARCHITECTURE.md` 和一个复用现有参数加载机制的 session 示例配置。运行产物放新建 `outputs/` 子目录。保持原始 session、YAML、相机外参、口扫、分割及六区源线不变；不调整机器人 XML、关节角、TCP、规划策略或保护阈值，不接入实机控制。

## 调查与证据

- `outputs/session_frame_hypotheses_20261115/comparison.png`、`audit.json` 和 `README.md` 保存了仅在内存中试算的两种解释；实验脚本位于仓库外，完成后已删除。
- 当前发布变换与读取程序数值一致，没有发现米/毫米混用、显示网格额外偏置、重复应用口扫变换。拍摄角度下，MuJoCo attachment site 与发布标称末端 FK 的位置差约 1.26 mm。
- 当前 TCP 高度为 controller_base 下 248.791 mm；下颌网格全部位于 333.725–404.084 mm，上颌位于 311.847–384.292 mm，因而并非观察视角导致牙齿高于尖端。
- 当前解释推导的相机光轴与工具安装 +Z 夹角约 89.84 度；光学帧假设下约 0.65 度。这是视线方向比较，不是完整相机姿态精度。
- 2026-11-15，开发者明确说明“镜头与洁治器大致同向，都朝向牙齿”，并提供拍摄照片，照片支持牙模位于腕部工具下方的空间关系。此信息支持光学帧候选解释，但不能替代现场三轴 TF 核实，也不构成实施批准。
- 光学帧试算下，网格顶点均值相对 TCP 的高度为下颌 -91.1 mm、上颌 -53.8 mm；当前解释分别为 +119.5 mm、+104.3 mm。中心定义为共享扫描网格顶点的算术均值。
- 原始记录保留相机帧转换未与 live TF 核实的标记；origin/wyh 交接说明称感知提供者已确认标准 ROS 约定。因此本方案不修改归档声明或伪造新的已验证来源。

## 设计与约定

`T_A_B` 把 B 中的点转换到 A，矩阵平移及运行时几何单位均为米；原始口扫毫米先转米。B 为 controller_base，E 为标定 ROS wrist_3_link，D 为 depth optical，C 为声明的 color stream frame，Co 为 color optical，S 为口扫，W 为仿真世界，M 为 MuJoCo ur_base。

1. 新增 `--session-camera-frame {declared,color_optical}`，默认 `declared`，仅与 `--perception-session` 配合使用。通过现有 `--params` 机制接受同名参数 `session_camera_frame`，不修改通用规划 Config。新增示例配置明确选择 `color_optical`，用于本批新预览；没有显式选择时保留现有行为。
2. 在 perception 中统一解析两种坐标链。`declared` 使用现有 `T_B_E @ T_E_C @ T_C_D @ T_D_S`。`color_optical` 将归档手眼数值明确解释为 `T_E_Co`，使用 SDK 的 `T_Co_D`，计算 `T_B_E @ T_E_Co @ T_Co_D @ T_D_S`；不再叠加 stream/optical 轴转换，不对手眼求逆，不手工平移牙模。
3. 始终先验证原始发布链、区域 CSV/NPZ 和 PreparedRegion 与原始发布 T_B_S 的一致性；显式派生候选链是之后的独立步骤，不放宽或绕过原验证。读取 SDK 外参时检查来源、单位及刚体矩阵，并记录本次实际依赖的文件指纹。
4. 选择 `color_optical` 时同时派生有效 SessionRegion：口扫局部曲线、网格及法向不重建；其 scan-to-base 使用有效 T_B_S。需要 base 位置/法向/切向时用 `Delta = T_B_S_effective @ inv(T_B_S_published)` 从已验证的发布 base 几何派生，方向只乘 Delta 的旋转。保留原始来源元数据，并明确标记有效几何来自新的帧解释，禁止原始 base 数组与有效 scan 变换混用。
5. planning 仍使用 `T_W_B = T_W_M @ Rz(pi)`，对上下颌分别计算 `T_W_S = T_W_B @ T_B_S_effective`。牙模摆位与 PreparedRegion 使用同一有效变换；机械臂保持 `robot_joint_observation.json` 中的六个拍摄关节角。
6. 新产物记录相机帧解释及其版本、实际使用矩阵、原始发布矩阵、源链、SDK 依赖指纹及“仿真假设、现场 TF 未独立验证”的状态。回放按保存的解释重建并核对有效牙颌位姿和相关指纹；旧产物没有新解释字段时只按旧 `declared` 解释重建，不自动转换成光学帧。既有基座适配版本校验仍保留。
7. 继续禁止 session 数据的 `--execute`；坐标候选可以供仿真规划检查，但不能把外观符合或几何可达解释为实机执行就绪。

## 选择依据与备选

- 推荐显式选择光学帧解释：与目前物理方向说明一致，可复现且保留两种解释，适合继续核对仿真和规划。其限制是原 YAML 的帧声明尚未被现场 TF 推翻，需要显式保留假设状态。
- 另一种方式是获得现场 TF 或重新导出已确认帧定义的标定后，重新发布完整 session。这是确认绝对正确性的更强证据，但当前机器缺少该记录；本方案不伪造该步骤。
- 不采用直接覆盖原矩阵或只挪动显示模型：前者会损失来源对应关系，后者不能保证规划曲线、法向和回放使用相同几何。

## 兼容性与风险

- 默认非 session 入口和 `declared` 解释不变；新解释只在明确选择时生效。原始资产和既有产物保留，旧轨迹不能视为已在新牙齿摆位下通过验证。
- 下游 PreparedRegion 的单位、坐标语义及局部曲线契约不变；有效变换和依赖记录需要随新产物保存。
- 光学帧候选仍可能存在相机安装变化、手眼结果失准、拍摄角度不同步或原配准误差。照片和工具近似网格只能支持空间关系核查，不能证明毫米精度。

## 实施步骤

1. 明确批准本方案中的光学帧候选解释、显式 CLI 选择和同时更新几何/显示/回放的范围。
2. 在 perception 实现原始链验证、有效链派生及区域几何一致性；增加针对两种解释的单元测试。
3. 通过 planning 入口传参，在 session 构建和保存记录中使用有效链；更新回放重建与错误检查。
4. 同步说明和示例配置，在新的输出目录生成拍摄姿态预览及截图。
5. 运行回归，保存实际验证结果；不修改保护阈值或用截图代替规划验收。

## 验证与验收

- `declared` 数值与现有发布值和现有测试保持一致；`color_optical` 与独立逐级点变换一致，单位误用、重复轴转换、矩阵取反能被测试检出。
- 两颌 body、规划曲线、点、法向、切向使用同一有效链；局部扫描几何不变，方向只旋转，六个关节角和上游机器人位姿不变。
- 在当前归档数据下重现诊断结果：下颌、上颌顶点均值相对 TCP 高度约 -91.1 mm、-53.8 mm，作为定位错误的回归样例，不作为真实精度门限。
- 新产物回放能保持所选解释；改动解释、有效变换或依赖文件后不能通过原产物核对。旧产物不被静默改成新解释。
- 实际运行 `python -m unittest tests.test_perception_session -q`，根目录与 planning 测试发现入口，以及 `git diff --check`；新增 CLI/集成测试按现有测试布局纳入发现。
- 用新的 session 参数做 dry-run 和 `--geometry-only` 导出，重新加载回放并截图核对。此验收不包含完整避碰、轨迹动力学或真实控制，不声明执行就绪。

## 回退方式

新命令改回 `declared` 或不传新增选项即可恢复原解释；只撤回本次加载/传参/说明/测试变更，保留原始资产与诊断、新产物用于对照，不清理其他工作区修改。

## 审核记录

- 2026-11-15，负责本次任务的开发者在收到本方案链接及实施确认请求后明确回复“确认”，批准上述范围。此前的镜头方向说明及拍摄照片只作为事实依据。

## 实施与验证记录（2026-11-15）

- 已实现显式 `session_camera_frame`，默认 `declared`；示例 `config/planning_session_optical_preview.json` 选择 `color_optical`，仅导出几何。未更改原始 session、标定资产、机器人 XML、关节角、工具或保护阈值。
- `perception/session.py` 先检查归档来源指纹和原发布链，验证 scan/base 曲线点和法向关系，然后派生有效位姿及一致的 base 点、法向、切向。`planning/session.py` 将有效位姿用于双颌摆位，并保存两套矩阵链、解释版本和依赖指纹；回放按这些记录重建并核对。
- `tests/test_perception_session.py` 覆盖独立逐级点变换、局部几何保持、base 方向旋转、双颌高度、拍摄关节角、旧输出默认解释、新输出回放、解释/矩阵/指纹变更拒绝、原始链校验和 CLI 参数/导出集成。
- 实际命令及结果：`python -m unittest tests.test_perception_session tests.test_planning_parameters -q` 定向 26 项通过；修正 CLI 测试临时目录隔离后，`python -m unittest tests.test_automatic_registration.AutomaticPipelineTests.test_automatic_pipeline_preserves_registration_and_stops_if_planning_rejects tests.test_perception_session -q` 15 项通过；`python -m unittest discover -s tests -p 'test_*.py' -q` 最终 317 项通过（92.350 s）；`python -m unittest discover -s planning -t . -p 'test_*.py' -q` 118 项通过；pipeline 原示例 dry-run 及 `git diff --check` 通过。均使用项目 `.venv/Scripts/python.exe`。
- 首次完整回归中，新增 CLI 测试误用了已有示例输出目录，已改用隔离临时目录；另有原自动配准测试出现 fitness=0.2000 未达原 0.8 门限，未更改算法或门限，单独复测与第二次完整回归均通过。最终完整日志保存在 `outputs/planning/session_optical_preview/root_tests.log`。
- `python -m planning --params config/planning_session_optical_preview.json --dry-run` 和同配置正式几何导出均通过。使用现有 `load_scene` 重新加载产物、通过 MuJoCo renderer 渲染并人工核对 `outputs/planning/session_optical_preview/capture_pose.png`；图像为 1200 x 900，RGB 标准差 69.20，有色像素 33322，非空且完整呈现机械臂和牙模，椅子按原查看器行为隐藏。
- 重新加载后下颌、上颌共享网格顶点均值相对 TCP 的 world Z 分别为 -91.0642 mm、-53.7782 mm；六个关节角与拍摄记录一致。细节见同目录 `pose_audit.json`。下颌六区均通过输入校验且有效 scan-to-base 矩阵一致。
- 验证范围与限制：当前上颌 PreparedRegion 依赖的 `assets/regional_paths/maxillary/metadata.json` 在本机缺失，因此双颌刚体摆位均已检查，区域几何与集成验收使用下颌；没有补造或跳过上颌来源验证。这是数据可用性限制，没有扩大实现范围。光学帧解释仍为仿真假设，现场 TF、标定绝对精度、修正场景下的完整 IK/避碰/时序/动力学与实机执行均未在本次验收中确认。
