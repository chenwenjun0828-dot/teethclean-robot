# wyh 真机感知结果接入仿真
- 状态：已批准（2026-11-14）
- 问题与目标：已从 `origin/wyh` 获取最新提交 `be9cc2b`，其中包含 D405 手眼/设备外参来源及 `assets/perception/sessions/20261113_220209/` 的上下颌六区域基座坐标结果。当前 `cwj` 工作区有大量未提交规划改动，且现有 MuJoCo 世界坐标不等同于交付包的 `controller_base`。目标是在不覆盖现有改动的前提下，让仿真能够显式选择这批感知结果进行区域规划/回放，并保留单位、坐标系、来源指纹和“不可实机执行”状态。
- 范围：选择性引入 `assets/calibration/ur7e_d405/20261113/`、`assets/perception/sessions/20261113_220209/` 及其必要文档；在 `perception/regions` 或专用适配模块中增加相对 manifest/shared artifact 的安全读取；增加仿真回放入口或参数，使 `controller_base` 点、法向和切向通过 MuJoCo `ur_base` 锚点转换到仿真世界；必要时将牙颌模型放置到同一 `T_world_controller_base @ T_controller_base_scan` 位姿。范围外：不整体合并 `wyh` 的控制/力控重构，不把手眼 YAML 直接当作 `T_base_camera`，不接入真实机器人通信，不改变实机执行许可，不批准尚未确认的上颌区域边界。
- 设计：
  1. 保留当前 `cwj` 工作树和历史输出，使用 `origin/wyh` 的提交内容选择性导入资产；资产清单和相对路径以 `planning_input_manifest.json` 为准。
  2. 读取 session 的 `PreparedRegion`、`points_controller_base_nominal_m`、`region_controller_base_nominal.npz` 和共享 `region_mesh.npz`，校验 manifest 指纹、米制单位、4x4 刚体矩阵、颌/区域匹配；禁止把已在 base 坐标的点再次乘 `T_controller_base_scan_m`。
  3. 在 MuJoCo 场景中读取 `ur_base` 的当前位姿作为 `T_world_controller_base`。区域目标点转换为 `p_world = T_world_controller_base * p_controller_base`；法向和切向只乘旋转。使用同一锚点和 session 的颌扫描变换设置牙颌静态模型位姿，保证规划目标和碰撞网格在同一仿真世界。
  4. 增加显式的 session 参数/CLI 入口，默认仍使用现有仿真自动配准流程；只有指定 session 时才读取真机交付结果。输出报告记录 `source=wyh_session`、session ID、`controller_base`、`T_world_controller_base`、资产哈希、`robot_execution_allowed=false` 和区域边界状态。
  5. 手眼与设备外参只在来源报告中展示和校验身份；由于 `handeye_inventory.json` 明确标记 `handeye_only_not_ready_for_base_export`，仿真入口不重新声称从 YAML 得到经过实机验证的 `T_base_camera`。
- 选择依据：直接切换/合并整个 `wyh` 会覆盖当前大量未提交规划改动并引入与本任务无关的力控重构；直接把 session 的 `world_from_scan` 当 MuJoCo 世界会发生坐标系错误。使用已发布的 base 坐标结果加显式仿真锚点，能够复用已生成的感知几何，同时保持 perception 与 planning 的职责边界。默认入口保持兼容，便于回退和对照现有仿真自动配准。
- 兼容性与风险：新增资产体积较大，需保留 `.gitattributes` 字节一致性和相对目录结构；当前 session 使用标称 UR7e FK、手工关节角和静态时间对齐，不能证明实机绝对精度。MuJoCo `ur_base` 与控制器 Base 的轴定义必须通过单元/几何检查确认；放置牙颌模型后需重新做碰撞和可达性验证。上颌区域边界未批准，默认不把仿真通过结果推广为实机执行结果。
- 实施步骤：
  1. 在当前工作树外生成 `wyh` 资产快照并逐项检查路径、哈希、矩阵方向和单位；将批准范围内的资产复制进仓库。
  2. 实现 session manifest/共享网格的只读加载和校验，补充单元测试覆盖相对路径、重复变换、单位及来源指纹失败。
  3. 实现仿真坐标适配和场景锚定，增加一个下颌 `outer_front` 的显式回放/规划入口，保留现有默认入口。
  4. 更新 perception/planning 文档和示例参数，明确输出是仿真验证、不是实机执行许可。
- 验证与验收：
  - `git diff --check` 通过，当前已有未提交改动不被覆盖。
  - 资产清单、SHA-256、`T_controller_base_scan_m` 组合关系和 session round-trip 加载测试通过；验证 base 点只转换一次。
  - 使用新 session 运行一个区域的 `--dry-run` 和 `--check-mode geometry_only`；确认报告记录坐标链、来源指纹和 `execution_ready=false`/实机执行禁止状态。
  - 在 MuJoCo 中检查同一批点、法向、牙颌网格和 `ur_base` 锚点的相对位姿；必要时运行完整碰撞/动力学仿真，但不连接真实设备。
  - 运行受影响的感知/规划测试及根目录回归入口，并如实记录因图形或依赖缺失而未运行的项目。
- 回退方式：删除本次新增的 session/calibration 资产和适配代码，恢复默认自动配准入口；不回滚或清理本任务之外的工作树修改。已有 `wyh` 远程跟踪分支和本地提交指针保留，便于重新比较。
- 审核记录：2026-11-14，开发者明确批准将本批 `wyh` 真机感知 session 接入当前 `cwj` 分支的 MuJoCo 仿真，用于恢复牙颌相对机械臂的位姿并供后续 planning 使用；批准范围为本方案所列资产导入、manifest/session 加载、`controller_base -> MuJoCo world` 适配、牙颌场景位姿覆盖和仿真验证，不包含真实机器人执行。
