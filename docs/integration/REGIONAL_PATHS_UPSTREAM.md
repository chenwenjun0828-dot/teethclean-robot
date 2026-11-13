# 六区域路径：静态检查与仿真运动

**当前状态：六段完成记录属于旧 `geometry` 几何演示，不能当作牙面避碰验证。**
旧 15° 工作角子段曾通过，但使用逐点增大的间隙，明显改变牙龈线形状，该策略已取消。
当前区分指定工作尖端与非工作部分，允许尖端接触目标路径附近的下颌，工作角 8°–45°。
区域 2 尚未通过整段验证；历史非接触结果不代表接触运动已经通过，不能直接作为接触洁治轨迹使用。

此入口用于在当前 MuJoCo 牙模上检查已制作的六区域位置曲线，不做提取、IK 或机械臂运动。
支持已知牙模位置预览，以及本次虚拟 RGB-D 自动配准后的预览。

## 运行

在项目根目录使用本机环境：

```bash
OMP_NUM_THREADS=1 .venv/bin/python -m pipeline.path_viewer
```

上述默认 `--placement known` 使用已知牙模位置。要检查本次相机自动配准效果：

```bash
OMP_NUM_THREADS=1 .venv/bin/python -m pipeline.path_viewer --placement automatic
```

自动模式先复用 `capture_rgbd` / `save_capture` 保存新的 RGB、深度、相机点云，
再复用 `register_capture(..., registration_method='automatic')` 做 FPFH、RANSAC 和 ICP。
不重新提取或修改六区域线，不打开人工选点窗口。
彩色粗线为本次估计位置，白色细线为已知牙模位置下的参考；按 `R` 显示/隐藏白线。
参考只用于求解后的误差评价和对照，不参与配准输入、初值或结果修正。
配准失败会保存失败报告并停止，不自动回退到已知位置；评价超限仍显示估计结果并明确提示。

窗口支持鼠标旋转和缩放。按 `0` 显示全部，按 `1`–`6` 单独显示对应区域：

| 编号 | 区域 | 颜色 |
| --- | --- | --- |
| 1 | 外侧左后牙 `outer_left` | 橙色 |
| 2 | 外侧前牙 `outer_front` | 黄色 |
| 3 | 外侧右后牙 `outer_right` | 粉色 |
| 4 | 内侧右后牙 `inner_right` | 青色 |
| 5 | 内侧前牙 `inner_front` | 绿色 |
| 6 | 内侧左后牙 `inner_left` | 蓝色 |

绿色球为起点，红色球为终点，短箭头指示输入点序方向。数字标注在起点。
左右沿用原参考点定义，不代表旋转后屏幕的左右。区域之间不画连接。
默认聚焦下颌并使用半透明显示，使背面路径也可见；路径坐标没有为显示而偏移。
预览模型的透明度调整仅作用于本进程，不改 XML 或资产。

只保存图片和报告：

```bash
OMP_NUM_THREADS=1 .venv/bin/python -m pipeline.path_viewer --headless
```

可加 `--region outer_front` 启动时选定区域；`--context` 显示完整场景，其他物体可能遮挡路径。
每次运行创建 `outputs/regional_preview/<时间戳>/`，其中包含：

- `regional_preview.png`：MuJoCo 实际渲染的初始视图。
- `preview_report.json`：数据来源、模型哈希、变换、区域点数和采样点到口扫表面的最大距离。
- `regional_paths_world_m.csv`：六区域世界坐标位置，保留 `chain_order`、`region`、`point_order`，不同区域不可直接连接。

自动模式还保存 `capture/`、`registration/`、`registration_config.json` 和
`reference_paths_world_m.csv`。`regional_reference_scan_mm.csv` 是适配现有配准服务的口扫参考点文件，
没有跨段连线含义，也不是配准对应点或算法初值。
评价报告包含整体/逐区域对应点 RMS 和最大误差、变换平移和旋转误差。
`alignment_passed` 使用现有配置的仿真评价阈值：平移 ≤0.5 mm、旋转 ≤0.5°、路径 RMS ≤0.5 mm、最大 ≤1 mm。
这些阈值仅用于当前仿真检查，不是临床或真实设备精度标准。

## 数据与坐标

`assets/regional_paths/mandibular/` 保存六条主线、688 点的冻结版本，来源为
`registration_robust_20261020/stitched_regional_paths_v1`。
输入模型 `work/mandibular_input.ply` 与本项目下颌 PLY 的 SHA-256 完全相同。

转换链为：旧 D405 相机 mm → 口扫 mm → 当前 MuJoCo 世界 m。
第一步取旧 `T_camera_scan.txt` 的逆变换，恢复口扫局部路径。
第二步按运行模式选择：

- `known`：使用场景内 `dental_mandibular` 的实际刚体放置位置，只做已知位置预览。
- `automatic`：使用本次估计的口扫→相机变换和采集时相机→世界变换。

旧 D405 变换只用于还原路径，不作为新配准初值。`automatic` 的已知牙模位置只用于独立仿真评价。
当前虚拟相机默认用 `ideal_segmentation` 选取牙模像素，比真实相机更理想化。
报告中的 `simulation_alignment_evaluated` 表示已做仿真对应点评价；
`registration_accuracy_validated=False` 表示没有据此认定真实相机配准精度已验证。
采样点到原口扫三角表面的距离超过 0.35 mm 时拒绝预览；这个阈值是本次数据检查参数。
表面距离小不等于牙龈解剖位置正确，仍需人工检查分区和补线。

运行时只读取路径 CSV、变换和 `metadata.json`，校验模型和输入文件哈希。
`stitch_report.json` 原样留作溯源，内部历史 Windows 路径不会被运行时读取或解析。
无移动硬盘依赖，不自动寻找“最新结果”。

## 代码边界

```text
冻结资产 → perception/regional_paths.py → pipeline/path_viewer.py → MuJoCo 静态展示

自动模式：虚拟 RGB-D → perception/regional_registration.py
                       → 复用现有 register_capture → 估计位置下的六区域展示

原入口 pipeline.run → 感知/配准 → 规划 → 控制执行（保持原流程）
```

- `perception/regional_paths.py` 只处理校验、路径分组和坐标转换，保留原顺序和方向。
- `perception/regional_registration.py` 组织本次采集、配置冻结和现有配准服务调用，不实现另一套配准算法。
- `pipeline/path_viewer.py` 负责表面距离检查、窗口、按键、渲染和预览报告。
- 复用 `pipeline/simulation.py` 的场景创建与画线接口。
- 不导入旧选点、编辑、裁剪、平滑、补线、配准实验脚本。
- 静态入口不调用规划控制；运动入口通过共用 `tooth_aligned_targets` 和 `run_path_tracking` 接入原有能力，`RegisteredPaths` 的原感知读取逻辑保留。

静态入口保持只预览。运动使用下面的独立入口，复用现有姿态、IK、预检与控制。
冻结资产的 `robot_execution_allowed=False` 保持不变；新的运动入口仅用于 MuJoCo 仿真，没有硬件执行授权。

## 六段仿真运动

```bash
OMP_NUM_THREADS=1 .venv/bin/python -m pipeline.regional_run \
  --robot-offset-mm -160 -120 100 --playback-speed 10
```

完整顺序为外侧左、外侧前、外侧右、内侧右、内侧前、内侧左。
每段包含接近、进入、沿线、撤离；下一段从上一段实际撤离后的机器人状态继续。
不重置实时关节状态或仿真时间，不跨区域直接连牙龈线端点。
先对所有选定区域及段间接近过程完成 IK 和关节扫掠预检，再开始任何运动；失败会停止并保存区域和原因。

**基座位置是本次验证的必要条件。** 原安装位置下外侧右区出现不可达；仅靠更换 IK 初值无法解决。
上述命令在仿真世界坐标中将 `robot_mount` 偏移 `(-160, -120, +100) mm`，在采集与规划前设置并写入报告。
原 XML、牙模放置、路径和工具标定不改。省略参数时默认偏移为零，不保证六区域预检通过。
代码会尝试有限组确定性的 IK 接近种子，但每个候选都使用原求解器和完整扫掠检查，不忽略不可达或碰撞。

`--playback-speed 10` 仅减少窗口播放等待时间；物理步长、指令时序与关节限速不变，实际播放倍率受计算速度限制。
沿用默认 `geometry` 模式：牙模凸包碰撞关闭，其他机器人接触检查仍启用；六段完成不等于接触洁治已验证。
窗口用六种颜色显示来源路径，青色为当前目标、橙色为实际轨迹；左上角显示区域、阶段及该段仿真时间。
显示时牙模透明度调整发生在采集和预检之后，不参与配准，也不改变碰撞设置。

仅预检、单区域验证和无窗口完整运动：

```bash
OMP_NUM_THREADS=1 .venv/bin/python -m pipeline.regional_run --plan-only --headless --robot-offset-mm -160 -120 100
OMP_NUM_THREADS=1 .venv/bin/python -m pipeline.regional_run --region outer_left --headless --robot-offset-mm -160 -120 100
OMP_NUM_THREADS=1 .venv/bin/python -m pipeline.regional_run --headless --record-gif --robot-offset-mm -160 -120 100
```

输出为 `outputs/regional_tracking/<时间戳>/`：

- `regional_report.json`：基座偏移、自动配准评价、逐区域预检与运动状态。
- `capture/`、`registration/`：本次虚拟相机采集和估计变换，六区域共用。
- `preflight/<区域>/`：原始/筛选目标、姿态筛选原因和完整预检关节路点。
- `tracking/<区域>/`：共用执行器输出的位置/姿态误差、局部时间记录、预览和可选 GIF。

本机完整验证结果：`outputs/regional_tracking/20261109_162148_951050/`，六段均为 `completed`。
各段最大跟踪位置误差约 0.072、0.209、0.095、0.161、0.297、0.226 mm。
这些是 TCP 相对已规划目标的跟踪误差，与配准误差、解剖提取误差分别统计。

运动代码职责：

- `planning/trajectory.py:tooth_aligned_targets`：共用的有序世界路径到牙齿/TCP 目标计算；原注册路径入口仍可使用。
- `pipeline/regional_run.py`：一次采集配准、区域顺序、有限 IK 分支尝试、整体预检与执行调度。
- `pipeline/tracking.py:run_path_tracking`：共用目标导出、控制执行、相对计时、窗口和误差报告；原 `run_tracking` 是原感知流程的适配入口。
- `control/position.py`、`planning/ik.py` 和碰撞检查算法继续复用，没有另写控制器或 IK。

## 测试

```bash
OMP_NUM_THREADS=1 .venv/bin/python -m unittest tests.test_regional_paths tests.test_regional_registration -v
OMP_NUM_THREADS=1 .venv/bin/python -m unittest tests.test_regional_motion tests.test_tcp_tooth_alignment tests.test_planning_control -v
```

测试覆盖坐标往返与单位转换、输入哈希、区域和点序，以及绘制时不跨区域连接；
另检查真实虚拟 RGB-D 自动配准、估计变换组合、对照误差、失败不回退，以及整个预览不调用机器人步进。
运动测试覆盖非零仿真时刻下两段连续执行、不重置实时状态，以及任一区域预检失败时不启动任何段的控制。

## 固定沿线位置、8°–45° 工作角：区域 2 预检

### 当前：指定尖端接触

```bash
OMP_NUM_THREADS=1 .venv/bin/python -m pipeline.regional_run \
  --region outer_front --tip-contact --clearance-mm 0 --work-angle-range-deg 8 45 \
  --jaw-opening-mm 30 --robot-offset-mm -160 -120 100 --headless --record-gif
```

`--tip-contact` 当前只支持一个完整下颌区域，不裁子段。规划探针、机器人预检和每个物理步
统一使用 `planning/contact_policy.py`，不关闭尖端碰撞或接触求解：

- 工作尖端只包括所有顶点位于 TCP 半径 1 mm 内的三角形，边界混合三角形仍归非工作部分。
- 只允许尖端接触选定路径 1 mm 带内的下颌；对颌实际接触仍禁止。
- 尖端凸覆盖体压入容差为 0.1 mm；其他工具部位要求正间距至少 0.02 mm，禁止穿入。
- 下颌路径附近细网格为 0.15 mm、尖端分组 0.1 mm、非工作部位近尖端 5 mm 内分组为 0.15 mm、几何填充厚度 0.005 mm，
  用于减少凸包过度膨胀；这些覆盖仍然保守，并非真实网格接触的精确证明。
- 正距离预警保留用于检查，通过 `gap` 避免在尚未接触时施加预警距离对应的接触力。
- 这些数值是仿真检查参数，不能解释为临床允许压入或力控制设定。

原始 115 点逐点向**源口扫网格**做最近表面投影，上限 0.35 mm；投影与碰撞候选无关，
不使用仿真已知位置修正配准。区域 2 最大修正 0.1893 mm、RMS 0.0340 mm，
长度从 55.4362 mm 变为 55.5470 mm。原线和落面目标分别保存，不改变原始数据或连线顺序。

姿态在局部牙面坐标系内分别插值工作角、切向朝向和滚转，避免直接旋转插值跑出角度范围。
直接连接发生干涉时，还会检查在同一空间线段内临时提高工作角到 44.5° 的过渡，
端点保持原角度，并检查整个过渡；不通过时仍停止，不放行杆身接触。
增加中间样本（空间步长不大于 0.1 mm、姿态参数步长不大于 3°），仍保留全部原始采样点；
记录 `original_sample_index`、`source_segment_index`、`source_segment_fraction`，辅助点不冒充原始点。
默认保留 8 条候选序列；`--pose-search-width 24` 可扩大有限搜索，不能视为完备求解器。

执行报告将分别统计允许尖端接触的采样比例、凸覆盖体最大压入和最大法向接触力，
不会再把允许接触写成“机器人零接触”。只有实际角度、其他碰撞、轨迹误差均通过，
且沿线阶段接触采样比例至少 90% 才允许完成状态；尚未加入力闭环控制。

完整区域尝试 `outputs/regional_tracking/20261109_191807_499257/`：原始点 0–84 的工具姿态序列
通过局部几何检查，连接至原始点 85 时非工作部位干涉被拦截；机器人 IK、实际运动尚未执行。
这不是已通过前 85 点的机器人运动，也不是完整区域无解的证明。
`physical_nonworking_mesh_check.json` 是该被拒绝候选的原始网格抽样复核，
`blocked_contact_pose.png` 是静态诊断图，不能当成执行成功截图。
另外的局部连接复核（含扩大候选、近尖端网格细化）只验证该连接，不代表完整区域通过。
最新 `local_connector_refined_check.json`：保留 24 条序列、每个端点最多 108 个无禁止接触候选，
2592 条连接（含工作角抬升过渡）仍未通过。这个有限搜索不是全局无解证明。
`reference_normal_check.json`：该段长 0.461 mm，最近牙齿参考法向变化 149.688°；
需要检查跨牙缝时的牙面关联，不能靠扩大尖端许可区或压入容差绕过。

### 历史：固定 0.5 mm 非接触偏移

```bash
OMP_NUM_THREADS=1 .venv/bin/python -m pipeline.regional_run \
  --region outer_front --work-angle-range-deg 8 45 --clearance-mm 0.5 \
  --jaw-opening-mm 30 --robot-offset-mm -160 -120 100 --headless --record-gif
```

冻结原始 115 点及顺序，位置仅施加恒定 0.5 mm 法向偏移，不加大间隙、不删点。
角度优先接近 15°，候选倾角约 5° 步进并包含范围端点，切向方向/滚转按 30° 步进。
每种倾角保留最多 4 个无碰撞姿态，宽度 8 的序列搜索检查相邻姿态插值角度及工具碰撞。
这是有限候选搜索，不保证找出所有可行姿态；随后仍必须通过原有完整机器人 IK、关节扫掠与时序检查。
运行中实际角度超出 8°–45° 会停止，15° 仅作为偏好而非固定验收角。

`outputs/regional_tracking/20261109_174652_987924/`：自动配准通过，起点测试 1296 个候选仍未满足安全裕量，
机器人未运动，其余点尚未检查。保存了原线/固定目标线 CSV 和 `pose_search_diagnostic.json`。
诊断保存的一个无凸包重叠候选，最近保守凸包距离约 0.026 mm，仍触发正距离预警，
说明不能把本次拒绝直接描述为真实牙面穿透，也不能据此断言区域无解。
失败时不会打开执行窗口或生成运动 GIF。

## 历史 15° 子段记录（逐点扩大间隙策略已取消）

以下命令和数字用于说明旧结果；当前固定间隙策略不保证复现旧通过状态。

```bash
OMP_NUM_THREADS=1 .venv/bin/python -m pipeline.regional_run \
  --region outer_left --point-range 69 84 \
  --work-angle-deg 15 --tool-roll-deg -90 --clearance-mm 1.5 \
  --jaw-opening-mm 30 --robot-offset-mm -160 -120 100
```

`--point-range` 是零基 `[START, STOP)` 原始区域点范围，以上仅包含 69–83，不代表整个外侧左区。
完成时总体状态为 `completed_subset`，避免和完整区域完成混淆。
加 `--headless --record-gif` 可保存无窗口验证结果和动画；加 `--plan-only` 只做预检。

工作角定义为 **TCP +X（尖端切向）与局部牙面切平面的夹角**。15° 是用户选定的仿真值。
TCP 原点经网格核验位于工具尖端，最近顶点距离约 0.000002 mm。
新模式不使用旧的 45° 姿态丢弃/替换逻辑；绕法线的切向方向及绕尖端的滚转可在候选中变化，
但各目标点保持 15°，点间插值相对参考牙面坐标系的角度误差需小于 2°。
运行中另外统计实际工作角，偏差超过 3° 时拒绝完成状态。
参考是原路径牙面坐标系及其插值，不是实时测量的真实牙面。

旧实现把 `--clearance-mm` 作为名义最小离面距离，候选可增加 0、0.5、1.5、3、4、5 mm。
当前实现已取消这种增加：`--clearance-mm` 为所有点相同的固定法向偏移。
每点实际偏移、方向和原始点号记录在 `work_pose.pose_selection`。
工具滚转会改变，不能把 `--tool-roll-deg` 当作所有点最终固定滚转角。
这条离面验证轨迹不是接触轨迹，TCP 目标长度也会因法线及间隙变化而不同于原牙龈线长度。

**场景条件必须保留：**基座偏移 `(-160,-120,+100) mm`；上颌沿口扫 +Y 平移 30 mm，
用于构造开口的仿真工作空间。这是平移开口近似，不是颞下颌关节运动模型。
两项均在采集和规划之前设置、写入报告；不修改原 XML，也不删除上颌碰撞。
新模式显示下颌实体和半透明上颌，透明度只影响显示。

### 牙缝跨越规则

牙龈线在相邻牙之间会扎进邻面沟槽，最近的外侧牙面因此换到邻牙上，参考牙面法向在缝处
转过 100°–150°（`outer_front` 实测 6 处，边索引 9、32、50、65、85、102）。
缝处不存在唯一正确的局部参考牙面，所以**缝区不做 8°–45° 工作角锥约束**，
改用一次抬起跨越：沿两侧外法向均值方向抬起、横移越过、再落回原牙面。

- 抬起量**自适应**：按 0 → 1.0 → 1.5 → 2.0 → 3.0 → 4.0 mm 取第一个使整段过渡无碰撞的最小值。
  实测多数缝原地即可跨越，抬起只是给姿态过渡提供解的手段。
- `--seam-angle-deg`（默认 60）设定判定牙缝的参考系转角阈值；`--seam-lift-mm`（默认 0）限制抬起量上限。
- 缝区姿态走两端已验证候选之间的 SO(3) 测地线，**每个采样都做避碰检查**；
  非工作部位避碰、尖端接触规则在缝区同样生效。
- 缝区样本在 `work_pose.pose_selection` 中标记 `is_seam`、`seam_lift_mm`、`seam_edge_index`；
  `work_pose.seams` 汇总每条缝的实际抬起量；`preflight/<区域>/seams.json` 单独落盘。
- 运行时统计把缝区样本与面内样本分开：缝区不计入工作角锥判定，也不计入 90% 尖端接触覆盖率的分母，
  另报 `seam_count`、`seam_samples`、`seam_actual_angle_min/max_deg`。

抬起的工具杆长约 138 mm，缝处绝对姿态摆动的理论下限由两侧 15° 工作角决定（约 74°–90°），
这是几何约束而非算法限制，靠保守凸包碰撞检查拦截；失败时报告会指出是哪条缝，不做静默放宽。

### 工作角锥为偏好项、机器人可达性为一等约束

姿态搜索原先只做几何避碰，不检查机器人腕部能否真正保持该姿态。结果是：位置全部可达
（全局优化器给出 0.0000 mm 残差），但所选工作姿态在该牙位转不过来，全局最优位置残差
高达 13.9–23.5 mm，整段预检在整机 IK 阶段失败。

现在有两条规则：

- **可达性是一等约束。** 候选姿态在进入 beam 之前先做一次逆运动学筛查；不可达的候选直接剔除，
  并在 `search_diagnostic.points[].unreachable_candidates_rejected` 中计数。
  筛查是提示而非证明：某点所有候选都被剔除时，该点会去掉筛查重试，绝不因为筛查误判而丢掉可解姿态。
- **工作角锥是偏好项。** 锥内没有任何可行姿态时，搜索自动放宽倾角（锥外 ±15°，5° 步进）
  继续找解；**避碰始终是硬约束**，不会被放宽。
  实际生效的角度范围记录在 `work_pose.effective_angle_range_deg`，
  放宽过的点数记录在 `work_pose.widened_angle_cone_points`，
  运行时按生效范围校验实际工作角，并另报 `actual_work_angle_effective_range_deg`。

逆运动学求解器（`planning/ik.py`）本身也做了加固：种子续解失败时从其他分支重启，
仍失败时用最小二乘精修脱困。实测整机 IK 失败样本从 113 处降到 66 处，
剩余部分经全局优化确认是几何上真的不可达。

### 当前阻塞点：关节扫掠与单点避碰结论不一致

`outer_front` 的 `outer_front` 段目前仍未通过整机预检，失败发生在**关节扫掠**阶段，
而非姿态搜索或逆运动学阶段。已定位的事实：

- 姿态搜索 115/115 全部通过，6 条牙缝全部 0 mm 原地跨越成功。
- 整机逆运动学不再报 "IK target unreachable"：求解器加固后该类失败已全部消除。
- 失败的关节区间有两类，穿深都很浅：
  - 工作尖端对下颌：`working tip exceeds penetration tolerance`，穿深约 **0.10 mm**（容差恰为 0.10 mm）。
  - 非工作部位（近尖端）对下颌：`nonworking tool/robot contact`，间隙约 **0.019 mm**（容差 0.02 mm）。
- 关键证据：**目标姿态本身不碰撞**。把机器人放到解出的关节位姿上，单点接触检查与扫掠一致都判为无接触；
  只有两点之间的**关节插值路径**会切过牙面。也就是说，问题出在离散关节插值的曲率，
  不在端点姿态。
- 探针与扫掠用的是同一套接触策略和同一套碰撞体，结论并不矛盾：
  探针只检查端点姿态，扫掠检查整段插值。

下一步的方向（尚未实施）：

1. 在姿态搜索阶段就检查**相邻姿态间的插值**，而不是只检查单点姿态；
   现在的搜索是逐点独立的，缝隙处两段插值会切牙。
2. 或在规划时加入关节插值的曲率约束，限制相邻关节解之间的跃变。
3. 收紧接触容差不是可行解：实测把穿深容差收到 0.05 mm、非工作间隙提到 0.30 mm，
   1001 个目标姿态里会有 948 个被判为碰撞，说明这些间隙本来就是贴着牙面走的，
   必须靠插值路径来解，不能靠放宽或收紧单一容差。

### 碰撞范围与限制

`--work-angle-deg` 自动启用 `surface` 模式：

- 对上下颌及整把洁治器的原网格三角形作局部凸覆盖，细化路径附近牙面和尖端到 1 mm 网格。
- 只有生成替代碰撞体后，才关闭原来的整牙弓/整工具视觉网格凸包碰撞；原机器人碰撞体及传感器凸包保留。
- 覆盖体包含原三角形并外扩 0.03 mm，可能保守地填充部分自由空间，不是精确解剖分割或精确凸分解。
- 规划关节扫掠步长细化到 0.0005 rad；新时序拟合后再次密集检查碰撞及 TCP 偏差。
- 执行每个物理子步仍调用原有接触检查。碰撞体各有 0.3 mm 预警 margin，进入预警带也会停止，不必等到实际穿透。
- `--planning-margin-mm` 可增加规划余量，范围 0.3–1 mm；运行时 margin 固定 0.3 mm，不因提速减少。

这些仍是离散检查和近似碰撞体，不是连续碰撞的数学证明，不包含力控或真实设备安全认证。
全段未找到候选姿态或扫掠失败时保留失败报告，不跳过失败点继续执行。

### 速度与已验证结果

`planning/timing.py` 按局部区间拟合单调三次时钟及关节曲线，端点速度为零，
解析检查每段速度和加速度峰值，并用原 IK 路点作几何复核，不再让一个最坏区间拖慢整条曲线。
默认命令关节速度上限 0.3 rad/s、加速度上限 1 rad/s²、TCP 速度上限 4 mm/s、目标角速度上限 60°/s。
`--timing global` 可作对比；旧几何演示继续默认全局时序。

已复验结果：`outputs/regional_tracking/20261109_171759_137952/`。
子段 15 个点，离面 TCP 目标约 14.77 mm；沿线时间约 28.53 s，对应全局时序估计约 31.94 s，
总运动约 59.51 s。最大位置跟踪误差约 0.127 mm；实际工作角约 14.38°–15.07°；运行未检测到接触。
这里只能确认小幅提速和子段效果，不能宣称完整六段的工作角/避碰/提速已经验证。

`--joint-speed-rad-s`、`--joint-accel-rad-s2` 是显式仿真实验参数。
提高到 0.6 rad/s 和 2 rad/s² 的候选虽通过静态预检，却在实际执行触发接触预警而停止，
因此不作为推荐配置。更大的规划余量也可能令当前路径不可行，不能为通过而取消检查。
失败时 `tracking_report.json` 的 `failure_contacts` 保存碰撞体、距离及是否穿透。

原外侧左区初版逐点审计见 `outputs/regional_tracking/20261109_170734_983223/pointwise_pose_audit.json`。
该历史候选网格在 100 个点中有 18 点未找到可行工具姿态；它只检查独立姿态，不证明 IK、段间连接或所有方向都不可行。
需要先复核这些牙间路径、法线和工具空间，再扩展到完整区域。

代码职责：`planning/tool_pose.py` 生成受角度约束的候选；`pipeline/collision_scene.py` 构建独立碰撞覆盖及工具姿态探针；
`planning/timing.py` 处理局部时序与重新检查。现有 IK、执行器控制和数据记录继续复用。

### 2026-11-10 阶段小结

已完成并验证：

- **牙缝跨越**：缝区改用两端候选之间的 SO(3) 测地线过渡，不做工作角锥约束；
  `outer_front` 的 6 条缝（边 9、32、50、65、85、102）全部以 **0 mm 抬起**原地跨越成功。
  抬起阶梯（0→1→1.5→2→3→4 mm）保留为后备手段。
- **姿态搜索**：115/115 点全部通过。修复前该阶段在第 85 点就失败。
- **整机逆运动学**：`planning/ik.py` 加入分支重启与最小二乘精修后，
  "IK target unreachable" 类失败全部消除（此前 113 处）。
- **可达性进入搜索**：姿态候选先做逆运动学筛查（`planning/ik.py` 的 `reachable`）。
- **工作角锥降级为偏好项**：锥内无解时自动放宽倾角，避碰始终是硬约束。
  实际生效范围记于 `work_pose.effective_angle_range_deg`。

尚未解决：

- **关节扫掠预检仍失败**。根因已定位为：目标姿态本身不碰撞，但相邻姿态间的
  离散关节插值会切过牙面（尖端穿深约 0.10 mm，非工作部位间隙约 0.019 mm）。
  现有姿态搜索是逐点独立的，不检查段间插值。详见上文「当前阻塞点」。

测试状态：`tests/test_tool_pose.py` 17 项全过；全量 107 项中 4 失败 1 错误，
均为既有问题（2 项 GUI 在无显示环境下段错误、3 项配准精度不达标，
已在 `outputs/real_sim_comparison_20261109/实验说明.md` 记录），与本轮改动无关。

```bash
OMP_NUM_THREADS=1 .venv/bin/python -m unittest tests.test_tool_pose tests.test_surface_collision tests.test_regional_motion -v
```
