# 2026-11-11 代码整合与模块边界

> 历史验证记录：本文保留当时的代码、命令和输出证据。旧三分区提取线分支、兼容别名和旧参数已于 2026-11-11 删除，文中结果不代表当前六区域流程的覆盖率或端到端验收。当前入口与验证见 [六区域清理记录](2026-11-11-six-region-cleanup.md)。

本次合并以当前工作区为主，整合 `teethclean_robot_1110` 交付包的六区域源路径。
交付说明原文保存在本目录的 `2026-11-09_10功能更新说明.md` 和
`REGIONAL_PATHS_UPSTREAM.md`；其中旧 planning 和模型路径是历史说明，不是当前入口。

## 当前流程

```text
口扫 / 六区域源线 + 本次 RGB-D 配准
  -> perception: 牙位归属、分区、牙面法向、拟合与区内连接
  -> PreparedRegion / perception_region.json
  -> planning: 区域固定姿态 -> ±5° 姿态微调 -> 连续 IK
  -> 固定关节样条 + TOPPRA 时序 + 点间碰撞审计 + 动力学验证
  -> control: 已验收轨迹的仿真执行
```

`perception/regions/` 是几何处理的唯一实现位置，不依赖 planning 或 control。
旧的 `planning/path_reconstruction/` 和 `planning/infrastructure/geometry.py` 已删除，
当前规划只保留姿态搜索、IK、碰撞审计和时序生成。
交互式姿态滑块现已恢复为 `planning.visualization.pose_angles`，使用当前六区域参考和三角度定义，
详见 [查看器说明](../../planning/visualization/README.md)。
`planning.pipeline.run()` 接受 `PreparedRegion` 或其 JSON 路径，传入原始 `Inputs` 会报错。
独立 planning CLI 只负责加载现有感知结果，然后调用规划 API。

planning 可以采样既有曲线、应用指定的法向离面距离和生成姿态，但不会重新拟合、连接、
按可达性截短或丢弃感知曲线。失败时导出诊断并停止。
`sweep_max_trim_mm` 保留命令兼容；本次在 perception 使用 `sweep_trim_mm` 一次性确定几何，
planning 不再扩大裁剪来寻找可行路径。修改感知裁剪需要重新导出感知结果。

## 六区域

`assets/regional_paths/mandibular/` 保存交付的 688 点、旧相机变换和元数据。
加载时校验源文件与口扫模型 SHA-256。当前英文路径的下颌模型与交付模型完全一致。
旧 `T_camera_scan` 只用于将历史相机坐标恢复为口扫坐标；本次世界坐标由新配准决定。

| 区域 | 含义 | 兼容别名 |
| --- | --- | --- |
| `outer_left` | 外侧左后牙 | `left` |
| `outer_front` | 外侧前牙 | `middle` |
| `outer_right` | 外侧右后牙 | `right` |
| `inner_right` | 内侧右后牙 | |
| `inner_front` | 内侧前牙 | |
| `inner_left` | 内侧左后牙 | |

内外侧法向分别从该侧牙面支撑片拟合。分割缺失附近的最近牙位只作姿态支撑，
超出归属距离的点数会记录；不是已认证的 FDI 编号或解剖分割。
源线表面距离必须不超过 0.35 mm。连续区域使用一条受源走廊约束的 B 样条，
牙位切换仅划分参考区间并平滑法向，不在区内重新裁剪或补线。
裁剪仅作用于整个区域的起终点。不同区域始终独立，不拼成一条清扫线。

默认 `--path-source auto` 对同哈希下颌采用六区域；其他口扫采用已提取拓扑。
`--path-source extracted` 强制使用原提取线；`--regional-dataset` 可指定冻结数据目录。
完整 pipeline 会在 perception 导出所有分区，再按 `--region` 选择一段规划。
本次不将六区历史几何演示解释为六区均已通过新规划器的运动验收。

## 接口和命令

```powershell
# 只做感知几何：六区域完整导出
.\.venv\Scripts\python.exe -m perception.regions --out outputs/perception/my_regions

# 只消费某一份感知输出；规划不再重构牙龈线
.\.venv\Scripts\python.exe -m planning `
  --perception-result outputs/perception/my_regions/outer_front `
  --pose-angles-deg 15 0 0 --out outputs/planning/my_front

# 完整流程：自动配准、感知分区、选区规划与仿真验证
.\.venv\Scripts\python.exe -m pipeline.run --region outer_front `
  --registration-method automatic --headless --output outputs/pipeline/my_front

# 回放本轮已通过的原提取线中区 90 秒验证结果
.\.venv\Scripts\python.exe -m planning.view `
  --plan outputs/integration_1111/pipeline_registered/tracking --speed 1

# 回放本轮通过的新六区域前区（已完成碰撞与动力学验证）
.\.venv\Scripts\python.exe -m planning.view `
  --plan outputs/integration_1111/six_front_full --speed 1
```

每个感知目录包含 `perception_region.json`、`continuous_gingival_line.json/.npz`、
`geometry.json/.npz`、`tooth_planes.json`、`tooth_lookup.npz` 和 `region_mesh.npz`。
保存注册变换时另有世界坐标 CSV 和 `world_from_scan_m.npy`。
总索引为 `perception_regions.json`。JSON 记录坐标单位、源哈希与产物哈希，
源或产物改变时拒绝复用。

新规划默认 `simple`：整区代表点建立固定参考系，三个角由
`--pose-angles-deg ANGLE AZIMUTH FLIP` 给出。先验证零修正，再按 2.5° 网格搜索整区修正，
最后尝试局部平滑修正。每个角相对原始基准始终在 ±5° 内。
候选点和相邻关节边均检查；实际样条和动力学仍独立验收。
`continuous` 仍可显式选择。旧存档缺少模式时按历史 `continuous` 解释。

## 配准合并

人工配准的 `landmark_picker.py`、`landmark_selection.py`、`registration_math.py`
及自动配准核心与交付包一致。保留左右同窗、逐对选点、撤销、清空、完成和取消。
当前工作区的 `algorithms.py` 与 `registration_workflow.py` 另有真实点云、标定检查和
计时支持，整合后继续使用。真实相机入口仍为 `perception.real`，见 `perception/REAL_CAMERA.md`。

交付包的六区域预览入口也已适配当前资产目录：

```powershell
.\.venv\Scripts\python.exe -m pipeline.path_viewer --headless --output outputs/regional_preview/my_preview
.\.venv\Scripts\python.exe -m pipeline.path_viewer --placement automatic
```

预览只显示源线，不执行机器人；自动模式复用当前配准服务。

## 实测验证

1. 六区域 688 个源点全部通过模型哈希与表面距离检查，最大 0.2633 mm；
   六份连续区域结果位于 `outputs/integration_1111/perception_verified/`。
2. 原提取线中区使用 `[6.37,13.26,-0.4]`、离面 1.8 mm、张口 20°、
   每端裁剪 2.5 mm，规划 1148 个姿态节点及总运动 90 秒。
   基准固定姿态通过，修正 `[0,0,0]`；点间运动审计和动力学通过，`execution_ready=true`。
   静态审计最小间隙 0.3251 mm；动力学最小间隙 0.3173 mm。
   结果位于 `outputs/integration_1111/planning_extracted/`。
3. 固定姿态碰撞后的统一修正、局部修正、±5° 硬边界、不可达失败、无碰撞模式不可执行、
   文件复读及篡改拒绝均有回归测试。
4. 完整 pipeline 使用 `config/pipeline_middle_surface15_90s.json`，
   从提取、自动配准到 90 秒运动全部通过；ICP RMSE 0.1485 mm，
   动力学最小间隙 0.3174 mm，执行加载器复查通过。
   结果位于 `outputs/integration_1111/pipeline_registered/`。
5. 新 `outer_front` 使用上述同一组角度、离面 1.8 mm、区域起终点各裁剪 1 mm、
   姿态间距 0.25 mm，在仿真模型变换下通过完整运动验证。
   保留 430 个姿态节点、432 个总关节路点，总运动约 66.6793 秒，
   源覆盖率 96.3361%；固定姿态修正 `[0,0,0]`。
   静态审计最小间隙 0.1735 mm，动力学最小间隙 0.1599 mm，
   `execution_ready=true`，执行加载器复查通过。
   结果位于 `outputs/integration_1111/six_front_full/`。
6. 六区配准预览 `outputs/integration_1111/regional_registered_preview/` 通过，
   ICP RMSE 0.1624 mm，源路径配准 RMS 0.0498 mm；该预览本身不执行机器人。
7. `planning` 的 136 项测试与 `tests` 的 222 项测试全部通过。
   针对新曲线暴露的短时间区间浮点误差，改用局部位移构造五次时钟，
   保持关节曲线、输入节点和物理约束不变，并加入 C2 连续性回归。

以上 90 秒案例使用原提取线；新六区域中仅 `outer_front` 完成了本次运动验收，
其余五区的位置、长度和法向不同，仍需独立选角与验收。
新前区直接指定 90 秒时，当前加速限速配置下 ECOS 返回 `FailUncontrollable`，
已导出失败诊断至 `six_front_motion/`，未绕过时序约束或生成可执行轨迹；
上述通过的 66.6793 秒结果使用默认物理限速和自动时长。
当前入口为 MuJoCo 仿真，不将几何或仿真通过等同于实机接触清洁验证。

本轮上述运动通过案例均为名义离面轨迹。随后补做的连续区域接触与 1 N 恒力试验，
及凸包与原口扫表面不一致导致的未通过结论，见
[接触与恒力验证](2026-11-11-contact-force-validation.md)。
