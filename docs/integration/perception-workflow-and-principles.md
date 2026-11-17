# 真机感知：操作流程与计算原理

本文说明 D405 拍摄到上下颌六区规划输入的完整过程。具体交付文件、矩阵数值和本批记录见[感知交接说明](2026-11-13-perception-handoff.md)。本文对应当前代码，不需要运行 ROS 或 MuJoCo；ROS 帧名仅用于明确坐标约定。

## 1. 整体流程与输入输出

```text
同一时刻的 RGB-D 帧 + 机器人关节角
           │
实时预览 → S 冻结 → ROI → 相机局部点云
                               │ 与对应颌的口扫选同名点
                        刚体初配准 → ICP 精配准
                               │ T_depth_optical_scan
关节角 → 正运动学 ──────────────┤
手眼标定 + D405 内部外参 ───────┤
                               ↓
                      口扫 → 机器人 Base
                               │
口扫牙龈源线 + 牙齿分割 → 六区曲线、法向和切向
                               │
                     区域导出 → 核查 → 发布 assets
                               ↓
                      下游规划读取正式清单
```

输入包括：对应上下颌口扫、D405 数据、同一拍摄姿态的六关节角、手眼结果、相机内部外参，以及与口扫匹配的牙龈源线和分割预测。输出是机器人 Base 下的几何参考，不包含 TCP 偏置、完整工具姿态、时间、逆解或碰撞规划。

**本批正式读取位置**：`assets/perception/sessions/20261113_220209/planning_input_manifest.json`。
固定标定：`assets/calibration/ur7e_d405/20261113/`。
`outputs/` 存拍摄与计算中间结果；下游读取这批正式数据不需要扫描 `outputs/`。

## 2. 重新采集和配准怎么操作

### 2.1 拍摄之前

固定相机安装和牙模；确认使用对应颌的口扫。机器人关节角应在冻结帧时记录；若拍后机器人保持不动，也可在该静止姿态读取。机器人或牙模已移动时，不能用现在的位姿解释旧帧。

在项目根目录打开 PowerShell，使用已安装项目依赖的 Python：

```powershell
cd D:\teethclean-robot
$py = 'D:\python\python.exe'
& $py -m perception.real devices
```

同一台相机的内部外参已有归档。更换相机、配置或需要复核时，可读取 SDK 数据到新目录：

```powershell
& $py -m perception.acquisition.camera_extrinsics `
  --serial 335122273144 --out outputs/perception/d405_extrinsics_new
```

此脚本读取深度光学帧到彩色光学帧的出厂外参，同时提供标准 ROS 轴约定候选值。当前默认光学帧解释不使用该 ROS 候选值；原标定现场 TF 尚未独立核实。相机内部外参通常可复用；拍摄时机器人位姿每次需要对应记录。相机安装改变后需重新标定手眼。

### 2.2 实时预览、冻结、ROI 与选点

下颌示例（本机优先用已验证的兼容选点器）：

```powershell
& $py -m perception.real manual --source camera `
  --jaw mandibular --scan assets/source_cad/teeth/mandibular_scan.ply `
  --picker legacy --view
```

上颌将 `--jaw` 改为 `maxillary`，口扫改为 `assets/source_cad/teeth/maxillary_scan.ply`。

1. 实时画面中调整相机及牙模，按 **S / 空格**冻结。
2. 在冻结帧拖框选 ROI，按 **Enter** 确认。取消框选返回实时预览；预览中 **Q / Esc** 退出。
3. 第一幅三维图选相机点云中的至少六个特征点；**Shift + 左键**选点，**Shift + 右键**撤销，**Q**完成。
4. 第二幅图在口扫中按完全相同的顺序选择同名点，数量一致，按 **Q**完成。参考图保留第一组点的编号，便于对照。
5. 自动精配准并显示叠加图，记录终端打印的输出目录。后续使用该目录中的报告和矩阵，不凭窗口朝向判断坐标轴。

点应分散在可见牙面，覆盖左右和前后，避免都选在一条直线或局部平坦小区域。三维窗口初始视角与二维预览不同，不代表点云被错误旋转；可以先调整观察视角再选点。冻结、裁剪和保存使用同一帧，避免 ROI 对应到后续画面。

若已有保存的完整采集数据，可不插相机重新选 ROI：

```powershell
& $py -m perception.real manual --source capture `
  --capture <原运行目录/capture> --jaw maxillary `
  --scan assets/source_cad/teeth/maxillary_scan.ply --picker legacy --view
```

这里需要包含原始帧和元数据的采集目录；仅一张截图或裁剪后的 PLY 不能恢复完整 RGB-D 帧。重新裁剪不产生新的真实拍摄时刻，仍须使用原拍摄姿态。

### 2.3 配准原理与误差解释

深度像素通过相机内参反投影为三维点。理想针孔模型为 `X=(u-cx)Z/fx`、`Y=(v-cy)Z/fy`；实际畸变和内参处理由 SDK 完成。预览使用深度网格对应的着色图，使 ROI 像素与深度点对应。

人工同名点确定口扫与相机的初始刚体变换：

```text
p_camera ≈ R p_scan + t
min Σ ||R s_i + t - c_i||²，约束 RᵀR=I、det(R)=1
```

代码用 SVD 求旋转和平移，不估计缩放；稳健求解会枚举三点模型、按 2 mm 阈值筛除错误对应并重估，人工配准要求至少六个有效点。口扫和相机点在这一阶段统一为毫米。

随后进行三尺度 point-to-plane ICP：内部将相机点云向口扫对齐，最后取逆返回口扫到相机的变换。三个阶段的“体素大小 / 对应距离门限”依次为 `1 / 3`、`0.5 / 1.5`、`0.25 / 0.75` mm；采用 Tukey 稳健损失，每阶段最多 100 次迭代。目标是减小对应点到局部表面切平面的距离，初始位姿错误过大仍可能收敛到错误位置。

人工点残差是 `e_i=||R s_i+t-c_i||`，RMS 为 `sqrt(mean(e_i²))`，包含人工选点误差。ICP 的 inlier RMSE 只统计门限内匹配，fitness 表示参与匹配的比例；低 RMSE 可能同时伴随很低的覆盖率。两类指标都不能单独代表真实机器人定位精度。

检查时应同时看全局叠加、牙尖/牙缝、覆盖率和残差分布；真实精度需用未参与配准的独立标记或测量验证。矩阵往返测试只能证明计算一致性，不能证明标定或实际位置准确。

## 3. 如何把口扫坐标变成机器人 Base 坐标

### 3.1 矩阵方向、单位和乘法顺序

统一定义 `T_A_B` 为“把 B 坐标表达的点转换为 A 坐标表达”：`p_A=R_A_B p_B+t_A_B`。齐次矩阵左乘列向量，串联时从右往左执行。

| 符号 | 坐标系 |
| --- | --- |
| S | 口扫局部坐标 |
| D | D405 深度光学帧：右、下、前 |
| Co | D405 彩色光学帧 |
| C | 原 YAML 的帧名声明；当前手眼数值按 Co 光学帧解释 |
| E | 手眼 parent：`wrist_3_link` |
| B | UR 控制器 Base |

```text
T_B_S = T_B_E @ T_E_Co @ T_Co_D @ T_D_S
p_B   = T_B_S @ [p_scan_m, 1]
```

- `T_D_S`：本次口扫配准，来自 `transforms/T_camera_scan_m.npy`；此处 camera 指深度光学帧。
- `T_Co_D`：同一台 D405 的 SDK 深度到彩色光学外参。
- 当前有效链不使用 `T_C_Co`，只有显式选择历史 `declared` 模式才加入 optical → ROS stream 的轴转换。
- `T_E_Co`：归档 `handeye_25.yaml` 原数值，按用户确认解释为彩色光学帧到末端。原 YAML 不改，原标定现场 TF 尚未独立核实。
- `T_B_E`：拍摄时关节角通过 UR7e 正运动学计算。

所有参与此链的矩阵平移均为米，口扫毫米坐标先除以 1000。最终 CSV 已是 Base 米制坐标，不能再乘一次矩阵。此模型下 ROS `base_link` 与控制器 Base 相差绕 Z 轴 180°，需使用文件名中的 `controller_base`，不能混读。

### 3.2 本批手眼和机器人记录

手眼 `T_E_Co`（平移单位米；下列仅展示，计算读归档文件）：

```text
[-0.007710396085  -0.999929868810   0.008989285474   0.052784734483]
[ 0.999968096791  -0.007728828685  -0.002017574656  -0.008900953969]
[ 0.002086909809   0.008973442387   0.999957560169   0.061150245919]
[ 0               0               0               1             ]
```

本批关节角依次为机座、肩、肘、手腕 1、2、3：`[78.28, -46.01, 65.01, -273.56, 29.12, -19.07]` 度。原始记录在正式 session 的 `robot_joint_observation.json`，计算结果在 `fk_report.json`。记录依赖“拍摄后机器人未移动”的条件，没有硬件同步时间戳。

正运动学按 UR7e 标称几何，逐节组合固定连杆变换与关节绕轴旋转，得到 `wrist_3_link` 在基座下的位姿。脚本用独立标准 DH 链核对结果。`nominal` 表示尚未应用该台机器人出厂运动学校准，也没有加入当前工具 TCP 偏置；这也是交付文件保留该后缀的原因。

### 3.3 离线计算命令

以下是重算流程模板；尖括号需替换为真实路径，所有输出使用新目录。新拍摄必须创建新的关节记录，不能照抄本批角度。

```powershell
& $py -m perception.coordinate_conversion.ur7e_fk `
  --joint-observation <本次关节记录.json> --out <新的FK目录>

& $py -m perception.coordinate_conversion.convert_points `
  --mandibular-run <下颌采集配准运行目录> `
  --maxillary-run <上颌采集配准运行目录> `
  --fk-report <新的FK目录/fk_report.json> `
  --handeye-inventory assets/calibration/ur7e_d405/20261113/handeye_inventory.json `
  --handeye-label 25 --handeye-frame color_optical `
  --camera-extrinsics assets/calibration/ur7e_d405/20261113/camera_extrinsics.json `
  --out <新的转换目录>
```

关节记录 JSON 至少包含按上述顺序排列的 `joint_positions_deg` 六元素数组。转换命令可只指定一颌；两颌一起指定时，代码要求来自同一次原始采集和拍摄位姿。分两次拍摄的两颌应分别使用各自的 FK 计算。

转换会检查配准文件指纹、采集身份与设备，并检查分步变换等于组合变换以及逆变换恢复原点。它只导出整颌点集和矩阵，**不会自动生成六区**。

## 4. 六区如何产生，再如何导出

六区顺序为 `outer_left`、`outer_front`、`outer_right`、`inner_right`、`inner_front`、`inner_left`。上下颌各自独立；六区之间没有自动补线或连接运动。

区域源线来自 `assets/regional_paths/<jaw>/`。代码先利用源文件中的历史变换还原口扫局部坐标，再使用本次新配准确定 Base 位置，不会把历史相机位姿当成本次拍摄位姿。

下颌使用已有六区源线；上颌由 `perception/regions/build_template_source.py` 参考下颌结构，将锚点映射到上颌牙龈拓扑，沿上颌网格边寻找连接路径并消除回环。它不是将下颌坐标直接用作上颌坐标；本批上颌仍是需要人工核对的候选分区，不能把“已经分成六段”理解为解剖边界已经批准。

区域模块根据匹配的 CrossTooth 分割预测分配牙齿支撑，拟合连续 B 样条曲线和面积加权的表面参考法向，再计算曲线切向。牙齿边界用于划分方向支撑，不会把整条区域曲线切断。区域两端执行整体裁剪；本批下颌每端 1 mm，上颌每端 0.25 mm，以保留较短的端点支撑。

在转换目录下生成区域的命令模板：

```powershell
& $py -m perception.regions --jaw mandibular `
  --world-from-scan <下颌通过刚体校验的口扫到Base矩阵.npy> `
  --trim-mm 1 --out <新的转换目录/mandibular/regions>

& $py -m perception.regions --jaw maxillary `
  --world-from-scan <上颌通过刚体校验的口扫到Base矩阵.npy> `
  --trim-mm 0.25 --out <新的转换目录/maxillary/regions>

& $py -m perception.coordinate_conversion.export_regions `
  --conversion-dir <新的转换目录>
```

本批两颌区域均使用有来源记录的 `T_controller_base_scan_nominal_rigid_m.npy`：通过 SVD 消除 SDK 浮点旋转误差，保持平移不变，各颌目录保存 `regional_transform_normalization.json`。新转换结果应通过刚体校验后再准备区域；不能用放宽检查来掩盖问题。这个数值正交化不能改善真实标定精度。`--world-from-scan` 是通用参数名，本流程赋予它的 world 实际是控制器 Base。

位置转换为 `p_B=R p_S+t`；法向与切向只乘 `R` 并归一化，不加平移。输出参数 `parameters_m` 是曲线参数，不是时间，也不能直接当作精确累计弧长。

核查后可发布到新的正式资产目录：

```powershell
& $py -m perception.coordinate_conversion.publish_session `
  --conversion-dir <新的转换目录> `
  --out <新的assets/perception/sessions/批次目录> `
  --calibration-out <新的标定归档目录>
```

发布器要求目标目录不存在，会重写相对路径和文件指纹。默认保留规划必需的数据，并让每颌共用一份网格；不复制原始拍摄大文件。新批次应核对清单、来源和说明后再交付，不能覆盖旧批次冒充同一采集。

## 5. 下游规划读取什么

首先读取正式 session 的 `planning_input_manifest.json`，按照清单路径和校验信息寻找区域。每区的主要接口为：

| 文件 | 含义 |
| --- | --- |
| `points_controller_base_nominal_m.csv` | Base 下米制位置，N×3 |
| `region_controller_base_nominal.npz` | `points_m`、`normals`、`tangents`、`parameters_m`、`tooth_ids`、区域变换 |
| `perception_region.json` | `PreparedRegion` 几何接口与来源指纹 |
| `base_coordinates.json` | 坐标、单位与转换说明 |

最小读取示例（从项目根目录运行）：

```python
from pathlib import Path
import numpy as np

session = Path('assets/perception/sessions/20261113_220209')
region = session / 'mandibular/regions/outer_left'
data = np.load(region / 'region_controller_base_nominal.npz')
points, normals, tangents = data['points_m'], data['normals'], data['tangents']
```

本批下颌共 2711 点，上颌共 1984 点。它们是几何采样点数，不是控制周期数。规划同事基于这些参考确定工具姿态、TCP/接触偏置、法向使用方向、进退刀和区域间转移，再完成逆解、碰撞检查、速度与时间分配。法向加切向提供方向参考，不能直接当作已经确定的工具四元数。

## 6. 对应代码与核查入口

| 阶段 | 代码 |
| --- | --- |
| 实时预览、冻结、ROI、重新裁剪 | `perception/real.py` |
| 人工流程与多尺度 ICP | `perception/registration/manual_real.py` |
| 同名点刚体求解与残差 | `perception/registration/registration_math.py` |
| 兼容选点与编号参考图 | `perception/registration/legacy_landmark_picker.py`、`landmark_reference.py` |
| D405 内部外参与 ROS 轴约定 | `perception/acquisition/camera_extrinsics.py` |
| UR7e 标称正运动学 | `perception/coordinate_conversion/ur7e_fk.py` |
| 坐标链组合与整颌点转换 | `perception/coordinate_conversion/convert_points.py` |
| 六区源线与曲线、牙面支撑 | `perception/regions/build_template_source.py`、`frozen.py`、`geometry.py`、`surface.py` |
| 区域 Base 坐标导出 | `perception/coordinate_conversion/export_regions.py` |
| 稳定资产发布与精简 | `perception/coordinate_conversion/publish_session.py`、`compact_session.py` |

交付前重点核对：口扫/预测/区域源线指纹匹配；同名点与叠加合理；拍摄位姿对应；矩阵方向与单位明确；区域数量、连续性和覆盖有效；正式清单能定位并加载各区。当前数据保留标称运动学、位姿同步和上颌候选边界的限制，具体记录以[本批交接说明](2026-11-13-perception-handoff.md)和 JSON 为准。
