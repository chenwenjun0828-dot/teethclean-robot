# 感知模块交接说明：上下颌六区域规划输入

采集与交付日期：2026-11-13。源码整理日期：2026-11-14。交付对象：路径规划、运动控制同事。

正式规划输入已迁入 assets/perception/sessions/20261113_220209/，固定标定记录在 assets/calibration/ur7e_d405/20261113/；规划读取不再依赖 outputs/。三个可复用计算入口已迁入 `perception/coordinate_conversion/`，详见该目录 README；`outputs/` 中对应的旧计算脚本已移除，生成数据保留。

## 1. 交付内容与接口边界

本次感知交付的是：**上下颌各六个独立区域的牙龈参考曲线、机器人基座坐标下的位置点、表面参考法向、曲线切向，以及原始口扫到基座的变换和数据来源记录**。这些数据用于后续规划，不是可直接下发控制器的机器人轨迹。

统一交付根目录（下文称 `交付根目录`）：

```text
D:/teethclean-robot/assets/perception/sessions/20261113_220209/
├── README.md                         数据包读取入口（完整说明在 docs/integration/）
├── planning_input_manifest.json       机器可读交接清单，含文件指纹
├── transforms/                      固定手眼、拍摄位姿和相机到基座矩阵
├── robot_joint_observation.json       本批关节角
├── fk_report.json                     本批标称正运动学结果
├── regional_export_report.json        当前上下颌六区域统计
├── mandibular/                       下颌
│   └── regions/<区域名>/             每区域独立目录
└── maxillary/                        上颌
    └── regions/<区域名>/             每区域独立目录
```

优先读取 `planning_input_manifest.json`，不要按文件修改时间寻找结果。该清单以自身所在目录为路径基准。**正式分区输入以各颌的 `regions/` 为准**；初次转换的整颌点集只保留在历史输出，不作为本批规划读取入口。

已完成与尚未完成的集成：

- 手眼 `_25.yaml`、矩阵、D405 内部外参、关节角、标称正运动学结果已归档，并已用于本次坐标组合。
- 上下颌已保存现有框架的 `PreparedRegion` / `perception_region.json`，可作为规划模块的数据输入。
- `config/camera_to_base.example.json` **仍是 placeholder 示例**，未被覆盖；本次不是通过该配置启动真机执行。
- 当前转换由 `perception/coordinate_conversion/convert_points.py` 与 `export_regions.py` 完成，是可复现的离线交付流程；未新增自动接收机器人实时位姿并向控制器下发轨迹的接口。

## 2. 运控同事应该读取哪些文件

每个区域目录中最直接的输入：

本交付采用精简布局：每颌只有一份 `<jaw>/shared/region_mesh.npz`，由区域 manifest 的
`artifact_paths` 相对路径引用并保留原有哈希校验。逐牙预览 PLY、逐牙标签 NPY 和重复网格
不再复制到 Git；完整实验输出留在本地 `outputs/`。

| 文件 | 内容 | 单位 / 坐标系 |
| --- | --- | --- |
| `points_controller_base_nominal_m.csv` | 按区域曲线方向排列的 N×3 位置点 | 米，机器人控制器 Base |
| `region_controller_base_nominal.npz` | 位置、法向、切向、曲线参数、牙齿编号及变换 | 位置/曲线参数为米，方向无单位 |
| `base_coordinates.json` | 坐标定义、裁剪参数、变换来源和状态 | 显式标注 controller_base |
| `perception_region.json` | 现有规划框架的 PreparedRegion 入口 | 含口扫局部几何及定位矩阵 |
| `continuous_gingival_line.npz` | 曲线在口扫局部坐标的点和法向 | 米，**不是基座坐标** |

`region_controller_base_nominal.npz` 的字段：

```text
points_m                    N×3，机器人 Base 坐标下的位置
normals                     N×3，机器人 Base 坐标下的表面参考单位法向
tangents                    N×3，机器人 Base 坐标下沿当前区域方向的单位切向
parameters_m                N，曲线采样参数
tooth_ids                   N，最近 CrossTooth 标签，用于姿态支持，不作为已认证 FDI 牙位
T_controller_base_scan_m     4×4，口扫局部米制坐标 → 机器人 Base
```

法向和切向是姿态规划的几何参考，不是已经确定的工具姿态。方向与点按同行对应。`parameters_m` 是现有曲线接口的参数，**不是时间，也不要据此直接计算进给速度**；真实位置距离应从曲线或相邻位置点计算，再由规划层定时。

### 最小读取示例

从项目根目录运行以下 Python 代码即可读取，不需要连接机械臂或相机：

```python
from pathlib import Path
import numpy as np

root = Path('assets/perception/sessions/20261113_220209')
region = root / 'mandibular/regions/outer_front'  # 上颌改成 maxillary

points = np.loadtxt(region / 'points_controller_base_nominal_m.csv', delimiter=',')
with np.load(region / 'region_controller_base_nominal.npz', allow_pickle=False) as data:
    normals = data['normals'].copy()
    tangents = data['tangents'].copy()
    T_base_scan = data['T_controller_base_scan_m'].copy()

assert points.ndim == 2 and points.shape[1] == 3
assert points.shape == normals.shape == tangents.shape
# points 已在机器人 Base 中，不要再乘 T_base_scan。
# 只有下游接口明确要求毫米时才执行 points_mm = points * 1000。
```

使用现有框架的曲线接口时：

```python
from perception.regions import PreparedRegion

prepared = PreparedRegion.load(region)
s = prepared.reference.samples(0.00025)  # 采样间距参数，米
points_scan = np.array([prepared.reference.evaluate(x) for x in s])
T = prepared.world_from_scan
points_base = points_scan @ T[:3, :3].T + T[:3, 3]
```

**本批 PreparedRegion 中，`world_from_scan` 的 world 明确定义为机器人控制器 `controller_base`。** `continuous_gingival_line_world_m.csv` 因而也是该基座坐标；建议使用显式命名的 `points_controller_base_nominal_m.csv`。不要把这个 world 当成项目原有 MuJoCo 世界坐标；使用现有仿真规划器时，需由规划层核对机器人安装位置和世界原点。`planning` 目前的验证/执行链包含 MuJoCo，不能把其仿真执行入口等同于实机下发入口。

PreparedRegion 读取会核对源文件及产物指纹。交付给另一台电脑时，应复制项目源码及匹配的 `assets/`，保留相对目录结构；读取本批规划输入不需要复制 `outputs/`。只复制单个 JSON 或 CSV 不包含全部几何、分割和标定来源。

## 3. 六区域定义与本批数据量

区域顺序采用既有框架约定；名称中的左右沿用源数据定义，不额外宣称已认证的患者解剖左右。不同区域独立保存，**不存在隐含的跨区域连接或接触转移轨迹**。

| 区域 | 含义 | 下颌点数 | 上颌点数 |
| --- | --- | ---: | ---: |
| `outer_left` | 外侧左区 | 416 | 323 |
| `outer_front` | 外侧前区 | 430 | 315 |
| `outer_right` | 外侧右区 | 479 | 486 |
| `inner_right` | 内侧右区 | 416 | 305 |
| `inner_front` | 内侧前区 | 553 | 279 |
| `inner_left` | 内侧左区 | 417 | 276 |
| 合计 | | 2711 | 1984 |

下颌沿用已有六区源线，整区两端各裁剪 1 mm。上颌参考下颌六区结构，在**上颌自身**牙龈线和网格上生成源线；整区两端各裁剪 0.25 mm，以保留端点牙位支持。没有逐牙重新裁剪或跨区补线。

上颌源线生成方法：XZ 牙弓模板转移、牙龈线锚点选取、上颌网格边加权最短路、重复顶点回环清除。它包含新生成的表面路线，**不是简单复制下颌坐标，也不是只保留原始提取线的所有边**。边界状态仍为 `region_boundaries_approved=false`；下颌源快照本身也没有边界已批准标记。

上颌预览：[regions_overview.png](../../assets/perception/sessions/20261113_220209/maxillary/regions/regions_overview.png)。表面检查：[surface_qa.json](../../assets/perception/sessions/20261113_220209/maxillary/regions/surface_qa.json)。本批上颌采样曲线到网格的最大距离约 0.176 mm；此数值仅为几何贴合检查，不是配准绝对精度、接触精度或执行精度。

## 4. 坐标转换的完整逻辑

定义 `T_A_B` 为“把 B 坐标中的点变换到 A”：`p_A = R_A_B @ p_B + t_A_B`。4×4 矩阵用于列向量左乘；NumPy N×3 行数组使用 `P_A = P_B @ R_A_B.T + t_A_B`。

坐标系：

| 符号 | 含义 |
| --- | --- |
| S | 对应颌的原始口扫局部坐标 |
| D | D405 深度光学坐标：X 右、Y 下、Z 前 |
| Co | 彩色光学坐标 |
| C | 手眼 YAML 的 `camera_color_frame`，按标准 ROS stream-frame 约定 |
| E | 手眼 YAML 的末端 `wrist_3_link` |
| B | 机器人控制器/示教器 Base，文件命名为 `controller_base` |
| L | ROS `base_link`；与 B 存在绕 Z 轴 180° 的固定转换 |

```text
口扫 S --配准--> 深度光学 D --设备外参--> 彩色光学 Co
       --标准 ROS 轴变换--> camera_color_frame C
       --手眼标定--> wrist_3_link E --拍摄位姿--> 控制器 Base B

T_B_S = T_B_E @ T_E_C @ T_C_Co @ T_Co_D @ T_D_S
```

本批上下颌来自**同一张原始 RGB-D 帧**，只分别框选 ROI 并与各自口扫配准，因此共用 `T_B_E`、手眼标定和相机内部外参；两颌的 `T_D_S` 不同，必须分别使用。

原始拍摄：2026-11-13 22:02:09.329364（北京时间），元数据中记录为 `2026-11-13T14:02:09.329364+00:00`。
相机：Intel RealSense D405，序列号 `335122273144`，固件 `5.17.0.10`。

原始口扫、配准目录内相机 PLY 多为毫米；本交付基座 CSV/NPZ/PLY 和组合矩阵平移均为米。
把米制矩阵用于口扫点时，应先将口扫点除以 1000。**不得只换平移单位而保留毫米点坐标。**

### 手眼矩阵：本次采用 `_25.yaml`

已保存到项目：

```text
assets/calibration/ur7e_d405/20261113/handeye_25.yaml
交付根目录/transforms/T_wrist3_color_frame_m.npy
交付根目录/transforms/T_wrist3_color_frame_m.txt
assets/calibration/ur7e_d405/20261113/handeye_25.yaml
```

原始 YAML：`eye_on_hand=true`，parent=`wrist_3_link`，child=`camera_color_frame`。
平移 `[x,y,z]`（米）：`[0.052784734483046124, -0.00890095396872504, 0.0611502459188928]`。
四元数 `[qx,qy,qz,qw]`：`[0.0039010393200360908, 0.002449858713323372, 0.7098233556560035, 0.7043646668095719]`。

`T_E_C`（显示精度 12 位；实际计算用 NPY）：

```text
[[-0.007710396085, -0.999929868810,  0.008989285474,  0.052784734483],
 [ 0.999968096791, -0.007728828685, -0.002017574656, -0.008900953969],
 [ 0.002086909809,  0.008973442387,  0.999957560169,  0.061150245919],
 [ 0,               0,               0,               1             ]]
```

这是相机到末端的固定关系，**不是相机到基座的外参**。不要用它直接替换 `T_base_camera`。
归档的 `_13.yaml` 只供比较，本批未用它参与计算。

### 相机内部转换与 ROS 帧约定

设备读取记录：`assets/calibration/ur7e_d405/20261113/camera_extrinsics.json`，该固定标定目录保留原始读取内容。
SDK 的 `T_color_optical_depth_optical_m` 与两颌原始采集元数据中的 `T_color_depth_m` 已核对一致，设备序列号一致。

标准 ROS 的 `T_C_Co`：

```text
[[ 0,  0, 1, 0],
 [-1,  0, 0, 0],
 [ 0, -1, 0, 0],
 [ 0,  0, 0, 1]]
```

SDK 读取的是 optical 到 optical；上面的 ROS 轴转换不是从 Windows SDK 实测出的 ROS TF。本批按感知提供者已确认 `camera_color_frame` 定义正确使用该约定，保留原始报告中的“未与现场 live TF 核对”标记作为来源事实。

### 机器人拍摄位姿

示教器提供的关节角按度处理，顺序为基座、肩、肘、腕1、腕2、腕3：

```text
[78.28, -46.01, 65.01, -273.56, 29.12, -19.07] deg
```

正运动学使用 UR 官方开源 UR7e **标称参数**，离线计算，不需要 ROS 或机械臂连接。
`T_B_E` 与 `T_L_E` 分别保存，避免控制器 Base 和 ROS base_link 混用。
本批以机械臂从按 S 拍摄到读角度保持静止为时间对齐条件；关节角没有设备同步时间戳，输入精度为示教器手工抄录的两位小数。

位姿记录直接保存在交付根目录的 `robot_joint_observation.json` 和 `fk_report.json`。它们来自原始下颌实验记录，但正式读取不需要原始输出目录。
计算已用 URDF 变换链与独立 DH 计算交叉验证，但**未加入该台机械臂的出厂运动学修正**。

矩阵文件选择：

- 下颌六区域：`mandibular/T_controller_base_scan_nominal_m.npy`。
- 上颌六区域：`maxillary/T_controller_base_scan_nominal_rigid_m.npy`。
- 上颌完整点集初次转换使用非 `_rigid` 版本；区域版将约 `1.66e-8` 的旋转元素数值误差投影回正交旋转，平移不变；详情在 `maxillary/regional_transform_normalization.json`。两者差别是浮点数级别，不是新标定。
- 名称包含 `base_link` 的其他矩阵和点云是 ROS 坐标版本，不要与上述 `controller_base` 文件混用。

## 5. 原始结果、源线与相关代码

### 本批原始配准结果

```text
下颌：outputs/perception/real/20261113_220105_579747_mandibular/
上颌：outputs/perception/real/20261113_224125_939919_maxillary/
```

两目录内的 `capture/` 保留完整深度图、点图、颜色、设备信息和原始点云；`registration/` 保留 ROI 点云、人工对应点、初始/ICP 矩阵及报告。
`transforms/T_camera_scan_m.npy` 是各颌口扫到深度相机的米制配准矩阵。
这些运行根目录中的 `base_coordinates_available=false` 仍描述**原始配准时未输入基座外参**的状态；后续转换结果另存在本交付根目录，未篡改原始配准报告。

### 代码职责

| 代码 | 职责 |
| --- | --- |
| `perception/real.py` | D405 采集、保存完整原始数据、离线重裁 ROI |
| `perception/acquisition/realsense.py`、`capture.py` | SDK 点图/深度光学坐标和采集元数据持久化 |
| `perception/acquisition/camera_extrinsics.py` | 读取设备深度/彩色外参、检查方向，导出标准 ROS 约定候选变换 |
| `perception/registration/manual_real.py` | 相机/PLY/保存采集入口，人工初始化与三尺度 ICP |
| `perception/registration/legacy_landmark_picker.py`、`landmark_reference.py` | 兼容选点窗口、独立参考图进程、相机标记恢复 |
| `perception/frames.py`、`artifacts.py` | 单位/变换校验、已有坐标导出接口；本批基座组合还使用下列离线脚本 |
| `perception/regional_paths.py` | 六区域源线读取、模型哈希检查、移除源数据序列化时的相机放置 |
| `perception/regions/result.py` | PreparedRegion、六区域准备、保存/加载及指纹验证 |
| `perception/regions/frozen.py`、`geometry.py`、`continuous.py`、`surface.py` | 区域曲线拟合、牙齿归属、法向、连续曲线和整区端点裁剪 |
| `perception/regions/build_template_source.py` | 本次新增的上颌六区域候选源线构建，显式运行，不是运行时隐式回退 |
| `perception/coordinate_conversion/ur7e_fk.py` | UR7e 标称正运动学，通过显式关节记录和输出目录参数离线运行 |
| `perception/coordinate_conversion/convert_points.py` | 组合拍摄位姿、手眼、设备外参和配准，导出两种基座坐标版本 |
| `perception/coordinate_conversion/export_regions.py` | 将 PreparedRegion 导出为显式 controller_base 坐标点、法向和切向 |
| `planning/pipeline.py`、`planning/__main__.py` | 现有规划消费者入口；不属于本次感知交付的真机执行验证 |

源线数据：`assets/regional_paths/mandibular/`、`assets/regional_paths/maxillary/`。
口扫：`assets/source_cad/teeth/{mandibular,maxillary}_scan.ply`。
分割预测：`assets/perception/tooth_segmentation/crosstooth_{mandibular,maxillary}/`。
标定读取入口：`assets/calibration/ur7e_d405/20261113/README.md`。早期审核过程保留在本地 `outputs/perception/handeye_audit/README.md`，只用于历史追溯，不是规划加载依赖。

复现时须使用实际同批数据路径，输出到新目录；转换脚本会新建时间戳目录。任何一项配准、手眼、相机帧、拍摄位姿改变，都应重新组合坐标，而不是编辑已有点 CSV。

## 6. 感知交付状态与下游责任

已检查：设备/同一原始帧身份、源口扫与配准指纹、矩阵组合与逐级转换一致、逆变换回原坐标、12 个区域保存/加载、上颌曲线表面距离，以及分区源线与移植相关测试。

仍然保留的状态：

- 标称运动学尚未替换为实机出厂修正；手工关节角和时间对齐条件如上。
- 上颌六区为新生成的候选边界，尚未人工批准；下颌沿用源快照的未批准标记。
- ICP 内点 RMSE、人工对应点残差及表面贴合距离均不是独立绝对精度证明。
- 所有相关交付保持 `robot_execution_allowed=false`，表示本次没有完成真机轨迹执行验收。

规划/运控层后续处理：

1. 接收各区域基座位置、法向、切向，核对模型与控制器使用的基座定义。
2. 根据洁治工具实际尖端和安装，使用工具 TCP 标定；这些点是牙龈参考几何位置，不是机械臂法兰目标位置。
3. 结合工具轴定义、牙面方向、目标角度、离面/接触偏置及姿态连续性生成完整目标位姿；法向不等于完整四元数。
4. 完成逆运动学、关节分支、限位、完整工具/双颌碰撞检查、进出场和跨区抬起转移、速度/加速度及时间参数化。
5. 用实机或独立参考验证坐标链和配准精度，再由运控层按其验收流程决定是否执行。感知交付不包含控制器通信、接触力控制或运动执行。

**交接结论：本目录是后续规划的感知几何输入包；位置已转换到机器人 Base，六区域相互独立，当前精度等级和待确认事项已显式保留。**
