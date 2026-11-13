# 本批正式规划输入：2026-11-13 22:02:09

**运控/规划读取本目录，不读取 `outputs/`。**
完整交接说明：[docs/integration/2026-11-13-perception-handoff.md](../../../../docs/integration/2026-11-13-perception-handoff.md)。

## 读取位置

```text
assets/perception/sessions/20261113_220209/
├── planning_input_manifest.json      相对路径清单和文件指纹
├── robot_joint_observation.json      本批拍摄位姿对应的关节角记录
├── fk_report.json                    本批标称正运动学结果
├── transforms/                      共享手眼与拍摄位姿组合矩阵
├── mandibular/regions/<区域>/        下颌六区
└── maxillary/regions/<区域>/          上颌六区
```

六区依次为 `outer_left`、`outer_front`、`outer_right`、`inner_right`、`inner_front`、`inner_left`。
下颌共 2711 点，上颌共 1984 点。每区独立，无跨区连接。

每区读取：

- `points_controller_base_nominal_m.csv`：机器人控制器 Base 下的 N×3 位置，单位**米**，无需再乘变换矩阵。
- `region_controller_base_nominal.npz`：同一组位置 `points_m`、单位法向 `normals`、单位切向 `tangents`、曲线参数和牙齿标签。
- `perception_region.json`：现有 `PreparedRegion` 接口；其局部曲线仍在口扫坐标，`world_from_scan` 在本批定义为 controller_base。
- `base_coordinates.json`：坐标、状态、裁剪量、变换来源和指纹。

本包只保留规划必需数据。每颌网格共享于 `<jaw>/shared/region_mesh.npz`，
由区域清单的 `artifact_paths` 引用并校验，六个区域不再各存一份。
逐牙预览 PLY、独立逐牙标签 NPY、重复逐区网格和三维预览点云不在本包重复保存；
完整实验输出仍在本地 `outputs/`。

```python
from pathlib import Path
import numpy as np
from perception.regions import PreparedRegion

root = Path('assets/perception/sessions/20261113_220209')
region = root / 'mandibular/regions/outer_front'  # 上颌用 maxillary
points = np.loadtxt(region / 'points_controller_base_nominal_m.csv', delimiter=',')
with np.load(region / 'region_controller_base_nominal.npz', allow_pickle=False) as data:
    normals = data['normals'].copy()
    tangents = data['tangents'].copy()
prepared = PreparedRegion.load(region)  # 核对源文件、几何和产物指纹
```

固定标定记录在 `assets/calibration/ur7e_d405/20261113/`，清单使用相对路径引用。
本包依赖项目中 `assets/source_cad/`、`assets/perception/`、`assets/regional_paths/` 的匹配源资产，
应随仓库一起复制；单拷一个区域目录并不包含全部来源。

## 数据状态

- 本批两颌来自同一原始 RGB-D 帧，相机序列号 `335122273144`。
- 使用手眼 `_25` 和六个关节角计算的 UR7e **标称**运动学；未包含该台机械臂的出厂修正。
- 以从拍摄到读取关节角期间机械臂未移动为时间对齐条件。
- 上颌采用新生成、待核对的六区候选源线；区域边界尚未批准。
- 位置和参考方向是后续规划输入，不是工具 TCP 的最终执行位姿或已定时轨迹。
- `robot_execution_allowed=false`。工具 TCP、姿态、偏置、碰撞、可达性、时间参数化和实机验收由下游处理。

原始 RGB-D、重裁 ROI 与人工选点过程仍保存在本地 `outputs/perception/real/`，
用于重新处理与实验追溯；**它们不是读取本规划资产包的依赖**。
