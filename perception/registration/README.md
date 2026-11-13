# 牙齿点云配准算法开发

真实点云的人工配准统一入口是 `python -m perception.registration.manual_real`，支持连接 D405
采集、文件选择框或指定已有 PLY；历史 `4.ply` 已作为仓库样例导入。
具体命令、单位/坐标系与结果格式见 [人工配准接口](MANUAL_REAL.md)。

局部 RGB-D 牙齿点云与完整口扫模型的配准实现统一放在本目录。

固定外置 D405 的真机采集、离线配准、检查点评价和实时化限制见
[真机验证指南](../REAL_CAMERA.md)。入口为 `python -m perception.real`，
与下文现有仿真入口并存；未提供相机到基座的标定时仅输出相机坐标。

| 文件 | 职责 |
| --- | --- |
| `registration_workflow.py` | 检查采集与口扫是否匹配，读取或点选标志点，运行配准，保存变换、报告和叠加预览 |
| `automatic.py` | 无人工选点的统计离群点过滤、FPFH、RANSAC、Point-to-Plane ICP |
| `main.py` / `__main__.py` | 独立点云自动配准运行入口、结果保存与可视化 |
| `algorithms.py` | 加载口扫点云及牙龈线，体素下采样、法线估计，标志点初值和粗/细 point-to-plane ICP |
| `registration_math.py` | Kabsch 刚体估计、异常标志点剔除、点坐标变换与残差计算 |
| `landmark_picker.py` | Open3D 左右点云配对窗口 |
| `landmark_selection.py` | 配对顺序、撤销、完成条件及屏幕坐标计算 |
| `__init__.py` | 提供统一的算法和流程接口，按需加载实现 |

## 开发与调用

完整采集配准流程继续由 `perception/workflow.py` 调用 `register_capture`：

```python
from perception.registration import register_capture

result_dir = register_capture(
    capture_dir="outputs/my_capture",
    config_path="config/global_camera_registration.json",
    jaw="mandibular",
    output_dir="outputs/my_registration",
    registration_method="automatic",  # 自动方法，不打开人工配对窗口
    show_view=False,
)
```

`registration_method` 可选 `manual` 或 `automatic`，调用参数优先于配置。
未传调用参数时读取配置中的 `registration_method`，缺省为 `manual`。
人工模式可传 `landmarks` JSON，否则打开配对窗口；自动模式不接受人工标志点输入。

仅开发自动算法时，直接调用：

```python
from perception.registration import AutomaticRegistrationConfig, register_clouds_automatic

options = AutomaticRegistrationConfig(voxel_mm=0.5, seed=0)
T_camera_scan, metrics = register_clouds_automatic(scan_cloud, camera_cloud, options)
# metrics["T_camera_scan_initial"] 同样为口扫模型 -> 相机的 RANSAC 初始矩阵。
print(metrics["fitness"], metrics["inlier_rmse_mm"])
```

人工标志点初值算法继续使用原有点云接口：

```python
from perception.registration import coarse_from_landmarks, register_clouds
from perception.registration.registration_math import transform_points

# source_scan_mm 与 target_camera_mm 为同序的对应点，默认至少六组。
initial, inliers, residuals = coarse_from_landmarks(source_scan_mm, target_camera_mm)
T_camera_scan, metrics = register_clouds(scan_cloud, camera_cloud, initial)
registered_points = transform_points(scan_points_mm, T_camera_scan)
```

点云及平移单位为毫米。返回变换始终为 **口扫坐标 → 相机坐标**：
`camera_points = scan_points @ T_camera_scan[:3, :3].T + T_camera_scan[:3, 3]`。
自动算法和人工算法默认的 ICP 均用局部相机点云匹配完整口扫点云（`camera_to_scan`），返回前取逆，
因此调整 ICP 实现时需保留上述变换方向。流程再结合采集元数据转换世界坐标，单位为米。

配准参数仍位于 `config/global_camera_registration.json`；
采集、配置加载和坐标系公共工具仍位于父目录的 `global_camera.py` 与 `frames.py`。
人工算法及其默认参数保持不变。配准实现只保留在本目录中，
父目录中的旧兼容入口已删除；项目调用、测试及点选模块均使用本目录的新导入路径。
新增或修改配准实现请编辑本目录中的文件。

## 自动算法参数

完整流水线读取 `config/global_camera_registration.json` 中的 `automatic_registration` 对象，
人工配准参数仍使用原有的顶层字段。Python 接口对应 `AutomaticRegistrationConfig`。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `voxel_mm` | 0.5 | FPFH/RANSAC 点云的体素边长，毫米 |
| `normal_radius_mm` | null → `2 * voxel_mm` | 法向估计邻域，最多 30 个邻点 |
| `fpfh_radius_mm` | null → `5 * voxel_mm` | FPFH 邻域，最多 100 个邻点 |
| `ransac_distance_mm` | null → `1.5 * voxel_mm` | RANSAC 最大对应点距离 |
| `icp_distance_mm` | null → `voxel_mm` | ICP 最大对应点距离 |
| `refinement_voxel_mm` | null → `0.5 * voxel_mm` | ICP 使用更细点云；设为 0 使用清洗后的原始点云 |
| `outlier_neighbors` / `outlier_std_ratio` | 20 / 2.0 | 统计离群点过滤参数 |
| `minimum_points` | 200 | 原始、清洗和下采样后点云的最低点数 |
| `ransac_iterations` / `ransac_confidence` | 100000 / 0.999 | RANSAC 停止条件 |
| `edge_length_ratio` | 0.9 | 对应点边长一致性检查 |
| `icp_iterations` | 50 | Point-to-Plane ICP 最大迭代次数 |
| `seed` | 0 | Open3D 随机种子；并行 RANSAC 的逐次结果可能有细微差异 |

法向半径、特征半径和距离阈值为 `null` 时随体素大小派生；显式填写数字时使用该数字。
报告记录实际使用的完整参数、各阶段点数、RANSAC/ICP 对应点数量和运行时间。
预处理及配准不会修改传入的两个点云。

## 运行自动配准

项目根目录下运行完整流水线，无人工点选：

```powershell
.\.venv\Scripts\python.exe -m pipeline.run --jaw mandibular --registration-method automatic --headless
```

也可将配置中的 `registration_method` 改为 `automatic`。
选择自动方法后，`--headless` 的完整流程不打开配准或执行窗口。

只使用两个已保存的毫米点云时：

```powershell
.\.venv\Scripts\python.exe -m perception.registration `
  --scan '.\assets\source_cad\teeth\mandibular_scan.ply' `
  --camera '.\outputs\global_camera_registration\capture_002\camera_mm.ply' `
  --out '.\outputs\automatic_registration\my_run' `
  --voxel-mm 0.5 --no-view
```

`--scan` 是完整口扫，`--camera` 是局部相机点云，支持 Open3D 可读取的 PLY/PCD。
两者需使用相同的毫米单位；算法不估计尺度。省略 `--no-view` 显示三维叠加窗口。
`python -m perception.registration.main` 使用同一入口。
`--settings` 可读取仅包含自动参数的 JSON 对象，命令行参数覆盖 JSON；
`--normal-radius-mm`、`--fpfh-radius-mm`、`--ransac-distance-mm`、`--icp-distance-mm` 等可单独调整。

## 自动结果

- `T_camera_scan_initial.npy/.txt`：RANSAC 初始变换，口扫模型 → 相机。
- `T_camera_scan_estimated.npy/.txt`：ICP 最终变换，口扫模型 → 相机。
- `scan_registered_camera_mm.ply`：完整口扫变换到相机坐标，保留原口扫点数。
- `registration_report.json`：方法、参数、初值、Fitness、RMSE、点数和方向信息。
- `registration_overlay.png`：三方向静态叠加预览。

采集配准接口还保存现有的牙龈线相机坐标及世界坐标输出，可直接供后续轨迹规划读取。
自动结果中人工标志点数为 0，不生成 `landmark_correspondences.json`。
独立双点云入口没有采集元数据，因此只用于配准开发与几何输出，不直接提供世界牙龈线。

**变换方向与评价方向分别记录**：保存和返回的变换始终为口扫到相机，
但 `fitness` / `inlier_rmse_mm` 是局部相机点云到完整口扫的匹配统计，
其坐标系由 `fitness_source_frame`、`fitness_target_frame` 明确标注。
`icp_fine_fitness` / `icp_fine_rmse_mm` 与上述值相同，供现有流程使用。

空点云、非有限坐标、退化几何、无效参数、点数不足或未建立至少三组对应点时抛出错误，
完整流水线记录失败并停止后续规划执行。第一版按提供文档实现单阶段精配准，
尚未加入牙齿分割、多尺度 ICP 或复杂的成功判定；重复牙形、很小的覆盖区域和低重叠
可能导致错误定位，高 Fitness 或低 RMSE 本身不能验证牙位正确。

## 验证

在项目根目录运行现有配准与配对状态测试：

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_global_registration tests.test_paired_landmark_picker -v
```

自动算法及完整自动流程测试：

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_automatic_registration -v
```

包含完整流程的项目测试：

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

已有上下颌 MuJoCo 仿真采集样例的自动结果保存在
`outputs/automatic_registration/capture_002_both_001/`。
该次运行的 Fitness 为 0.8859、Inlier RMSE 为 0.1699 mm；
运行后读取独立仿真真值评价得到平移误差 0.0522 mm、旋转误差 0.00759°，
详见该目录的 `independent_evaluation.json`。真值没有作为配准输入。
这些数值描述该仿真样例，不代表真实口腔采集的普遍精度。
