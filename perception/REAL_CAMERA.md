# D405 真机采集与配准验证

本文保留相机采集、文件格式、离线回放与评价的详细说明；模块职责和两条主流程统一见
[感知总说明](README.md)，人工配准参数见 [分类接口文档](registration/MANUAL_REAL.md)。

**当前工作区状态：**旧 `calibration/` 文件已删除，真机入口已将旧标定导入延迟到使用时，
不影响设备枚举、采集和人工配准。下文 `calibrate` / `--calibration` 是保留的历史
固定外置相机接口说明，当前仍不可用；调用时会提示缺失旧标定模块。

**末端相机的当前入口**为 `python -m perception.real manual`，统一支持相机拍摄保存 PLY、
选择已有 PLY、人工选点及多尺度 ICP。通过 `--extrinsics` 导入外部标定流程提供的
`T_base_camera`（相机 → 机器人基座，米），不在此重新计算手眼标定。
末端相机的这个矩阵必须对应拍摄时的机器人姿态；姿态改变后必须更新。
默认输出 `outputs/perception/real/<时间戳>/`，详见 [人工配准接口](registration/MANUAL_REAL.md)。

以下章节保留原有**固定外置相机**的采集、回放、评价和标定工具说明，
其中 `--calibration` 使用带相机身份的旧标定格式，与新 `--extrinsics` 格式不同。

适用于固定在机械臂外的 D405。当前流程为：采集并保存原始数据，随后离线配准、检查与评价。
入口是 `python -m perception.real`，不创建 MuJoCo 场景，也不向机械臂发送指令。

## 1. 安装与设备检查

以下命令均在项目根目录的 PowerShell 运行：

```powershell
$py = '.\.venv\Scripts\python.exe'
& $py -m pip install -r requirements-real-camera.txt
& $py -m perception.real devices
```

SDK 仅在设备枚举、采集时导入。离线回放、重选 ROI、评价和标定不需要相机 SDK。
连接多台 D405 时必须用 `--serial` 或配置的 `camera.serial` 指定设备。
无设备、帧超时、分辨率不支持时会报错，不会用仿真数据替代。

## 2. 采集一组真实数据

### 单独读取 D405 深度/彩色外参（无需 ROS）

插上 D405 并关闭占用设备的 Viewer/采集程序，在项目根目录执行：

```powershell
& D:\python\python.exe -m perception.acquisition.camera_extrinsics
```

多台相机追加 `--serial 实际序列号`；`--config` 可指定采集配置中的流参数。
输出自动保存到 `outputs/perception/camera_extrinsics/<时间戳>/`，也可用 `--out` 指定空目录。
`camera_extrinsics.json` 保存序列号、固件、内参、SDK 原始外参、矩阵及来源；矩阵另存 NPY/TXT，平移单位米。

- `T_color_optical_depth_optical_m` 是设备 SDK 读取的深度光学帧到彩色光学帧转换。
- `T_color_frame_depth_optical_ros_convention_m` 是在此基础上按标准 RealSense ROS 帧轴向推导的候选值。
- 后者不是现场读取的 ROS TF，报告明确标注未验证；需在手眼标定的 ROS 机器上核对实际帧定义。
- 这不是相机到机器人基座的外参，不能直接作为 `--extrinsics` 配置。

ROS1 上可在相机节点运行时查看：

```bash
rosrun tf tf_echo camera_color_frame camera_depth_optical_frame
```

实际帧名以该设备 TF 为准。须确认手眼标定与当前相机序列号、安装状态一致；相机内部外参可在稍后读取，
机器人末端位姿则必须对应牙模拍摄时刻。

参考：[SDK 外参说明](https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/)、
[ROS1 驱动静态 TF 实现](https://github.com/IntelRealSense/realsense-ros/blob/ros1-legacy/realsense2_camera/src/base_realsense_node.cpp)。

使用与口扫文件对应的实体牙模；先固定一个颌，保证该颌在曝光及采集过程中静止。
调整照明、距离和视角，在预览中确认牙面深度完整，再保存。

```powershell
& $py -m perception.real capture `
  --jaw mandibular --out outputs/real_capture_001 `
  --select-roi --preview
```

先显示实时画面，按 S 或空格冻结当前帧，再在该帧上框选牙模区域，Enter 确认并保存。
取消 ROI 框选返回实时预览；实时预览中 Q 或 Esc 退出，关闭窗口也会取消采集。
单独使用 `--select-roi` 也会先显示实时预览；仅使用 `--preview` 时按 S/空格直接保存。
多帧采集只在第一帧选择 ROI，后续复用该 ROI；未加 `--preview` 时后续帧自动保存。
框选和预览都使用**原生深度图像素网格**上的着色点图，不是原始 RGB 像素网格。
GUI 点选需要可用的 Windows 图形桌面；无窗口采集可以省略这两个参数。

默认配置是 `config/real_camera_registration.json`：

- 默认请求 640×480、30 FPS，跳过 30 帧供曝光稳定；这些参数可改。
- 初始深度筛选范围为 70–300 mm，需要按实际安装位置修改。
- `roi=null` 表示整个深度图；指定时格式为 `[x0,y0,x1,y1]`，右下界不包含。
- `capture_outlier_neighbors=0` 可关闭采集后的统计离群点过滤。
- ROI 和距离范围只排除部分背景，不等同于牙齿语义分割。

自动算法默认体素为 0.5 mm，未显式设置时 ICP 对应距离也派生为 0.5 mm。
这组初始值尚未经过 D405 实测调优；需要按实际点间距、噪声和初值误差调整
`automatic_registration` 中的体素、邻域与对应距离，并保留每次实验的完整配置。

一次保存多帧，便于静态重复性实验：

```powershell
& $py -m perception.real capture --jaw mandibular `
  --out outputs/real_static_session_001 --count 10 --select-roi
```

多帧保存到 `capture_0001/` 等子目录，并保存 `session.json`。文件写入与采集串行，
`--count` 不是按请求 FPS 录制视频；中途失败时 session 标为未完成，并保留已经保存的帧。

每个采集目录包含：

| 文件 | 含义 |
| --- | --- |
| `rgb.png` | 原生彩色图像，未强行重采样到深度图 |
| `depth_raw.npy` | 原始 Z16 深度值 |
| `depth_m.npy` | 原始值乘设备实测 depth scale，单位 m |
| `points_camera_mm.npy` | SDK 生成的 H×W×3 原生深度光学坐标点图，单位 mm |
| `point_colors.npy` / `color_valid_mask.npy` | SDK UV 映射后的点颜色及颜色有效性 |
| `roi_mask.npy` / `valid_mask.npy` | 深度与 ROI 筛选结果 / 统计过滤后的最终结果 |
| `camera_raw_mm.ply` | 所有有效原始深度点，尚未裁剪 |
| `camera_roi_mm.ply` / `camera_mm.ply` | ROI 点云 / 最终配准输入点云 |
| `depth_preview.png` / `selection_preview.png` | 深度与最终筛选预览 |
| `capture_metadata.json` | 序列号、固件、SDK、真实内参和畸变、深度比例、时间戳、帧号、配置与文件 SHA-256 |

几何坐标固定为**深度光学坐标**：X 向右、Y 向下、Z 向前。颜色通过 SDK 纹理坐标映射；
无颜色对应的点使用中性灰色，仍可参与几何配准。元数据中的 `T_color_depth_m` 将深度坐标
转换到彩色坐标。原始时间戳与时间域分别保存，未假设设备时间与主机或机器人时钟同步。

## 3. 回放配准

先确认口扫文件和牙模属于同一对象，单位为毫米。支持 PLY/PCD 点云或 STL 网格；
算法只估计刚体变换，不会修正比例尺错误。

```powershell
& $py -m perception.real register `
  --capture outputs/real_capture_001 `
  --scan assets/source_cad/teeth/mandibular_scan.ply `
  --out outputs/real_registration_001 --method automatic --view
```

`--scan` 必须替换为与实物匹配的口扫。仓库自带的文件仅适用于它对应的实物。
省略 `--view` 可以无窗口运行自动配准。结果目录必须不存在或为空，避免覆盖实验记录。

人工基线使用相同采集，换一个结果目录：

```powershell
& $py -m perception.real register `
  --capture outputs/real_capture_001 `
  --scan assets/source_cad/teeth/mandibular_scan.ply `
  --out outputs/real_registration_manual_001 --method manual --view
```

人工窗口左侧为相机点云、右侧为原始口扫。按提示交替选择至少六对分布良好的对应点。
也可使用 `--landmarks` 读取此前保存的 `landmark_correspondences.json`，但采集 ID 与口扫
SHA-256 必须一致。人工基线不是独立真值。

输出根目录包含运行配置和 `run_report.json`；`registration/` 中包含：

- `T_camera_scan_initial.npy/.txt` 与 `T_camera_scan_estimated.npy/.txt`。
- `scan_registered_camera_mm.ply`、三方向 `registration_overlay.png`。
- 配准指标、参数、相机身份、输入指纹及结果指纹：`registration_report.json`。
- 独立评价与质量状态：`validation_report.json`。
- 人工模式另保存标志点；自动模式不保存人工点。

矩阵方向始终是**口扫毫米坐标 → 深度相机毫米坐标**。Fitness 与 ICP RMSE 的评价方向
是局部相机点云到完整口扫，报告中明确记录。自动方法的 `elapsed_s` 是算法耗时；
`registration_wall_time_s` 还包含人工点选等待，完整运行时间见 `run_report.json`。

可选 `--line` 读取与口扫处于同一坐标系、具有 `x,y,z` 毫米列的牙龈线 CSV，输出相机坐标线。
没有提供外参时，不会生成基座或世界坐标，不会把单位矩阵当作已标定外参。

### 离线修改 ROI 和过滤参数

修改配置中的 ROI、深度范围、统计过滤参数后，无需重新连接相机：

```powershell
& $py -m perception.real recrop `
  --capture outputs/real_capture_001 `
  --config config/real_camera_registration.json `
  --out outputs/real_capture_recrop_001
```

随后对新目录运行 `register`。重处理保留最初的 `acquisition_id`、时间戳和父采集关系，
但分配新的 `capture_id`，因此旧点选文件不能直接冒充新采集的点选文件。
仅修改 `register --config` 的 ROI 不会重裁已保存点云；要使用 `recrop`。

## 4. 独立检查点和质量限值

在配准完成后选取至少三个不共线、未作为人工初值的对应检查点：

```powershell
& $py -m perception.real checkpoints `
  --registration outputs/real_registration_001/registration `
  --out outputs/checkpoints_001.json
& $py -m perception.real evaluate `
  --registration outputs/real_registration_001/registration `
  --checkpoints outputs/checkpoints_001.json `
  --out outputs/real_registration_001/withheld_validation.json
```

报告输出每点误差、均值、RMS、P95 和最大值，均为毫米。也可在下一次回放时用
`register --checkpoints outputs/checkpoints_001.json`，检查点只在优化完成后读取。

检查点 JSON 使用 `capture_id`、`source_hashes` 绑定数据，点数组字段为
`source_scan_mm`、`target_camera_mm`，并声明 `independent_of_registration=true`。
`measurement_method` 描述人工辨认或外部测量方法。程序会拒绝直接复用人工配准点；
无法判断人工选点是否可靠、是否选对牙位，也不包含测量不确定度。
这些点仍可能位于 ICP 使用过的牙面上，因此它们是未参与人工初值的对应点检查，
不自动构成独立传感器测量或可追溯绝对精度。

质量限值在配置的 `quality` 中设置：
`min_fitness`、`max_inlier_rmse_mm`、`max_checkpoint_p95_mm`。
默认均为 null；先收集实验数据，再按误差预算确定限值。

| 状态 | 含义 |
| --- | --- |
| `not_configured` | 未配置验收限值，不代表通过精度验收 |
| `not_evaluable` | 缺少某个限值要求的证据，例如未提供检查点 |
| `failed` | 至少一个限值未满足 |
| `passed_configured_limits` | 满足已配置限值；仍不能仅此证明牙位正确 |

CLI 返回码：0 表示处理完成且没有限值失败，1 表示输入/设备/算法错误，2 表示限值失败
或证据不足。默认未配置限值时返回 0，但报告保留 `not_configured`。
所有报告均保留 `robot_execution_allowed=false`。

## 5. 静态重复性

让牙模与相机保持固定，对多个独立采集分别配准，再比较结果：

```powershell
& $py -m perception.real repeatability --static-scene `
  --registrations outputs/real_registration_001/registration outputs/real_registration_002/registration `
  --out outputs/static_repeatability_001.json
```

报告给出相对于第一组结果的平移/旋转偏差与 RMS。平移差定义在口扫坐标原点，
不是整条牙龈线的最大位移。相机、牙模、口扫、颌和坐标系必须相同；牙模移动后
不能把矩阵变化当作重复性误差。重复处理同一原始帧只反映算法或预处理重复性，
程序用 `acquisition_id` 区分这一情况。重复性不是绝对精度。

## 6. 可选固定相机外参

若只验证配准，可暂不标定。需要基座坐标时，把标定板刚性装到末端，采集不同位置、
不同旋转轴的机器人位姿和板位姿。输入格式如下，数组需填入实际测量的 4×4 齐次矩阵：

```text
{
  "units": "m",
  "camera_serial": "实际 D405 序列号",
  "optical_frame": "depth",
  "samples": [
    {"T_base_gripper_m": 4×4矩阵, "T_camera_target_m": 4×4矩阵},
    ...至少三组，实际建议采集更多分布良好的姿态
  ]
}
```

这里的板位姿必须是**板 → 深度相机**。若 PnP 得到彩色相机坐标的板位姿，先计算：
`T_depth_target_m = inverse(T_color_depth_m) @ T_color_target_m`。
机器人位姿、板模型尺寸与平移都统一为米。当前实现读取已有位姿记录，
未实现机器人通信、标定板检测/PnP 或同步采样，这些输入需要在实验端准备。

```powershell
& $py -m perception.real calibrate --samples outputs/calibration_samples.json `
  --validation-samples outputs/calibration_holdout.json `
  --out outputs/base_camera_calibration.json
```

使用 OpenCV PARK 法求 eye-to-hand，检查非平行旋转轴，并分别报告拟合闭环残差与
未参与拟合的姿态验证残差。`--validation-samples` 可省略，此时未声明已完成独立验证。
没有独立验证或标定板/机器人输入误差较大时，闭环残差低也不能保证绝对定位精度。

随后在 `register` 命令加 `--calibration outputs/base_camera_calibration.json`，会校验
序列号、光学坐标系和单位，额外输出 `T_base_camera_m.npy` 与 `T_base_scan_m.npy`。
如同时提供 `--line`，会额外输出牙龈线基座米坐标。

```text
T_camera_scan_m = T_camera_scan_mm，仅将平移除以 1000
T_base_scan_m = T_base_camera_m @ T_camera_scan_m
```

相机支架移动后需要重新标定；序列号校验无法检测支架是否移动。

## 7. 当前限制与实时化路径

当前方法的限制：

1. 每组数据重新执行 FPFH/RANSAC 全局搜索及 ICP，口扫法线与特征重复计算；
   耗时会随点数、重叠与几何歧义变化，没有实时帧率或延迟保证。
2. 牙形重复、小视野、遮挡和低重叠可能造成错误牙位；高 Fitness、低 RMSE 仍可能出错。
   当前没有多候选牙位判别或经过验证的语义分割。
3. ROI 与深度筛选无法完整排除唇、舌、器械；反光、低纹理、湿润牙面和掠射角会
   影响真实深度质量。统计离群点过滤不能恢复缺失或系统偏差的深度。
4. 刚体算法不描述软组织变形；上下颌有相对运动时必须分别注册。
5. 当前没有连续运动估计、失踪检测、全局重定位、同步机器人状态或过期结果失效机制。
6. PNG/NPY/PLY 写盘、全文件哈希和 Matplotlib 绘图在离线流程中串行执行，不能放入
   将来的每帧关键路径。结果引用原始文件绝对路径，移动实验目录后通常需要重新回放。
7. 软件测试使用合成点云与 SDK 替身，不代表 D405 实测精度或临床使用条件下的性能。

后续实时化建议采用“全局初始化 → 局部跟踪 → 失效重定位”：

| 要补的能力 | 具体工作 |
| --- | --- |
| 持续采集 | 保持 RealSenseCamera 打开，独立采集线程，只保留最新帧；记录丢帧与帧龄，防止排队延迟 |
| 固定模型缓存 | 预计算口扫多尺度点云、法线、FPFH 和搜索结构，避免每帧重算 |
| 局部跟踪 | 使用上一帧有效变换或运动预测初始化局部多尺度 ICP，匹配可见区域 |
| 质量与失效 | 检查覆盖率、内点分布、残差、位姿突跳和跟踪退化；失败时使结果失效并触发全局重定位 |
| 牙位与遮挡 | 跟踪牙面 ROI，处理器械与软组织遮挡，加入独立标志或牙位约束解决相似牙形歧义 |
| 时间与坐标 | 建立相机/主机/机器人时间映射；每个位姿携带采集时间、坐标系、序列号和有效期限 |
| 与控制交接 | 定义有效性、丢失、超时和重定位状态；实测端到端延迟及 P95，再确定控制消费频率 |
| 异步记录 | 将点云、图像、哈希和绘图移到后台，按需保存关键帧；评估 CPU/GPU 优化是否必要 |

已为实时化保留的基础是 `RealSenseCamera.read() -> RGBDFrame`：它不写盘；
`save_real_capture()` 单独负责持久化；`register_clouds_automatic()` 接受内存点云。
这些接口可复用，但当前版本没有声称实现实时跟踪。

参考：[RealSense 投影与坐标文档](https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/)、
[SDK 点云示例](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/opencv_pointcloud_viewer.py)、
[OpenCV 手眼标定文档](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)。

## 8. 软件验证

```powershell
& $py -m unittest tests.test_real_registration tests.test_global_registration `
  tests.test_global_rgbd tests.test_perception_frames tests.test_paired_landmark_picker -v

# 仿真流水线联测，另依赖当前 planning 模块的完整可导入状态
& $py -m unittest tests.test_automatic_registration -v
```

先做静态牙模实测，再以独立对应点或具有已知刚性关系的外部测量评估误差；
在这一基础上判断算法误差、深度误差与外参误差各占多少。
