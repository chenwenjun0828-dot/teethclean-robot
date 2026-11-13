# 真机人工选点配准：相机采集或现存 PLY

主入口：`python -m perception.real manual`。
兼容入口：`python -m perception.registration.manual_real`，参数和实现相同。
所有输入统一走“人工至少六对对应点 → 鲁棒刚体初始化 → 三尺度鲁棒 Point-to-Plane ICP”。
移植自旧项目的 `manual_real.py` 和 `compare_real_sim.run_assisted`，移除了仿真实验依赖。
ICP 三阶段的体素/对应距离为 `1/3`、`0.5/1.5`、`0.25/0.75` mm，Tukey 核，各最多 100 次迭代。

先在项目根目录执行 `conda activate rl`。新机器按 `requirements-real-camera.txt` 安装依赖；
真实相机采集额外需要 RealSense SDK 和已连接的 D405。选择文件的弹窗需要 Python tkinter。

## 1. 使用导入的历史样例

```bash
python -m perception.registration.manual_real --source sample --view
```

默认读取 `assets/real_captures/dental_phantom_legacy.ply` 和
`assets/source_cad/teeth/mandibular_scan.ply`。仅 sample 预设自动沿用历史的米制/OpenGL
坐标假设和固定下颌 ROI。样例来源及哈希见 [资产说明](../../assets/real_captures/README.md)。
`--prepare-only` 只转换和裁剪并保存 `camera_mm.ply`，不打开选点窗口，便于先检查输入。

## 2. 选择现存 PLY

```bash
python -m perception.registration.manual_real --source ply --units mm --frame cv --view
```

未指定路径会弹出文件选择框；也可明确传入路径：

```bash
python -m perception.registration.manual_real --source ply \
  --camera /实际路径/camera.ply --units m --frame opengl \
  --scan /实际路径/matching_scan.ply --view
```

现存 PLY 必须明确填写单位 `m/mm` 和坐标轴 `cv/opengl`，程序不推断。
CV 坐标为 X 向右、Y 向下、Z 向前；OpenGL 输入转换时翻转 Y/Z。
通用 PLY 默认不裁剪；可使用 `--roi-mm XMIN YMIN ZMIN XMAX YMAX ZMAX`，
范围作用于转换后的 CV 相机毫米坐标。

## 3. 连接相机采集 PLY，再人工配准

```bash
python -m perception.real devices
python -m perception.registration.manual_real --source camera \
  --scan assets/source_cad/teeth/mandibular_scan.ply --view
```

此入口复用 `perception.real capture`：先显示实时画面，按 S/空格冻结当前帧，
再在该帧上框选深度网格上的牙模 ROI，Enter 确认并保存。取消 ROI 框选返回实时预览；
实时预览中 Q/Esc 退出。保存 RGB-D、原始点图、元数据和 `capture/camera_mm.ply` 后，
自动进入人工配对。采集失败或取消会停止。多台设备用 `--serial` 指定；
`--config` 指定真机采集配置，`--jaw` 可选上下颌。相机输入直接使用 SDK 的 CV 毫米坐标，
无需设置 `--units/--frame`。详见 [真机采集文档](../REAL_CAMERA.md)。

## 4. 选点与输出

可从同一份已保存完整采集重新框选另一颌，无需相机连接：

```powershell
python -m perception.real manual --source capture --capture <旧运行目录>/capture --jaw maxillary --picker legacy --view
```

此模式显示原始深度网格着色图，框选上颌后按 Enter，自动生成新的采集和上颌配准。
新目录保留原始 acquisition_id、时间戳、父采集路径及父颌别，分配新 capture_id；原始目录不改写。
必须使用包含未裁剪深度/点图的采集目录，单独的下颌 PLY 或 PNG 不能恢复上颌几何。

左侧为相机点云，右侧为口扫。Shift+左键依次选左、右对应点，至少六对，分散覆盖牙面。
Shift+右键/Backspace 撤销；点击 Finish & Register 或按 Enter/Q 完成，Esc 取消。
`--view` 控制最终叠加窗口，人工模式始终需要选点窗口（除非传入已有标记）。

如果并排窗口未响应或无法旋转，可追加 `--picker legacy` 使用兼容窗口。
该模式先显示相机点云，Shift+左键依次选至少六点，记住顺序后按 Q；
随后显示口扫，按相同顺序选择相同数量的对应点，再按 Q 执行配准。
第二步会同时保留第一步的相机参考图，以 1、2、3… 标出选点顺序；
参考图也保存为 `registration/camera_landmarks_reference.png`，可随时打开对照。
参考图使用独立进程显示，不与点云窗口混用事件循环。第一步完成时同时保存
`registration/camera_landmarks_reference.json`；若第二步意外退出，可追加
`--picker legacy --camera-landmarks <旧目录>/registration/camera_landmarks_reference.json`
直接重选口扫点，程序会核对两份点云的指纹。
不按 Shift 时左键拖动旋转，滚轮缩放，Shift+右键撤销。关闭窗口也会提交当前选点，
点数不足或两边数量不同会停止配准。兼容模式与并排模式输出相同格式的对应点和矩阵。

默认输出 `outputs/perception/real/<时间戳>/`；可用 `--out` 指定不存在或为空的目录。
历史 `outputs/manual_real/` 不迁移或删除。

```text
capture/                              仅相机模式生成，保存采集 PLY、RGB-D、元数据
transforms/                           米制矩阵、外参副本与 metadata.json
paths/                                相机/基座路径及 metadata.json
run_report.json                       本次运行结果与坐标输出清单
registration/
  input.json                          输入哈希、单位、坐标轴转换和 ROI
  camera_mm.ply                       归一化和裁剪后的相机点云
  landmarks.json                      人工对应点及输入哈希
  T_camera_scan_initial.npy/.txt       人工初始化，口扫毫米 → 相机毫米
  T_camera_scan_estimated.npy/.txt     ICP 最终变换，仅精配准成功后生成
  scan_registered_camera_mm.ply       配准后的口扫点云
  registration_overlay.png            静态叠加图
  registration_report.json            成败、各尺度指标、标记残差
  line_camera_mm.csv                  可选：传入 --line 后导出的路径点
```

`--line /实际路径/line.csv` 接收与口扫同坐标系的 `x,y,z` 毫米 CSV。
Python 文件接口为 `register_manual_clouds(camera_path, scan_path, output_dir, units=..., frame=...)`。
可用 `--landmarks <旧输出>/registration/landmarks.json` 在新的输出目录复用标记；
程序核对口扫和处理后相机点云的哈希，输入或裁剪改变后必须重新选点。

## 5. 导入外参并输出基座路径

```bash
python -m perception.real manual --source sample \
  --line assets/perception/mandibular/gingival_line_points.csv \
  --extrinsics config/camera_to_base.example.json --view
```

文件格式：

```json
{
  "T_base_camera": [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]],
  "translation_unit": "m",
  "status": "placeholder"
}
```

该矩阵将 **CV 相机坐标转换到机器人基座坐标**。替换实测矩阵后设置 `status=measured`。
相机装在末端时，外参必须对应拍摄时的机器人姿态；本接口不计算手眼标定，也不读取机器人位姿。
检查 4×4 形状、有限值、无缩放的合法旋转、齐次末行、米制单位及状态。

`transforms/T_camera_scan_m.npy` 为米制口扫到相机变换。
有实测外参时另存 `T_base_camera_m.npy`、`T_base_scan_m.npy`；组合顺序为
`T_base_scan_m = T_base_camera_m @ T_camera_scan_m`。
同时保存所加载的原始外参字节到 `transforms/extrinsics_used.json`，记录来源及 SHA-256。

占位外参生成 `T_base_camera_placeholder_m.npy`、`T_base_scan_placeholder_m.npy` 和
`paths/line_base_placeholder_m.csv`；状态同时记录在元数据、报告和 CSV 注释头中。
实测状态生成 `paths/line_base_m.csv`。相机路径始终为 `paths/line_camera_m.csv`。
路径只在提供 `--line` 时生成，单位米，CSV 的 `#` 注释头标明坐标系和状态，
可用 `numpy.loadtxt(path, delimiter=',')` 读取。原有 `registration/line_camera_mm.csv` 继续保留。
无外参时不生成基座坐标文件。

Python 接口 `register_manual_clouds(..., extrinsics=外参JSON路径, run_dir=运行根目录)`
使用同一坐标导出实现；不传 run_dir 时额外输出放在指定的 output_dir 下。
允许读取旧脚本的 `roi_sha256` 标记格式，并核对处理后点云及口扫指纹。

精配准指标不等同于独立精度验证，`measured` 仅表示用户声明外参为实测值。
旧固定外置相机的独立评价与标定入口仍保留为 `perception.real register --method manual`
及其 `--calibration/--checkpoints` 接口。当前入口不驱动机械臂。

软件验证：`python -m unittest tests.test_perception_outputs tests.test_manual_real tests.test_real_registration -v`。
