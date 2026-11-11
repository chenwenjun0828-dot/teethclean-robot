# 交互式洁治器姿态查看器

在项目根目录启动，自动打开浏览器，无需 Node.js 或联网下载网页依赖：

```powershell
.\.venv\Scripts\python.exe -m planning.visualization.pose_angles --jaw-opening-deg 20
```

也可以从 config 文件读取初始角度、区域、离面距离、张口角、牙模范围和端口：

```powershell
.\.venv\Scripts\python.exe -m planning.visualization.pose_angles --params config/planning_pose_angles.json
```

命令行可覆盖文件，例如追加 `--perception-result <感知目录>`、`--model-scope both` 或 `--port 8766`。
该文件设置启动时的初始姿态，页面调节不会自动改写 config。

默认从附带的下颌数据准备 `outer_front`，可在页面切换六个区域。
已有感知结果时建议直接读取，以便与后续规划使用相同的区域参考系和配准旋转：

```powershell
.\.venv\Scripts\python.exe -m planning.visualization.pose_angles `
  --perception-result outputs/tutorial/manual/regions/outer_front `
  --pose-angles-deg 6.37 13.26 -0.4 --standoff-mm 1.8 `
  --jaw-opening-deg 20 --model-scope both
```

## 控件与参数

| 控件 | 行为 |
| --- | --- |
| 三个滑块及数字框 | `fixed_z_angle`、`fixed_y_azimuth`、`tool_x_flip`，各为 -180° 至 180° |
| 姿态参考 | `simple` 区域固定参考，或 `continuous` 局部牙面参考，与 planning 同名参数一致 |
| 离面距离 | TCP 沿路径法线偏移 0–30 mm，数值传给 planning 的 `--standoff-mm` |
| 路径位置 | 在处理后的感知参考线上移动 TCP；区域固定模式保持同一姿态 |
| 牙面选点 | 勾选后点击牙模，使用展示网格的局部法线；拖动仍旋转视角 |
| 恢复路径参考点 | 返回区域默认路径点，保留三个角度 |
| 牙模范围 | 局部牙冠、完整下颌、上下颌；按启动时的仿真张口角摆放 |
| 完整器械 | 切换实际工具全网格与距 TCP 65 mm 内的工作端显示 |
| 适配视角 / 角度归零 | 相机适配可见模型，或将三个姿态角归零 |
| 复制 / 导出 | 复制三个角度，或下载包含姿态矩阵、参考点和参数的 JSON |

鼠标拖动旋转、滚轮缩放。调整角度或路径位置时保持相机视角，切换牙模范围或主动适配时重新取景。
`--port 8765` 指定本地端口；占用时自动选择可用端口并打印网址。`--no-browser` 禁止自动打开浏览器。
按 Ctrl+C 停止本地服务。前端依赖及许可证保存在 `pose_angles_web/vendor/`，需一并上传 GitHub。

## 用于规划

滑块通过本地 Python 接口调用 `planning.pose_search.constraints.target_at`，
区域固定模式复用 `planning.simple.orient_reference`，工具网格以实际 MuJoCo TCP 为旋转中心。
复制的三角度可直接用于：

```powershell
.\.venv\Scripts\python.exe -m planning `
  --perception-result outputs/tutorial/manual/regions/outer_front `
  --pose-angles-deg 6.37 13.26 -0.4 --planning-mode simple `
  --standoff-mm 1.8 --jaw-opening-deg 20 --check-mode full `
  --headless --out outputs/planning/selected_pose
```

查看器显示**未做碰撞修正的请求姿态**，不求 IK、不验收碰撞或动力学。
完整规划可能采用 ±5° 修正；应再用 `python -m planning.view --plan <目录>` 回放实际结果。
牙面选点只用于比较局部姿态，不会改写感知路径，其局部法线也不等于区域拟合平面。
选点后复制的角度在区域固定规划中仍按区域参考系解释，需回到路径模式核对。
导出 JSON 标记 `preview_only=true`、`execution_ready=false`，不能作为控制模块的轨迹输入。
