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

## 真机 Session 选参与规划

在同一页面调节姿态、提交规划、检查结果，再调整重试：

```powershell
$py = ".\.venv\Scripts\python.exe"
& $py -m planning.visualization.pose_angles `
  --perception-session assets/perception/sessions/20261113_220209 `
  --session-camera-frame color_optical --jaw mandibular --region outer_front `
  --model-scope both
```

页面与规划使用同一套 session 场景加载，恢复双颌实测位置和拍摄关节角。
Session 输入不能与 `--perception-result` 或 `--jaw-opening-deg` 混用；
相机解释在启动时固定，切换解释需要重启。`color_optical` 仍是待现场核实的解释，结果仅用于仿真。

1. 调节三个角度、离面距离和姿态模式，用路径位置滑块检查沿线姿态。
2. 设置沿线/进出场速度，高级参数可调整姿态采样间距和逐位置微调。
3. “可达性检查”只验证几何与连续 IK；“完整规划”额外运行完整避碰、时序和动力学验收。
4. 查看实际 TCP 路径、结果、诊断日志和本次参数。完整机械臂动画使用结果区的回放命令。
5. 修改参数后旧结果标为待重新规划。每轮结果独立保存，可取消当前任务后重试。

本轮只开放非接触规划，默认额外避障间隙为 `0 mm`，接触和相交仍被拒绝；离面距离必须严格大于零。
旧轨迹继续使用保存的间隙。路径位置只改变预览点，不裁剪规划区域；
牙面选点模式下禁止提交，需恢复路径参考并退出选点。
区域固定和局部牙面模式允许既有的有界角度修正；关闭逐位置微调不关闭整区修正。
页面显示请求姿态，实际路径由保存轨迹采样和 FK 得到；显示采样不构成碰撞证书。

每次提交固定参数快照，后台通过当前 Python 的 `-m planning` 子进程运行，只允许一个活动任务。
产物在 `outputs/interactive_planning/<任务>/`：`parameters.json`、`planner.log`、`task.json` 和
`plan/` 内现有规划产物。导出的规划配置可用 `python -m planning --params <文件> --out <新目录>` 复现，
不同于工具栏的单点姿态 JSON。页面重载可读取本服务最近任务，服务重启不恢复历史任务。

完整通过要求现有 `load_verified` 检查成功；session 显式张口角覆盖仍会被拒绝。
所有结果保留 `robot_execution_allowed=false`，本服务不连接或驱动真机。

回归入口：

```powershell
& $py -m unittest planning.test_interactive_planning planning.test_pose_angles planning.test_view tests.test_perception_session tests.test_planning_parameters -q
# 已有 Playwright 的 Node 环境下，先启动上面的 session 服务
node planning/tools/check_interactive_planning.cjs http://127.0.0.1:8765/ outputs/interactive_planning/browser_checks
```

浏览器回归会创建一次真实可达性规划，并提交、取消一次完整规划；追加 `--full` 则等待完整规划验收，
并使用 0.25 mm 姿态采样间距检查完整结果和实际 TCP 显示。所有产物保存到新的任务目录。
本次 session 参数与验证结果见 [交互规划验收记录](../../docs/integration/2026-11-15-session-interactive-planning.md)。

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
| 姿态参考 | `simple` 区域固定；`local_surface` 局部牙面并允许规划微调；`continuous` 局部严格角度，与 planning 同名参数一致 |
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
区域固定与局部牙面模式复用 `planning.simple.orient_reference`，工具网格以实际 MuJoCo TCP 为旋转中心。
复制的三角度可直接用于：

```powershell
.\.venv\Scripts\python.exe -m planning `
  --perception-result outputs/tutorial/manual/regions/outer_front `
  --pose-angles-deg 6.37 13.26 -0.4 --planning-mode simple `
  --standoff-mm 1.8 --jaw-opening-deg 20 --check-mode full `
  --headless --out outputs/planning/selected_pose
```

滑块姿态预览显示**未做碰撞修正的请求姿态**，该预览本身不求 IK、不验收碰撞或动力学。
`simple` 和 `local_surface` 的完整规划可能采用 ±5° 修正；局部模式的修正相对每个位置的牙面参考系。
`continuous` 严格保持请求角度。应再用 `python -m planning.view --plan <目录>` 回放实际结果。
牙面选点只用于比较局部姿态，不会改写感知路径，其局部法线也不等于区域拟合平面。
选点后复制的角度在区域固定规划中仍按区域参考系解释，需回到路径模式核对。
导出 JSON 标记 `preview_only=true`、`execution_ready=false`，不能作为控制模块的轨迹输入。

局部牙面 `50, 0, 50` 的配套查看器和规划配置：

```powershell
.\.venv\Scripts\python.exe -m planning.visualization.pose_angles --params config/planning_pose_angles_local_surface.json
.\.venv\Scripts\python.exe -m planning --params config/planning_local_surface.json --out outputs/planning/my_local_surface
```

两者读取同一前牙感知结果，使用 1.8 mm 离面距离和 20° 张口角。
