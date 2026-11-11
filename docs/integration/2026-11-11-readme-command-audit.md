# README 命令与干净环境验收

日期：2026-11-11。目标是核对主 README 中的安装、完整流程及独立模块命令，
修复首次克隆和安装后会遇到的问题。以下初次命令验收阶段未提交或推送代码，
同日 Git 发布前的真实克隆验证见文末补充。

## 环境与隔离方式

- Windows x64，CPython 3.11.9，PowerShell。
- 从当前工作区复制 Git 可提交文件到独立目录，初始 501 个文件，随后同步本轮修复。
- 副本不含原工作区的 `.venv/`、`outputs/`、缓存或 `.git/`；新建虚拟环境，禁用系统 site-packages。
- 全部感知与轨迹输入在副本中重新生成，没有复制旧采集、配准矩阵、感知区域或轨迹。
- 实际验证目录：`outputs/readme_audit_20261111/clone/`；命令、退出码和耗时记录在上级的 `commands.json`，每条命令有单独日志。

副本验证的是当前待提交文件集合；GitHub 上的新克隆能否复现，还取决于发布时是否完整提交这些代码、配置和必需资产。
现有工作区有大量资产迁移及未跟踪文件，不能只提交本轮修改的 README 和 requirements。

## 安装与依赖修正

原 `requirements.txt` 只安装基础 MuJoCo 依赖，不能支撑 README 的感知和规划命令。
现调整为完整运行入口，通过 `requirements-perception.txt` 安装 Open3D、Matplotlib、SciPy、
scikit-learn、python-fcl、TOPPRA 和 ECOS；基础环境保存在 `requirements-simulation.txt`。
NumPy 限制在 1.x，OpenCV 限制在 4.12 之前，避免直接依赖约束冲突。

在副本新建环境后，实际执行：

```powershell
py -3.11 -m venv .venv
$py = ".\.venv\Scripts\python.exe"
& $py -m pip install --upgrade pip
& $py -m pip install -r requirements.txt -c requirements-perception-lock.txt
& $py -m pip check
& $py -m pip install -r requirements-real-camera.txt -c requirements-perception-lock.txt
& $py -m pip install -r requirements-animation.txt -c requirements-perception-lock.txt
& $py -m pip check
```

两次 `pip check` 均通过。可选包实装为 `pyrealsense2==2.59.1.11053` 和 `imageio-ffmpeg==0.8.1`；
未连接相机，不将 SDK 安装成功解释为 D405 采集已验证。
GIF 导出已在安装这两个可选包之前使用完整运行依赖通过。

另以 `pip wheel --no-cache-dir --no-deps toppra==0.6.3` 成功从源码构建 Windows / CPython 3.11 wheel，
随后将该 wheel 强制安装进副本新环境，`pip check`、19 项重计时重点测试、90 s 完整重计时和动力学验收再次通过。
这项检查不依赖已有 TOPPRA 编译包缓存。开发者仍须按 README 安装 C++ Build Tools。

## 批处理命令结果

`commands.json` 中 29 项检查均得到预期退出码。下表按模块合并重复参数写法；
两个 PowerShell 内嵌 Python 块直接从 README 提取并执行。

| README 入口 | 结果 |
| --- | --- |
| `pipeline.run --params ... --dry-run` | 通过，输入与配置校验 |
| `pipeline.run --params ... --headless` | 完整通过，单独日志 `pipeline.log` |
| `perception.gingival_extraction --no-view` | 1710 点、1693 条线段，导出成功 |
| 仿真 RGB-D 采集内嵌 Python | 成功生成 RGB、深度、点云及元数据 |
| 自动配准与世界变换内嵌 Python | 成功，ICP RMSE 约 0.1485 mm |
| `perception.registration --no-view` | 独立双点云配准成功 |
| `perception.tooth_segmentation` | 匹配的附带分割预测校验通过，无模型推理 |
| `perception.regions` | 六区感知结果成功导出 |
| `pipeline.path_viewer`，known / automatic，无窗口 | 均成功导图 |
| `planning --params ... --dry-run` | 通过 |
| `planning --params ...` | 完整验收通过 |
| `planning --check-mode geometry_only` | 可达性预览通过，保持不可执行 |
| `planning --check-mode full`，逐项 CLI | 完整验收通过 |
| 查看器选角后的独立规划命令 | 完整验收通过 |
| `planning.preview` / `sweep_preview` / `segment_map` | PNG/SVG 全部导出成功 |
| `planning.view --snapshot`，时间和路点定位 | 完整轨迹及候选轨迹均成功截图 |
| `planning.view --gif` | 5 倍速、10 FPS 整条轨迹 GIF 导出成功 |
| `control.position.load_verified` | 完整规划校验通过 |
| `control.position --headless` | 独立位置控制完成 |
| `planning.execute`，原时序 | 成功加载，复用已有无窗口动力学验收 |
| `planning.retime`，90 s | 修复后完整审计与实际动力学通过 |
| `planning.execute`，指定 90 s 新目录 | 校验和执行入口通过 |
| `planning.execute --no-original-timing --duration 90` | 自动生成并验收派生时序，入口通过 |
| `control.force`，前牙区 1 N | 按保护停止，返回预期退出码 2，不算恒力验收通过 |
| `control.force.benchmark`，球-平面 1 N | 通过，返回 0 |

原时序约 66.676880 s，位置执行最大位置误差约 0.087277 mm、旋转误差约 0.016478°、
最小实际间隙约 0.156703 mm。90 s 重计时包括进出场约 11.523414 s、清扫约 78.476586 s，
报告 `execution_ready=true`、`fast_dynamics_verified=true`。

## 重计时修复

原独立重计时入口将源配置的关节速度、加速度和角速度上限从
`0.3 rad/s`、`0.8 rad/s²`、`15 deg/s` 自动改成 `2.2`、`12`、`200`。
本例因此触发 ECOS/TOPPRA 数值失败；相同固定路径保留原运动限制时能成功求解。

`planning.motion_planning.timing.run` 现在保留源限制，仅接受显式设置的 TCP 速度上限和目标时长。
不修改路径、IK 节点、碰撞与动力学验收条件，不自动放宽限制以满足较短时长。
时序配置版本由 3 升为 4，执行缓存不会复用旧策略生成的派生配置。
回归检查同时确认重计时保留源限制、曲线、节点和几何产物。

## 交互入口

- 无窗口完整 pipeline 本机约 57.92 s 完成；另实际运行 README 的 `--no-headless` 完整 pipeline，
  GUI 位置控制正常完成，总耗时约 132.31 s。耗时不是性能保证。
- 单独 `planning.execute --params config/planning_execute.json --plan ... --no-headless` 也实际显示窗口并完成位置控制，
  不依靠自动关闭回调，不复用已有动力学结果。
- 原生 MuJoCo 回放、候选轨迹回放、六区路径窗口实际打开，并通过测试中的库 API 定时关闭。
- Open3D 配准叠加与牙龈线叠加窗口实际打开、渲染并由测试回调关闭。窗口本来会等待人工关闭，不能将此等待判为命令失败。
- 人工配准窗口实际启动后由测试取消，CLI 返回 1 并报告 `Landmark selection cancelled`；
  未输入合成点对或跳过人工步骤，不宣称完成了新的人工点选精度验收。
- 交互角度滑块使用副本中新生成的六区数据启动，页面本地资源加载成功。
  Playwright 验证三滑块、数字框、离面距离、路径位置、区域与模式切换、导出、选面及相机操作；
  1440×1000、390×844、320×720 与三种牙模范围共 9 组截图和 canvas 像素检查通过。
  此浏览器测试使用开发机的外部测试工具，用户运行查看器仍不需要 Node.js。
- 另单独以配置默认的 `perception_result=null` 启动查看器，接口确认可直接使用附带六区数据和工具网格，
  无需任何已生成的感知输出。验证结束后关闭了本轮测试服务，保留用户原有查看器服务。

## 回归与文档修正

- 干净环境 `unittest discover -s tests -p 'test_*.py' -q`：240 项通过。
- 干净环境 `unittest discover -s planning -t . -p 'test_*.py' -q`：108 项通过。
- 修复后的重计时与缓存重点测试：19 项通过。
- README 改用完整运行依赖安装入口，说明精确版本约束和可选包范围。
- 明确 `outputs/` 不随克隆提供，补齐单独命令的输入前置条件和路径变量。
- 默认分步提取、独立配准和六区导图采用批处理模式，交互查看另行说明。
- 修正路径查看器 R 键的含义：自动配准预览时切换已知位姿参考线，而非重置相机。
- 更新重计时成功状态；保留牙面恒力保护停止及人工选点、真实硬件的验证边界。
- 初次验收时主 README 的 31 个 PowerShell 代码块均通过语法解析，全部本地链接存在，`git diff --check` 通过。

上述结果只覆盖附带的下颌前牙区、当前系统和指定 Python/依赖组合。
其他区域动力学、全口串联、其他操作系统、真实相机和实机需各自验证。

## Git 发布前补充验证

同日创建 `cwj` 分支，提交完整模块、配置、依赖、文档图片及资产迁移。
为避免首次克隆时 Git 的换行转换破坏来源校验，新增 `.gitattributes`，
对 `assets/**` 设置 `-text`，保留资产原始字节。
`.gitignore` 继续排除虚拟环境、运行结果和缓存，并排除无关的 `design/session-*/` 临时演示目录。
README 增加实际仓库的 `git clone --branch cwj` 命令。

对代码提交 `0a9a230` 实际执行本地 Git 克隆，而非再次复制工作区：

```powershell
git clone --no-hardlinks --single-branch --branch cwj . outputs/github_cwj_verify
```

新克隆包含 501 个受版本控制的文件，使用 Windows Git 默认的 `core.autocrlf=true`。
执行时沿用上节已独立安装并验证过的 Python 3.11 虚拟环境，仅复用依赖；
模块、配置和资产全部来自新克隆，未复制旧 `outputs/` 或感知、规划产物。

- 清单中的 140 项资产逐项 SHA-256 校验通过；CSV、OBJ 和区域文件保持原始字节。
- `pipeline.run --params config/pipeline_outer_front.json --dry-run` 通过。
- `tests` 下 240 项测试通过，`planning` 下 108 项测试通过，共 348 项。
- 新输出目录的完整无窗口 pipeline 通过，墙钟总耗时约 62.86 s，轨迹仿真时长约 66.68 s。
- README 新增克隆命令后共 32 个 PowerShell 代码块，全部语法解析通过。
- 暂存内容检查通过，未包含本地环境、运行结果、缓存或临时演示目录；
  本次待推送历史中无大于等于 90 MiB 的文件对象，`git diff --cached --check` 通过。

此次补充验证确认 Git 检出不会改变所需资产，未扩大前述区域、系统、恒力及真实硬件的验收范围。
