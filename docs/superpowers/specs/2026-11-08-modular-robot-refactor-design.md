# 洁牙机器人模块化重构设计

日期：2026-11-08

## 1. 目标与范围

本次重构保留现有配准、牙龈线提取、牙齿分割、路径规划、位置控制和力控算法，只调整代码组织、模块接口、资产路径、文档、独立调试入口和测试。最终运行流程为：

```text
perception -> planning -> control
                  ^          ^
                  |          |
             perception   planning

pipeline 负责串联上述三个模块，不被它们反向依赖。
```

正式规划实现采用当前 `planning2` 的算法。旧的 `planning/` 实现不保留兼容入口；迁移完成后，`planning2/` 也不再作为顶层运行模块存在。

范围包括：

- 将感知、规划、控制和流程编排改造成四个清晰的运行模块。
- 为每个子模块定义稳定的输入、输出、单位和来源指纹。
- 将 `planning2` 的路径重构、姿态搜索和运动规划迁移到正式 `planning`。
- 将模型资产拆分为仿真资产、CAD 源文件和感知结果，并统一英文运行路径。
- 为每个子模块提供独立 CLI、README、architecture 文档和可视化结果。
- 为控制层加入仿真后端、真机协议和 mock 后端。
- 提供 `--no-output` 和 `--dry-run` 两种流程模式。
- 迁移并补齐现有测试，最终通过全量测试。

不在本次范围内：重新训练感知模型、改变规划数学算法、实现具体厂商硬件驱动、修改口腔几何算法或把仿真验证结果宣称为真机验证结果。

## 2. 目标目录

```text
perception/
  registration/
  gingival_extraction/
  tooth_segmentation/
  tools/
  contracts.py
  README.md
  ARCHITECTURE.md

planning/
  path_reconstruction/
  pose_search/
  motion_planning/
  infrastructure/
  tools/
  contracts.py
  README.md
  ARCHITECTURE.md

control/
  position/
  force/
  hardware/
  tools/
  contracts.py
  README.md
  ARCHITECTURE.md

pipeline/
  stages/
  tools/
  contracts.py
  run.py
  README.md
  ARCHITECTURE.md

assets/
  manifest.json
  simulation/
    dental_models/
    robot/
    scaler/
    dental_chair/
    force_sensor/
    scenes/
  source_cad/
    dental_models/
    robot/
    scaler/
    dental_chair/
    force_sensor/
  perception/
    registration/
    gingival_lines/
    tooth_segmentation/

config/
docs/
tests/
tmp/
```

`infrastructure/` 只包含规划三个算法模块共同使用的底层能力，例如机器人 FK/IK、碰撞检查、轨迹结构、序列化和来源指纹；它不承载新的规划策略。`tools/` 只放模块级调试、转换和可视化辅助程序，不放核心算法。

## 3. 模块职责与依赖

### 3.1 Perception

感知输入是带颜色的口扫点云/网格、MuJoCo RGB-D、相机配置、可选标志点和可选 CrossTooth 预测。感知模块负责：

- 点云与牙齿模型配准：人工或自动标志点、RGB-D、变换质量和误差可视化。
- 牙龈线提取：颜色分类、网格邻接平滑、等值线、外侧筛选、拓扑清理和牙弓排序。
- 牙齿分割：校验 CrossTooth 输入指纹和原始面编号，保留最大连通面片，生成逐牙实例和归属记录。

感知不求解机器人 IK，也不调用控制器。

### 3.2 Planning

规划输入是感知输出、牙齿模型、机器人场景、规划参数和可选底座/世界变换。规划拆为：

- `path_reconstruction`：逐牙归属、三点牙面、牙龈线曲线、可达区间裁剪、连续扫掠参考。
- `pose_search`：工具坐标约定、平面倾角、滚转策略、候选姿态、IK 和路径级姿态选择。
- `motion_planning`：进场、跨段过渡、关节轨迹、时间分配、碰撞检查、完整 FK 和轴向速率审计。

规划只生成通过检查的可执行轨迹；不负责位置伺服或力控。现有 `planning2` 的模块代码和 `planning` 中仍被复用的机器人、碰撞、轨迹基础能力迁移后只保留一份实现。

### 3.3 Control

控制输入是规划输出的已验证轨迹、当前机器人状态、碰撞检查器和控制参数。控制包括：

- 位置控制：轨迹采样、重力/惯性前馈、位置伺服、跟踪误差和运行时碰撞检查。
- 恒力控制：六轴信号补偿、法向柔顺、接触路径阶段切换、力限保护和接触记录。
- 硬件接口：连接、状态读取、六轴力读取、位置/速度命令、急停和断开。

控制算法只依赖 `HardwareBackend` 协议；MuJoCo、mock 和未来的 UR 驱动分别实现该协议。没有真实驱动依赖时，真机后端只进行参数检查，不模拟硬件执行。

### 3.4 Pipeline

`pipeline` 只负责阶段编排、配置覆盖、错误传播、输出目录策略和报告汇总。流程阶段为：

```text
asset/config validation
  -> perception
  -> planning
  -> control
  -> pipeline report
```

它可以选择仿真或硬件后端，但不复制三个模块的算法。每一阶段接收上一步的结构化结果并写入阶段状态。

## 4. 数据契约

每个模块的 `contracts.py` 定义 dataclass 和 schema 版本。文件边界使用 JSON 加 NumPy 二进制文件；所有输出记录：

- `schema_version`
- `module`
- `created_at`
- `units`
- `source_fingerprints`
- `config_fingerprint`
- `status`

单位约定固定为：口扫点云和牙齿几何使用 mm；世界位置使用 m；关节角使用 rad；力使用 N；力矩使用 N*m；时间使用 s。

### 4.1 PerceptionResult

包含配准变换、配准误差、牙龈线有序点和拓扑边、逐牙实例、原始扫描指纹、拓扑来源文件和各阶段报告路径。结果必须能被 planning 独立加载，不依赖 MuJoCo Python 对象。

### 4.2 PlanningResult

包含几何参考、路径段、姿态候选/选定姿态、关节轨迹、机器人关节名、TCP 坐标系、场景和牙模指纹、碰撞/可达性审计结果和 `execution_ready` 状态。control 只能加载 `execution_ready=true` 且来源指纹匹配的轨迹。

### 4.3 ControlReport

包含后端类型、执行状态、时间、关节状态、TCP 目标/实际位姿、位置/姿态误差、补偿后接触力、原始六轴信号、保护状态和 `fault_reason`。故障码至少包括：`invalid_trajectory`、`joint_limit`、`collision`、`force_limit`、`tracking_error`、`hardware_disconnected`。

### 4.4 PipelineReport

包含颌别、运行模式、阶段顺序、每阶段输入/输出路径、状态、耗时、错误、配置指纹和最终执行状态。输出目录默认结构为：

```text
<run_output>/
  request.json
  perception/
  planning/
  control/
  pipeline_report.json
```

## 5. 资产策略

`assets/simulation/` 保存当前 MuJoCo 场景、牙模、机器人、洁牙器、牙椅和力传感器运行所需的 XML、OBJ、STL、PLY 等文件。`assets/source_cad/` 保存仍有价值的相关 CAD 源文件，按类别和英文逻辑名称归档。`assets/perception/` 保存配准、牙龈线和牙齿分割结果，不与仿真网格混放。

只删除未被当前流程引用且不属于上述相关 CAD 源文件的重复/历史模型。所有 XML、配置、Python 默认值和测试路径改为英文运行路径。`assets/manifest.json` 为每个文件记录逻辑名称、类别、格式、用途、来源和 SHA-256；资产测试检查所有 XML 引用都能在 manifest 中解析。

## 6. 调试、可视化与运行模式

每个子模块提供独立入口，例如：

```text
python -m perception.registration ...
python -m perception.gingival_extraction ...
python -m perception.tooth_segmentation ...
python -m planning.path_reconstruction ...
python -m planning.pose_search ...
python -m planning.motion_planning ...
python -m control.position ...
python -m control.force ...
python -m control.hardware ...
python -m pipeline.run ...
```

每个入口都支持 `--help`、输入校验和可视化输出。可视化至少覆盖：配准叠加、牙龈线拓扑、牙齿实例、路径曲线、姿态坐标轴、机器人轨迹、位置误差和力控曲线。

运行模式定义如下：

- 默认模式：执行算法/控制并保存 JSON、CSV、NPZ、PNG 等结果。
- `--no-output`：完整执行，但不创建输出目录、不写任何运行文件、不修改资产；API 返回内存中的结果对象，CLI 只打印阶段和最终状态。
- `--dry-run`：只检查资产、配置、依赖、输入指纹和后端参数，不执行感知算法、规划算法或控制器。

`--no-output` 与 `--dry-run` 互斥。硬件后端必须显式指定，默认使用 MuJoCo 仿真后端；真机执行前强制核对轨迹指纹、TCP 坐标系、关节顺序、力传感器标定和急停状态。

## 7. 测试与验收

- 将 `planning2/test_*.py` 迁移到正式 `planning` 命名空间，并更新规划器标识和导入路径。
- 将原 `planning` 中仍有价值的机器人、碰撞、轨迹测试迁移到 `planning/infrastructure` 对应测试。
- 将感知测试按注册、牙龈线提取、牙齿分割和边界契约拆分。
- 为位置控制、力控、硬件协议、mock 后端和故障码增加测试。
- 为 pipeline 阶段顺序、错误传播、`--no-output` 和 `--dry-run` 增加测试。
- 增加资产 manifest、英文路径、XML 引用和 SHA-256 校验测试。
- 运行现有与迁移后的全量 `unittest`，同时提供快速冒烟命令。

验收必须满足：

1. 四个顶层模块可以独立导入和运行。
2. 每个子模块有 README、ARCHITECTURE、CLI 和可视化结果。
3. 完整仿真流程能从 perception 运行到 control。
4. 仿真后端和真机协议共享控制器接口。
5. `--no-output` 不创建任何运行文件。
6. `--dry-run` 不执行算法或控制器。
7. 全量测试通过，旧 `planning`/`planning2` 顶层入口不再被引用。
8. 所有运行时模型引用来自 `assets/`，不依赖中文路径。

## 8. 实施顺序

1. 建立数据契约、资产 manifest 和路径解析工具。
2. 迁移 `planning2` 为正式 `planning`，合并必要的规划基础能力。
3. 整理 perception 子模块并接入 `PerceptionResult`。
4. 整理 control 子模块，加入后端协议、MuJoCo 适配和 mock 后端。
5. 改造 pipeline，接入阶段报告、`--no-output` 和 `--dry-run`。
6. 整理资产并更新所有 XML、配置和测试引用。
7. 补齐 README、ARCHITECTURE、调试入口和可视化。
8. 迁移测试、运行全量验证并清理旧顶层入口。

每一步都先运行对应模块测试，再运行跨模块测试；只有全量测试通过后才删除旧模块目录和未使用模型。
