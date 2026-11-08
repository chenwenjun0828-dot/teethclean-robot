# 洁牙机器人模块化重构实施计划

> 依据：`docs/superpowers/specs/2026-11-08-modular-robot-refactor-design.md`

## 全局约束

- 保留现有算法行为；每次迁移后先运行对应测试，再扩大验证范围。
- 不覆盖或回退用户已有未提交改动。
- 所有新运行路径通过仓库根目录解析，避免依赖当前工作目录。
- `planning2` 迁移完成前不删除其源文件；迁移后的导入和测试稳定后再清理旧顶层入口。
- 默认运行继续写结果；`--no-output` 完整运行但不写文件；`--dry-run` 只校验。

## 阶段 1：路径解析、契约和资产清单

**目标：** 建立模块间可复用的版本化边界，先不改变算法流程。

**文件：**

- 新建 `perception/contracts.py`、`planning/contracts.py`、`control/contracts.py`、`pipeline/contracts.py`。
- 新建 `assets/manifest.json` 和 `assets/tools/build_manifest.py`。
- 新建一个共享的根路径解析工具，或将路径逻辑收敛到现有 `planning.io` 后由四个模块调用。
- 测试新增 `tests/test_contracts.py`、`tests/test_assets.py`。

**步骤：**

1. 定义 schema 版本、单位、状态和来源指纹字段。
2. 为 PerceptionResult、PlanningResult、ControlReport、PipelineReport 提供序列化/反序列化骨架。
3. 扫描当前 XML 和 Python 引用，生成初始 manifest；暂不移动模型。
4. 测试 JSON/NPZ 往返、路径解析、manifest 引用完整性和 no-output 辅助函数。

## 阶段 2：将 planning2 迁移为正式 planning

**目标：** 形成唯一正式规划命名空间，保持 `planning2` 逻辑和现有输出格式。

**文件：**

- 迁移 `planning2/path_reconstruction/` 到 `planning/path_reconstruction/`。
- 迁移 `planning2/pose_search/` 到 `planning/pose_search/`。
- 迁移 `planning2/motion_planning/` 到 `planning/motion_planning/`。
- 将 `planning2` 的配置、pipeline、可视化、入口迁移到对应 `planning` 文件。
- 将 `planning/geometry.py`、`robot.py`、`collision.py`、`trajectory.py`、`io.py` 等复用基础能力移动到 `planning/infrastructure/`，逐步更新导入。
- 更新 `pipeline/tracking.py`、`control` 和所有规划测试的导入。

**步骤：**

1. 先复制/迁移文件并让 `planning` 新入口能导入，不删除任何旧文件。
2. 统一规划器标识、请求文件和报告 schema，保留现有轨迹文件兼容性。
3. 将测试从 `planning2/test_*.py` 迁移到 `tests/` 或 `planning` 的新测试目录，并更新导入。
4. 运行 planning 单元测试和架构测试。
5. 用现有真实输入做 geometry-only 规划，比较几何、报告字段和轨迹结构。
6. 确认无代码引用 `planning2` 后，删除 `planning2/` 顶层实现和旧 planning 算法入口。

## 阶段 3：重构 perception 子模块

**目标：** 将现有注册、牙龈线提取和牙齿分割接入独立契约，不改算法。

**文件：**

- 将 `perception/tooth_instances.py` 归入 `perception/tooth_segmentation/`。
- 整理 `registration/`、`gingival_extraction/` 的 CLI 和模块 `__main__.py`。
- 增加 `perception/ARCHITECTURE.md`、各子模块 README、`perception/tools/`。
- 更新 `perception/workflow.py` 返回 `PerceptionResult` 并支持 `--no-output`。

**步骤：**

1. 先保留旧导入的内部实现，新增薄的子模块入口。
2. 让三个子模块分别输出结构化报告和预览图；no-output 时改用内存对象/临时缓冲。
3. 为输入单位、来源扫描指纹和 CrossTooth 面编号增加契约校验。
4. 迁移并运行现有感知测试。
5. 更新 `perception/README.md` 和 `ARCHITECTURE.md`，给出单模块命令。

## 阶段 4：重构 control 和真机接口

**目标：** 复用现有位置/力控算法，增加后端协议与可测试的仿真实现。

**文件：**

- 将现有 `control/position.py` 整理为 `control/position/`。
- 将 `force.py`、`compliance.py`、`contact_path.py`、`force_trajectory.py` 等整理为 `control/force/`。
- 新建 `control/hardware/protocol.py`、`simulation.py`、`mock.py`、`ur_robot.py`。
- 新建 `control/ARCHITECTURE.md`、子模块 README 和工具入口。

**步骤：**

1. 定义 HardwareBackend 协议和统一状态/故障码。
2. 将 MuJoCo 状态读写包在仿真后端，控制器接收协议而不是直接依赖 MuJoCo 数据结构。
3. 让位置控制和力控保留现有报告字段，并补充 ControlReport。
4. 增加 mock 后端、断连/碰撞/超力/跟踪误差测试。
5. 增加真机后端骨架，未安装驱动时只做配置和安全校验。
6. 运行 control 现有测试和新增协议测试。

## 阶段 5：资产迁移与英文路径

**目标：** 形成 `assets/simulation`、`assets/source_cad`、`assets/perception` 三层资产布局。

**步骤：**

1. 从场景 XML、规划配置、感知配置和测试提取实际引用文件。
2. 将实际仿真文件复制/移动到 `assets/simulation`，按类别使用英文名称。
3. 将相关 CAD 源文件归档到 `assets/source_cad`，同样使用英文逻辑名称；不再被引用的历史重复模型不迁移。
4. 将感知结果移动到 `assets/perception`，保留来源指纹。
5. 更新所有 XML、配置、Python 默认值、README 和测试引用。
6. 重建 manifest，校验所有引用文件存在、路径无中文、格式和 SHA-256 与清单一致。

## 阶段 6：pipeline、可视化和运行模式

**目标：** 让完整流程使用新契约，并实现两个运行模式。

**文件：**

- 新建 `pipeline/contracts.py`、`pipeline/stages/`、`pipeline/tools/`。
- 修改 `pipeline/run.py`、`pipeline/tracking.py`、`pipeline/simulation.py`。
- 新建 `pipeline/README.md`、`pipeline/ARCHITECTURE.md`。

**步骤：**

1. 将 pipeline 阶段拆为资产/配置校验、perception、planning、control。
2. 统一阶段报告和默认输出目录结构。
3. 为 pipeline 和各子模块加入 `--no-output`；禁止创建输出目录和文件。
4. 加入 `--dry-run`，只执行校验并明确不运行算法/控制。
5. 保留现有 MuJoCo 预览和控制可视化，在新目录写出阶段预览。
6. 添加阶段顺序、失败传播、模式互斥和无输出文件测试。

## 阶段 7：文档、测试和清理

**目标：** 完成协作开发所需文档并达到验收标准。

**步骤：**

1. 为每个子模块补齐 README、ARCHITECTURE、输入输出表、命令示例和可视化说明。
2. 更新根 README、`docs/PROJECT_STRUCTURE.md` 和 requirements 说明。
3. 运行模块测试、跨模块测试、全量 unittest 和 `git diff --check`。
4. 使用真实上下颌数据运行一次 geometry-only 和一次完整仿真 smoke test。
5. 确认 no-output 不生成文件，dry-run 不进入算法函数。
6. 清理旧 `planning`/`planning2` 顶层入口、未使用模型和过期导入；保留历史验证资料于 `tmp/`。
7. 最后再次运行全量测试并记录结果。

## 验证命令

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -q
.\.venv\Scripts\python.exe -m unittest discover -s planning -q
.\.venv\Scripts\python.exe -m perception.registration --help
.\.venv\Scripts\python.exe -m perception.gingival_extraction --help
.\.venv\Scripts\python.exe -m planning --help
.\.venv\Scripts\python.exe -m control.position --help
.\.venv\Scripts\python.exe -m control.force --help
.\.venv\Scripts\python.exe -m pipeline.run --help
```
