# 真机 session 默认使用光学帧解释

- 状态：已批准并实施，验证通过。
- 问题与目标：开发者认可 `planning.view --plan outputs/planning/session_optical_preview` 的显示效果，并要求把对应模式设为默认。已核对该目录 request/report 的 `camera_frame` 均为 `color_optical`，并非 `declared`。目标是新建 session 仿真/规划时不传模式即可得到当前认可的坐标解释；回放仍服从保存的解释。
- 范围：`perception/session.py`、`planning/session.py`、`planning/__main__.py` 的 session 默认参数，`tests/test_perception_session.py` 的默认/兼容回归，以及 `planning/README.md`、`planning/ARCHITECTURE.md` 的说明。不修改原始数据、坐标计算算法、相机解释版本、机器人姿态、XML、规划策略、执行保护或既有产物。
- 设计：在 perception session 定义唯一默认值 `color_optical`，让感知加载 API、场景构建 API 和 CLI 引用它。未指定解释的新 session 调用采用当前光学帧链；显式 `--session-camera-frame declared` 仍恢复原发布解释。JSON 的显式值和 CLI 覆盖优先级不变。
- 坐标与单位：继续使用 `T_A_B` 列向量左乘、平移和运行时点为米；变换公式和两种解释的原始来源验证保持不变。继续标记光学帧解释尚未用现场 TF 独立验证，不将默认选择记作标定已验证。
- 回放兼容：有解释记录时按记录中的值重建；旧记录缺少解释版本时仍显式使用 `declared`。不因新默认值改变已有结果。当前 `session_optical_preview` 的查看命令和结果保持一致。
- 选择依据：开发者认可的具体预览使用光学帧，故将该解释作为新 session 的默认选择；保留显式选择和产物记录，避免把当前预览误认为原始发布结果。
- 风险：新 session 命令/API 省略参数时，结果将与原先默认 `declared` 不同。已有的显式 `declared` 调用及保存结果不变；文档和回归测试必须清楚区分“新建默认”和“回放旧记录”。不涉及非 session 入口。
- 实施步骤：记录明确批准；统一默认值及调用入口；把原声明链兼容测试改为显式 `declared`；增加默认等于显式光学帧、CLI 无模式值、显式回退和旧产物回放测试；同步说明。
- 验证与验收：运行 `python -m unittest tests.test_perception_session tests.test_planning_parameters -q` 与 `git diff --check`；在隔离临时输出中以不传模式的 CLI 做几何导出并回放，验证保存模式为 `color_optical`，双颌矩阵和六关节角与当前认可预览一致；验证原声明解释及无版本旧产物仍按 `declared` 重建。仅修改默认选择，不以这些检查宣称完整规划或实机验证。
- 回退方式：将共享默认值恢复为 `declared`，或调用时显式选择它；保留原始数据和已有输出。
- 审核记录：2026-11-15，开发者在收到本方案链接及“将当前预览使用的 `color_optical` 设为新 session 默认”的确认请求后明确回复“确认”，批准本方案范围。

## 实施与验收记录

- `perception/session.py` 新增唯一默认值 `DEFAULT_SESSION_CAMERA_FRAME = 'color_optical'`，感知加载、规划场景构建与 CLI 共同引用。显式选择与回放兼容逻辑保持不变，文档已同步。
- 原声明链相关测试显式选择 `declared`；新增测试验证默认 API 等于显式光学帧，并在隔离临时目录中以不传模式/显式 `declared` 两种 CLI 分别导出几何、重新加载回放，核对保存解释、双颌位姿和拍摄关节角。
- 实际使用 `.venv/Scripts/python.exe -m unittest tests.test_perception_session tests.test_planning_parameters -q`，最终 28 项通过（9.551 s）；`git diff --check` 通过。
- 独立只读比对当前默认场景与 `outputs/planning/session_optical_preview/planning_report.json`：两颌 world 矩阵的最大元素差均为 0，六个拍摄关节角完全相同；现有预览通过 `load_scene` 回放重建，模式保持 `color_optical`。未改写已有预览。
- 无方案范围变化。此次仅验证默认选择及回放兼容，没有重新验证完整规划或实机精度；光学帧假设状态不变。
