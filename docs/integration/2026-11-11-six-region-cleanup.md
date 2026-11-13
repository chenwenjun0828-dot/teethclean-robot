# 六区域主流程清理与验证

日期：2026-11-11。本文描述删除旧提取线三分区分支后的当前代码。

## 当前入口

```powershell
.\.venv\Scripts\python.exe -m pipeline --params config/pipeline_outer_front.json --output outputs/pipeline/my_front
```

运行目录必须为空。六区域源数据、扫描网格和 CrossTooth 预测必须匹配；
`--dry-run` 也会检查这些输入。当前只附下颌六区域数据，上颌缺少数据时明确报错。
`perception` 准备六个独立区域；每次 `planning` 和位置控制只执行所选区域。
六区域自动排序执行、跨区转移和全口接触覆盖尚不属于这个入口的验收范围。

## 删除范围

- `perception/regions/source.py`：旧提取线三分区构建；共用牙面拟合迁移到 `surface.py`。
- `perception/gingival_order.py` 和 `tests/test_gingival_order.py`：旧牙弓排序及其测试。
- `config/gingival_regions.json`：旧三分区牙齿表。
- `config/pipeline_middle_surface15_90s.json` 及配套说明：旧分支 90 秒参数。
- `docs/PROJECT_STRUCTURE_LEGACY.md`：过时目录说明。
- 旧入口回退、三分区别名、逐牙主片段筛选、可达窗口裁剪、逐牙端点裁剪与跨牙补线。
- planning 侧的旧路径重建包装、重复几何构建和平面实现、硬编码中区九段诊断图分支。

入口只接受 `outer_left`、`outer_front`、`outer_right`、`inner_right`、
`inner_front`、`inner_left`，默认 `outer_front`。`--path-source`、`--extracted`
和旧的扩大裁剪范围参数已删除。旧三分区执行产物需要重新生成。

扫描原件、六区域数据、分割预测、提取线模块、配准、真实相机、硬件接口、力控模块和历史
`outputs/` 均保留。提取线仍是配准中间数据，不再作为轨迹生成的备用来源。
已有六区域产物保留只读加载兼容；历史设计和验证记录加上了归档说明。

## 覆盖率含义

当前每个区域使用一条有序 B-spline；牙齿边界只划分姿态支持，不改变源曲线。
默认只裁剪整区两端各 1 mm，没有逐牙丢段或人工补线。

| 当前 `outer_front` 指标 | 数值 |
| --- | --- |
| 区域源曲线 | 54.586750 mm |
| 保留源曲线 | 52.586750 mm |
| 整区端点损失 | 2.000000 mm |
| 保留比例 | 96.336107% |
| 人工补线 | 0 mm |

此前约 43.2% 来自旧提取线 `middle` 分支，不能用来评价当前六区域输入。
当前 96.34% 也仅表示所选区域源曲线的几何保留比例，不是全口覆盖率、组织接触率或治疗完成度。
此示例为偏移 1.8 mm 的非接触位置控制。

## 自动计时约束

清理时完整复跑发现，pipeline 未指定时长仍强制使用加速仿真档，导致当前六区域曲线在
TOPPRA/ECOS 阶段返回 `FailUncontrollable`。仅加密姿态采样未解决。
将自动计时与单独 planning 的默认约束统一后，同一自动配准数据通过完整验收：
关节速度 0.3 rad/s、加速度 0.8 rad/s²、角速度界限 15°/s。
显式指定总时长时仍沿用单独 planning 的加速仿真档，并须通过全部验收。
未更改几何误差、间隙或动力学通过标准。

## 本次验证

正式命令：

```powershell
.\.venv\Scripts\python.exe -m pipeline --params config/pipeline_outer_front.json --output outputs/integration_1111/six_region_cleanup_verified
.\.venv\Scripts\python.exe -m unittest discover -s tests -p 'test_*.py'
.\.venv\Scripts\python.exe -m unittest discover -s planning -t . -p 'test_*.py'
```

- `tests`：221 项通过；`planning`：102 项通过，共 323 项。
- 六个区域的几何导出、源曲线连续性、端点裁剪和保存加载均通过测试。
- 完整 pipeline 使用本次自动配准结果，ICP RMSE 约 0.1485 mm，未使用仿真真值变换替代。
- `outer_front`：430 个姿态节点，一段连续 trace，区域内部无退出或转移。
- 规划时间 66.676880 s，其中清扫 55.153467 s，进出场 11.523414 s。
- 独立 FK 路径误差约 0.000938 mm；静态轨迹最小间隙约 0.174677 mm。
- 动力学执行完成，最大位置误差约 0.087277 mm，最大旋转误差约 0.016478°，最小实际间隙约 0.156703 mm。
- 新结果与既有 `six_front_full` 六区域结果均通过 `load_verified` 加载检查。
- `compileall` 和 `git diff --check` 通过。

产物位于 `outputs/integration_1111/six_region_cleanup_verified/`，
包含 `pipeline_report.json`、六区域 perception 输出、tracking 规划与执行报告。
本次动力学验收仅涵盖下颌 `outer_front`，其余五区未在本次逐一执行动力学验收。
