# 右侧实时传感器面板验证

日期：2026-11-07。范围：代码职责整理、MuJoCo 内嵌右侧参数、滚动六轴信号与接触力曲线；本次没有调节力控参数。

## 自动检查

86 项相关测试通过，覆盖已有规划、接触实验、柔顺控制和完整路径保护，以及新增面板测试。新增测试确认：10 秒历史有界；结束样本不重复；显示压缩保留单采样尖峰和失力；原始力不裁剪；参数来自实际配置；即使未到下一显示帧也采集力峰值；显示模型和状态的扰动不会修改执行模型。

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_force_panel tests.test_contact_path tests.test_compliance planning.test_contact_experiment planning.test_motion_modes planning.test_planning planning.test_retime tests.test_planning_v2 tests.test_planning_reachability -q
```

最终显示层级修正后，增加“关闭窗口后最终快照仍正确记录停止状态”的检查，面板 5 项测试通过（累计 87 项）。`git diff --check` 通过。

## 实际碰撞仿真对比

来源：`outputs/planning/mandibular_fast_58s`。相同配置：`--mode trajectory --trace-speed-mm-s 0.5 --max-duration-s 8`，分别启用和禁用 `--viewer`。

| 项目 | 结果 |
| --- | --- |
| 有界面输出 | `outputs/planning/force_panel_graphs_checked` |
| 无界面输出 | `outputs/planning/force_panel_headless_final` |
| 数据行数 | 8001 |
| 接触建立阶段开始 | 4.009 s |
| 沿线阶段开始 | 4.927 s |
| 未滤波最大补偿接触合力 | 1.4347626760178183 N |
| 终止原因 | 用尽 8 秒试运行预算，剩余路径未跳过 |
| CSV 对比 | `trajectory_force.csv` 逐字节一致 |

两份传感器 CSV 的 SHA-256 均为 `DF7929EF0C6AD878F2358828D9B0B44B8EA0B37850EC294923AB2C6ECD6591CD`。这项短程测试只验证显示不干扰仿真，不代表完成整段洁治或通过严格力控验收。已有整段结果和限制仍见 [FULL_TRAJECTORY_FORCE_VALIDATION.md](FULL_TRAJECTORY_FORCE_VALIDATION.md)。

## 实际窗口检查

在原生 MuJoCo 窗口确认右侧参数、三组曲线和阈值线。修复近处三维物体遮挡图片的问题：同一侧栏先绘制原生二维背景，再绘制中文面板。缩小面板时按窗口尺寸缩放；三维观察与实际控制状态隔离。试运行结束额外停留 35 秒仅用于窗口截图，正式命令不包含该停留。

面板更新上限为 20 Hz，横轴为仿真时间；物理步长及日志保持 1 ms。显示压缩使用每像素列的最小/最大值包络，保留采样尖峰；没有额外平滑、力值截断或日志删减。
