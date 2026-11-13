# 连续区域接触与 1 N 恒力验证

> 历史验证记录：本文保留当时的代码、命令和输出证据。旧三分区提取线分支、兼容别名和旧参数已于 2026-11-11 删除，文中结果不代表当前六区域流程的覆盖率或端到端验收。当前入口与验证见 [六区域清理记录](2026-11-11-six-region-cleanup.md)。

## 结论

上一轮的 90 秒中区与 66.68 秒新前区验证均为 1.8 mm 名义离面轨迹，不是接触洁治。
本轮已接入并实测传感器闭环恒力控制，**尚未通过针对原口扫牙面的接触验收**。
不能将恒力曲线接近 1 N 解读为真实牙面上的恒力清洁已完成。

## 实现

- `control/force/continuous_path.py` 直接消费保存的连续区域参考和关节轨迹，
  不重建、裁剪或跳过感知曲线。整区一次贴合、连续沿线、一次卸力，随后执行原退出段。
  内部牙位切换不再触发逐牙重置或卸力；法向来自同一份感知参考并连续变化。
- `control.force --mode trajectory` 默认对连续区域采用上述调度。
  `--contact-schedule segments` 保留旧的逐源段贴合/卸力实验。
  区域模式不接受旧的逐段额外滚转，保持已规划姿态策略。
- 沿用已有法向导纳外环、位置伺服内环、重力/惯性前馈和安装处六轴传感器。
  调节量为**补偿后的接触合力大小**，不是原始 Fz，也不是仅法向分量。
  本轮目标 1 N、5 ms 滤波、0.4 秒缓升、法向行程 ±3 mm、最大法向速度 2 mm/s。
- 保留 2 N 软卸力、2.5 N 预测拒绝、3 N 硬停止、非工作端/对颌碰撞和关节限速保护。
  预测拒绝现在保留预测关节状态，明确标记为未执行，方便复查。
- `control/force/validation.py` 从 CSV 独立复算施加指令前后两组力、RMSE、失力、采样完整性，
  验证源文件指纹，再以 FCL 原始工具三角网格和原口扫三角网格复查实际关节位置。
  网格检查为 20 Hz，并补查阶段边界、力极值和首尾有力样本；它不是连续碰撞证明。
  几何一致性使用 0.1 mm 距离与距 TCP 3 mm 的工作端范围，不能代替组织接触模型。
- 报告区分 `force_tracking_passed`、`scanned_surface_contact_passed` 和
  `contact_trajectory_passed`。`force_control_passed` 必须同时通过力跟踪与原网格接触核验。
  保护触发、超时、失力或原网格不贴合不能因轨迹执行完成而被判为通过。

## 原提取线中区：连续区域完整尝试

来源为 `outputs/integration_1111/pipeline_registered/tracking`。
该来源的 23.3501 mm 保留源线，加上约 15.4727 mm 感知连接段，
组成约 38.8228 mm 连续区域参考。本轮不在连接处卸力；
不能把完整参考长度当成全都是原始牙龈源线，也不能把凸包上的接触计为牙面清洁长度。

| 指标 | 结果 |
| --- | --- |
| 执行含原始退出段 | 完成，150.893 s |
| 连续参考进度 | 100% |
| 力控沿线采样 | 145,665 条，1 kHz |
| 沿线接触合力范围 | 0.89439–1.11603 N |
| 沿线平均力 / RMSE | 1.00066 / 0.02861 N |
| 指令施加前后均在 0.8–1.2 N | 100% |
| 沿线失力时间 | 0 s |
| 全程力峰值，含贴合 | 1.94657 N |
| 非预期接触 / 牙模漂移 | 0 / 0 mm |
| 最大切向误差 / 姿态误差 | 0.18262 mm / 0.03127° |
| FCL 有力沿线复查 | 2,914 个状态 |
| 有力沿线状态的实际工具—口扫间隙 | **0.58453–1.59292 mm** |
| 当前物理模型中的力跟踪 | 通过 |
| 原口扫牙面接触验收 | **未通过** |

报告和全部原始信号位于 `outputs/contact_1111/middle_regional_force/`。
感知曲线与原关节轨迹没有修改；指纹核验通过。

## 新六分区前区：保护停止

来源为 `outputs/integration_1111/six_front_full`，调度保留全部 52.5867 mm 前区参考。
进场至 7.140 s 时，在独立预测状态中产生约 3.3136 N 的接触力，
超过 2.5 N 拒绝阈值，该预测步未提交，实际轨迹进度仍为 0。
该预测状态的真实工具—口扫网格间隙仍为 0.34848 mm。
结果位于 `outputs/contact_1111/front_regional_force/`，判为未通过，不能宣称已开始前区恒力清洁。

另以同一份感知前区、零离面参数和原基准角做了静态规划尝试，
现有碰撞规则与 ±5° 搜索未找到可执行路径，诊断位于 `outputs/contact_1111/front_zero_offset/`。
当前规划碰撞器仍要求完整工具不碰牙；该结果不是“所有接触姿态都无解”的证明，
也不能把任意工具—牙齿碰撞直接豁免为合法工作端接触。

## 控制器独立物理基准

`control/force/benchmark.py` 使用真实 MuJoCo 球—平面接触、单法向轴、同一传感器补偿与导纳算法，
保留预测保护，在 1 kHz 下运行 5 秒。末 1 秒平均力 0.999993 N、
RMSE 约 0.00000966 N、全程峰值约 1.0000 N，基准通过。
记录位于 `outputs/contact_1111/controller_1N/`。
它仅证明该理想模型下控制器组件能闭环收敛，不是六轴机械臂牙面轨迹的通过证明。

## 复现

命令在项目根目录执行，输出必须使用新的空目录。实际力不满足要求时返回非零退出码。

```powershell
# 连续区域恒力与原网格接触核验
.\.venv\Scripts\python.exe -m control.force --mode trajectory --contact-schedule regional `
  --source outputs/integration_1111/pipeline_registered/tracking `
  --target-force-n 1 --trace-speed-mm-s 0.5 --max-duration-s 180 `
  --out outputs/contact_1111/my_middle_force

# 控制器组件基准，不能替代上述轨迹验收
.\.venv\Scripts\python.exe -m control.force.benchmark `
  --target-force-n 1 --out outputs/contact_1111/my_controller_benchmark
```

每个区域力控目录包含 `trajectory_force.csv`、可能的 `trajectory_contacts.csv`、
`force_tracking.png`、`force_trajectory_report.json`、`contact_validation.json`、`scan_contact_audit.csv`。
传感器求解器一致性来自同一理想仿真模型，不代表真实硬件的噪声、延迟与标定精度。

回归结果：`planning` 136 项与 `tests` 226 项全部通过；
接触/力控重点回归 31 项通过，包括连续区域不重构、真实物理组件闭环、
预测状态不提交、完整采样保留，以及“有 1 N 力但离原网格 1 mm”不能通过的反例。

## 接触模型限制

本轮未改写原场景碰撞资产。MuJoCo 的网格碰撞使用凸包，
与带牙间隙和凹面的完整口扫表面不同，见
[MuJoCo mesh 文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh)。
接触余量还会提前激活接触力，见
[MuJoCo 接触计算](https://mujoco.readthedocs.io/en/stable/computation/#contact)。
当前间隙远大于每几何 0.05 mm 的接触余量，简单关闭余量不能修复整颌凸包问题。

后续真实牙面接触仿真需要表面保真的非凸碰撞表示、经核验的工作尖几何、
只允许目标工作端接触的规划/执行规则，以及每个区域独立的接触位置与力误差验收。
未在本轮通过这些条件前，不应转为实机接触清洁。1 N 仅为仿真测试设定，不是临床处方。
