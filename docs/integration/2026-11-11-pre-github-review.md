# GitHub 发布前流程检查

日期：2026-11-11。对象为当前工作区的六区域版本，环境为 Windows x64、Python 3.11.9、项目 `.venv`。
本轮未上传、提交或推送代码。既有资产迁移及其他工作区改动均保留。

## 结论与已知问题

1. 附带参数的下颌 `outer_front` 自动配准、感知、规划和非接触位置控制完整通过。
2. 发现并修复 `control.position.load_verified` 的模式兼容问题：默认 `simple` 轨迹原来被误报为旧 TCP 格式。
   加载器现在根据保存的配置选择期望的坐标约定，随后仍执行同一套来源、碰撞、轨迹及场景验证。
   新增测试覆盖 `simple`/`continuous`、错误坐标约定、旧 TCP 和共享验证失败的透传。
3. 恒力入口能运行并生成独立验收报告，但本例约 7.141 s 触发 2.5 N 预测力保护，未开始沿线清扫，
   `force_control_passed=false`、`scanned_surface_contact_passed=false`。不能宣称已经实现通过验收的牙面恒力清洁。
4. 已恢复 `planning.visualization.pose_angles` 交互滑块查看器，路径参考直接调用当前规划器姿态函数，
   支持三角度、离面距离、路径位置、区域和牙模范围切换。其输出始终为姿态预览，未经 IK 或碰撞验收。
   `planning.view` 用于回放候选或完整轨迹；`geometry_only` 可达性通过不能交给控制，必须重新运行 `full`。
5. 当前仅附下颌六区输入，每次规划一个区域。本轮不扩大结论到其余五区动力学、全口连接、真实相机或真实机器人。
6. 参数文件首次验证中的独立重计时失败已在后续 README 命令验收中修复：保留源关节和角速度限制后，
   本例 90 s 重计时、审计和动力学均通过。默认仍可沿用原时序，详见 [最新命令验收](2026-11-11-readme-command-audit.md)。

## 完整流程与独立执行

```powershell
$py = ".\.venv\Scripts\python.exe"
& $py -m pip check
& $py -m pipeline.run --params config/pipeline_outer_front.json --dry-run
& $py -m pipeline.run --params config/pipeline_outer_front.json --output outputs/pregithub_20261111/pipeline_front --headless
& $py -m control.position --plan outputs/pregithub_20261111/pipeline_front/tracking --headless
```

以上命令已实际执行，输出目录须为空，复跑请使用新目录。

| 项目 | 本轮结果 |
| --- | --- |
| 依赖一致性 | `pip check` 无冲突 |
| 自动配准 ICP RMSE | 约 0.1485 mm |
| 感知区域 | 六份独立 `PreparedRegion` |
| 选区 | `outer_front` |
| 模式与三角度 | `simple`，`[6.37, 13.26, -0.4]`，实际修正为零 |
| 场景 | 张口 20°，离面 1.8 mm，非接触 |
| 姿态节点 | 430 |
| 区域源线/保留线 | 54.586750 / 52.586750 mm |
| 几何保留率 | 96.336107%，无人工补线 |
| 轨迹时长 | 66.676880 s |
| 完整 pipeline 实际计算耗时 | 本机约 59.12 s，不是性能保证 |
| 独立位置执行 | `completed` |
| 最大位置/旋转误差 | 0.087277 mm / 0.016478° |
| 最小实际间隙 | 0.156703 mm |

## README 的分步命令验证

主 README 中两个 PowerShell 内嵌 Python 块已直接从文档提取并执行，而非另写等价代码。
独立输出位于 `outputs/tutorial/`，与完整 pipeline 运行目录分开。

- 彩色口扫牙龈线提取：1710 点、1693 边。
- 仿真采集：张口 20°，有效牙颌点 90589。
- 带元数据的自动配准与世界变换导出：完成，ICP RMSE 约 0.1485 mm。
- 独立双点云 CLI：完成，fitness 约 0.9445，RMSE 约 0.1485 mm。
- 已附 CrossTooth 快照校验：有效，234706 面，概率矩阵形状 `[234706, 19]`。
- 带本次世界变换的六区域独立导出：完成。
- `planning --check-mode geometry_only`：可达性通过，`execution_ready=false`，不执行碰撞和动力学。
- 从同一独立感知目录运行 `planning --check-mode full --headless`：通过，`execution_ready=true`，时长 66.676880 s。
- 独立完整轨迹加载通过；实际仅几何候选被控制加载器拒绝。仅几何候选的查看器截图也已成功导出。

交互人工选点与真机采集本轮未由人工实际操作。现有 GUI 状态/回调和配准回归测试通过；这不等于完成新的人工点选精度验收。

## 力控实测

```powershell
& $py -m control.force --source outputs/pregithub_20261111/pipeline_front/tracking `
  --target-force-n 1 --trace-speed-mm-s 1 --max-duration-s 90 `
  --out outputs/pregithub_20261111/force_front
& $py -m control.force.benchmark --target-force-n 1 --out outputs/pregithub_20261111/force_benchmark
```

前区实验按保护逻辑停止并保存日志，非零退出；源码 CLI 的验收失败返回值为 2。
该例补偿接触力仍为零，拒绝的是独立预测的下一步，不能将预测峰值当成已执行接触力。
源轨迹时间在进场阶段的增长也不代表清扫段覆盖，实际保留路径进度为 0。
具体信号和原网格接触判据见 `force_trajectory_report.json`、`contact_validation.json`。

独立球-平面 5 s 基准通过：末 1 s 平均 0.999993 N、RMSE 约 0.00000966 N。
这只证明该物理组件下的闭环，不替代牙面轨迹验收。
现有凸包接触代理和原始牙面不一致的问题仍见 [接触验证记录](2026-11-11-contact-force-validation.md)。

## 回归与图文

- `unittest discover -s tests -p 'test_*.py' -q`：修复后 228 项通过。
- `unittest discover -s planning -t . -p 'test_*.py' -q`：102 项通过。
- 位置加载和查看器重点复核：16 项通过。
- `planning.preview` 与 `planning.view --snapshot` 实际导出成功；另修正轨迹诊断图的刻度拥挤和 START/牙位标签重叠。
- README 配 11 张实际结果图，含恢复后的滑块页面；来源及重建命令见 [图片说明](../images/README.md)。

恢复交互式查看器后的重点回归 `planning.test_pose_angles`、`test_simple`、`test_region_pose`、`test_view` 共 44 项通过。
其中 6 项新测试覆盖真实 TCP 网格、两种模式的起中终点姿态一致性、坐标还原、选面、非法输入及离线资源服务。
Playwright 完成三个角度滑块/数字同步、离面距离、路径位置、区域与模式切换、导出、牙面选点和相机交互；
1440×1000、390×844、320×720 三种视口与三种牙模范围共 9 组截图及 canvas 像素检查通过。
检查记录在 `outputs/pose_angles_restore/browser_checks.json`，本轮未发现页面脚本错误或资源加载失败。

本轮没有重新安装全新环境；无缓存安装的已有独立验证见 [安装复现记录](2026-11-11-force-entry-and-installation.md)。

## 发布范围

当前工作区仍有大量既有文件迁移、删除和新增项。提交时应完整检查代码及必需资产，不能只提交入口文件或 README。
现有 `.gitignore` 排除运行产物、虚拟环境、缓存和本地 `.env`；文档图保存在 `docs/images/`。
滑块入口及其 `planning/visualization/pose_angles_web/` 离线资源、第三方许可证应一并提交。
当前资产中最大的单文件约 32.21 MiB，没有在本轮改写、裁剪或重新编码原始资产。

根目录尚无项目级许可证，机器人资产包含单独许可证。公开口扫、分割与源 CAD 前，数据所有者需要确认公开授权及去标识情况。
本轮只核对运行和文档可复现性，没有进行完整历史提交的秘密扫描或数据授权审计。
