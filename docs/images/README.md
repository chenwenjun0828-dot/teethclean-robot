# README 图片来源

这些图片是实际仿真、已附预测快照或其数据投影，不是 AI 生成的示意图。日期为 2026-11-11。
源数据路径和 SHA-256 记录在 [sources.json](sources.json)；原始运行产物位于被 Git 忽略的 `outputs/`，
PNG 随文档提交，不要求 GitHub 读者先运行程序。

| 图片 | 来源与解释 |
| --- | --- |
| `simulation_capture.png` | 本轮完整流程的 RGB、深度预览、有效相机毫米点云投影 |
| `gingival_extraction.png` | 同轮彩色口扫提取线的原始三视图 |
| `registration.png` | 同轮自动配准的原始叠加图，RMSE 约 0.1485 mm |
| `tooth_segmentation.png` | 已附 CrossTooth `segmentation.ply` 顶点颜色的 X-Z 投影，不是本轮神经网络推理 |
| `regional_paths.png` | `pipeline.path_viewer --placement known` 六区源线预览，不证明配准或机器人可达性 |
| `tool_pose.png` | `planning.view` 在已验证轨迹第 12 s 的回放截图，不是执行/接触截图 |
| `pose_angles.png` | `planning.visualization.pose_angles` 交互滑块实际页面截图，读取独立感知结果，未做 IK 或碰撞验收 |
| `trajectory.png` | `planning.preview` 连续参考/离面 TCP 轨迹诊断图 |
| `position_control.png` | 本轮独立 `control.position` 执行 CSV 的实际跟踪误差 |
| `force_control.png` | 前区力控保护停止时的原始曲线，未通过牙面接触验收 |
| `force_benchmark.png` | 通过的独立 1 N 球-平面组件基准，不代表牙面轨迹通过 |

## 重建

在项目根目录的 PowerShell 中执行，输出目录需要是新的空目录。
力控实验预期可能保护停止并返回非零退出码，报告和图片仍保存；应先检查报告再导出。

```powershell
$py = ".\.venv\Scripts\python.exe"
$run = "outputs/readme_rebuild/front"
& $py -m pipeline.run --params config/pipeline_outer_front.json --output $run --headless
& $py -m control.position --plan "$run/tracking" --headless
& $py -m planning.preview --plan "$run/tracking"
& $py -m planning.view --plan "$run/tracking" --time 12 --camera 10 --snapshot outputs/readme_rebuild/pose.png
& $py -m pipeline.path_viewer --placement known --headless --output outputs/readme_rebuild/regions
& $py -m control.force --source "$run/tracking" --target-force-n 1 --trace-speed-mm-s 1 --max-duration-s 90 --out outputs/readme_rebuild/force
& $py -m control.force.benchmark --target-force-n 1 --out outputs/readme_rebuild/benchmark
& $py docs/tools/export_readme_images.py --run $run --force outputs/readme_rebuild/force `
  --benchmark outputs/readme_rebuild/benchmark --regional-preview outputs/readme_rebuild/regions `
  --pose outputs/readme_rebuild/pose.png
```

导出器会更新本目录 PNG 和来源清单。改变源数据或参数后，应同步修改主 README 的数值、图注和验收结论。

滑块查看器截图可在导出命令中加 `--pose-slider <截图路径>`。
本轮截图来自 `outputs/pose_angles_restore/1440_mandibular.png`；使用同一张口角、感知结果和三角度启动
查看器后，可用 `planning/tools/check_pose_angles.cjs`（需开发环境已安装 Playwright）重做浏览器交互检查及截图。
