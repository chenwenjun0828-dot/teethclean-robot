# Planning 参数文件验证

日期：2026-11-11。环境为项目 `.venv`，Windows x64 / Python 3.11。

## 接入范围

主规划、角度滑块、轨迹回放、preview、sweep_preview、segment_map、retime、execute
均接受 `--params`，包括其 visualization / motion_planning 实现入口。
示例位于 `config/planning_*.json`，两个同参数诊断图命令共用 `planning_preview.json`。

复用 pipeline 的 argparse schema 读取器，增加必填参数由 JSON 提供、整数与自定义类型校验。
参数优先级为默认值 < JSON < CLI；重复键、未知键、非法类型、非有限数值、跨来源互斥参数被拒绝。
显式设置的区域和牙颌必须与感知结果匹配。常规布尔开关支持负向 CLI 覆盖。

## 回归结果

- `python -m unittest discover -s tests -p 'test_*.py' -q`：240 项通过。
- `python -m unittest discover -s planning -t . -p 'test_*.py' -q`：108 项通过。
- 新增 12 项参数文件测试，覆盖全部命令、兼容入口、必填路径、数值类型、布尔覆盖和感知区域一致性。
- pipeline 原有样例的 headless 偏好保持为 false；其转发测试改为显式传 `--headless`，与使用文档一致。
- pipeline 参数文件的真实 `--dry-run` 通过；`git diff --check` 通过。

完整测试日志位于 `outputs/planning_parameter_review/tests.log` 和 `planning_tests.log`。

## 实际命令

```powershell
$py = ".\.venv\Scripts\python.exe"
& $py -m planning --params config/planning_outer_front.json --dry-run
& $py -m planning --params config/planning_outer_front.json
& $py -m planning.view --params config/planning_view.json --time 12 --snapshot outputs/planning_parameter_review/pose.png
& $py -m planning.preview --params config/planning_preview.json
& $py -m planning.segment_map --params config/planning_segment_map.json
& $py -m planning.execute --params config/planning_execute.json
```

以上命令均通过。主规划输入为先前独立感知的 `PreparedRegion`，输出 `outputs/planning/outer_front`：
`execution_ready=true`，时长 66.676880 s，430 个姿态节点，名义三角度无需修正。
执行入口成功加载已验收计划，并按既有逻辑复用该计划已完成的无界面动力学运行结果，
没有把复用报告表述为第二次独立动力学实验。
同一目录不能再次用于生成规划；复跑需用 `--out` 指定新目录。

角度查看器使用 `config/planning_pose_angles.json --no-browser --port 8766`
及 CLI 指定的同一感知结果实际启动；HTTP 接口确认三角度、离面距离和张口角正确加载。
独立角度服务器仅用于本轮验证，结束后关闭；先前已打开的查看器保持可用。

## 独立重计时限制（修复前记录）

后续 README 命令验收已修复下述问题，90 s 重计时现已通过。原因和新环境实测结果见
[README 命令验收](2026-11-11-readme-command-audit.md)。以下保留首次失败记录。

```powershell
& $py -m planning.retime --params config/planning_retime.json
& $py -m planning.retime --params config/planning_retime.json `
  --trace-speed-mm-s 0.8 --out outputs/planning_parameter_review/retimed_90s_08
```

两次均正确读取 JSON 及 CLI 覆盖，进入原有重计时算法后失败：
`TOPPRA could not time the fixed joint path (ECOS; 1717 grid points; return code: FailUncontrollable; parameter scales: 1000, 100)`。
输出报告均为 `status=fixed_path_validation_failed`、`execution_ready=false`，不能交给控制模块。
源轨迹与其既有验收未被覆盖；`planning_execute.json` 的 `original_timing=true` 可正确执行源轨迹。
独立重计时配置展示参数读取方式，不代表当前样例已获得通过验收的 90 s 轨迹。
