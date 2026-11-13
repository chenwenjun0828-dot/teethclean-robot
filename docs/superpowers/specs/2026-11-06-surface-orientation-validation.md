# 牙面姿态规划验证记录

实现与配置说明见 `planning/SURFACE_ORIENTATION.md`。本次新增代码为 `surface_frames`、`pose_candidates`、`mujoco_feasibility`、`trajectory_search`、`replay_eval`、`dental_orientation` 和 `view_surface_plan`，配置为 `config/dental_orientation.json`。原始扫描、历史牙齿坐标系与既有优化结果未覆盖，未修改 `pipeline.run` 的默认规划策略。

## 自动测试

以下命令通过，共 57 项，含 17 项新增算法/运动测试和 40 项现有相关测试：

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_dental_orientation `
  tests.test_reference_frames tests.test_pose_ik tests.test_pose_optimization `
  tests.test_tooth_frames tests.test_tcp_tooth_alignment tests.test_planning_control -q
```

检查包括平面/圆柱投影、相邻双面不跳面、交错绕序修正、路径反向、段间隔离、刚体等变性、旋转时接触点保持、有限轴向工作区、qpos/DOF 独立地址、边中间碰撞、开放面背侧负距离带、距离下界、贪心死路/DP 替代分支、首点起步校验、五次时序限速和实际位置执行器回放。已有 matplotlib/pyparsing 弃用提示不影响测试通过。`git diff --check` 通过。

## 真实牙面几何

最新局部系检查保存于 `outputs/dental_orientation/surface_final_check`。原排序编号 1010 附近短段有 4 个重采样点，全部有效；最大相邻局部系转角 6.07798°、最大法向转角 5.85211°、正交性误差 4.44e−16、最小行列式 0.9999999999999997。输入平滑最大位移 0.15 mm、初次投影最大距离 0.00442 mm。原始点另存只读对照。

`view_surface_plan --snapshot` 已运行并人工查看图片，确认牙面高亮、工具模型和局部坐标箭头可显示。三维查看保留真实遮挡关系；被牙龈遮住的轴不会被绘成强制置顶。键盘 N/P 切采样点、C 切候选、O 切已选姿态。目标面片仅作显示偏移 0.015 mm 以避免与原网格共面闪烁，不改变计算数据。

## 真实前牙短段规划

`outputs/dental_orientation/anterior_hover_verified` 从场景 home 开始，四层均有可行节点（16/28/36/44），但所有进入候选的预接近扫掠均触发工具原扫描净距失败。首断点为入口层 0，结果 `entry_failed`，未输出完整关节轨迹或启动动态回放。

`outputs/dental_orientation/anterior_local_trace_verified` 显式把独立仿真起点放在一个通过检查的首点，以单独检查局部连接。四层节点数 18/26/36/40，检查 102 条连接、接受 2 条，第 3 层（零基）不连通。删除原因：关节分支跳变 91 条、工具净距 8 条、插值路径误差 1 条；剩余 2 条通过。没有把部分路径作为成功结果。

`anterior_micro_motion_validation` 的更短输入得到 3 个有效采样点及 22/26/36 个节点，但原始指定起点与这次重采样后的首点姿态不同，预接近扫掠仍未通过。它不构成牙面运动成功证据。

早期 `v1/v2/v3` 用于诊断工作区和名义方位，保留原报告；`v4/v5` 被改进后的原三角面查询加速版本替代，报告明确标记中断，不计入完成结果。最终测试包含针对开放扫描负侧距离带的回归检查，避免包围盒加速将背侧近邻错误判为正净距。

## 独立可达机器人运动验证

`outputs/dental_orientation/reachable_robot_fixture/validation.json` 记录独立 UR7e 结构代理夹具的完整搜索、加密运动学及执行器回放。最大 TCP 跟踪误差 0.05090 mm、最大实际关节速度 0.03894 rad/s、最大实际关节加速度 0.51514 rad/s²，均通过当前运动门限；夹具报告明确 `collision_checked=false`，不用于证明牙面避碰。

最终新增回归测试同时检查动态过程实际工作段倾角、工作侧及相对表面的误差。动态控制复用项目现有偏置力和阻尼速度前馈的仿射位置伺服逻辑，没有改变模型重力或降低碰撞/误差阈值。

## 验收边界

本次完成局部系修正和平滑、几何候选、可执行性筛选、全局连接搜索、有限采样运动检查及失败定位接口；没有得到从 home 进入并完整执行真实牙面短段的通过结果。局部/分牙凸分解、工作侧实测标定、允许接触阶段与力控制、自动避障入口及断段转移仍未实现。程序保留 `execution_ready=false` 和 `geometry_certified=false`，旧版整段几何模式动画不计作本方案的碰撞验收结果。
