# UR7e 轨迹运行说明

本目录的 `ur7e_saved_trajectory_executor.py` 和 `trajectory.json` 配对使用。把这两个文件复制到机械臂工作站同一目录即可，不需要克隆本项目，不需要携带原始导出目录，也不需要 `deployment.json`。本 README 可一同复制。

## 1. 工作站准备

- Linux、Python 3、NumPy，以及现场 ROS 1 / Catkin 环境。
- 已安装 `moveit_commander`、`rospy`、`actionlib` 和 `moveit_msgs / trajectory_msgs / sensor_msgs` 消息包。
- 先启动与实际 **UR7e** 匹配的驱动、机器人描述和 MoveIt `move_group`；在运行脚本的终端加载现场 ROS / Catkin 环境。只启动驱动还不够。
- 机器人程序、控制器和 MoveIt 执行通道应已按现场配置准备好。机器人 IP 由驱动配置，控制脚本不填写 IP。

本项目尚未核验这台 UR7e 的实际 launch 命令、模型和标定，不能直接沿用参考工程的 UR3e 启动配置。

默认接口沿用旧程序：规划组 `manipulator`、基座帧 `base_link`、末端帧 `tool0`；关节状态 `/joint_states`，执行 action `/execute_trajectory`。现场不一致时，使用 `--group`、`--base-frame`、`--tip-frame`、`--joint-states`、`--execute-action` 覆盖，完整选项见：

```bash
python3 ur7e_saved_trajectory_executor.py execute --help
```

## 2. 检查轨迹与起点

在两个文件所在目录运行，离线检查不连接 ROS 或机器人：

```bash
python3 ur7e_saved_trajectory_executor.py inspect --trajectory trajectory.json
```

启动现场驱动和 MoveIt 后，执行只读检查：

```bash
python3 ur7e_saved_trajectory_executor.py check --trajectory trajectory.json --out logs/check_001
```

`check` 不发送运动或停止命令。运行 `check` 或 `execute` 前，机械臂必须已处于 JSON 第一个点附近并停稳，默认各关节偏差不超过 `0.01 rad`（约 `0.573°`）。任一关节超过此偏差，会报告 `joint_position_mismatch` 并拒绝发送轨迹；脚本不会自动移动到首点。

当前配对 `trajectory.json` 的起始关节角如下，按 JSON 中的关节顺序排列：

| 关节 | 角度（°） | 弧度（rad，显示至六位小数） |
| --- | ---: | ---: |
| 基座 `shoulder_pan_joint` | 78.28 | 1.366244 |
| 肩部 `shoulder_lift_joint` | -46.01 | -0.803026 |
| 肘部 `elbow_joint` | 65.01 | 1.134639 |
| 腕部 1 `wrist_1_joint` | **-273.56** | -4.774523 |
| 腕部 2 `wrist_2_joint` | 29.12 | 0.508240 |
| 腕部 3 `wrist_3_joint` | -19.07 | -0.332834 |

腕部 1 保留 **-273.56°**。当前脚本直接比较实际关节角，不进行加减 `360°` 的归一化，因此不能用 `86.44°` 替代。

就位顺序：通过现场示教或已有控制程序到达上述关节姿态并停稳，再运行 `check`；检查通过后运行 `execute --keyboard`，显示就绪后按 S。到起点的运动路径需另行确认。本条轨迹的终点与起点完全一致，正常完整执行后会回到上述姿态；中途终止不会自动返回。

首点的原始数值可这样查看，数组顺序对应关节名，数值单位为 rad：

```bash
python3 -c 'import json; d=json.load(open("trajectory.json")); print(d["joint_names"]); print(d["points"][0]["positions"])'
```

关节状态必须包含六关节的位置、速度及有效时间戳。起点不符、状态过期或组内关节不匹配时，会拒绝执行。不要修改轨迹来凑当前姿态。

## 3. 启动、停止与速度

原速运行：

```bash
python3 ur7e_saved_trajectory_executor.py execute --trajectory trajectory.json --keyboard --out logs/run_001
```

等终端显示 `Ready. Press S to start` 后：

- 按 **S / s**：发送一次完整轨迹。
- 按 **空格**：开始前取消；运动中取消并请求停止，随后核对执行状态和新的关节速度确认停稳。
- 按 **Ctrl+C**：中断并尝试停止。

保持 Linux 前台终端输入焦点；不需要按回车。空格是终止，不是暂停，不能续跑。再次执行前重新检查起点，并换一个日志目录。软件停止不能代替实体急停。没有 `--keyboard` 时，脚本通过预检后立即发送轨迹。

两倍速运行：

```bash
python3 ur7e_saved_trajectory_executor.py execute --trajectory trajectory.json --keyboard --speed-factor 2 --out logs/run_2x_001
```

也可用 `--speed-factor 0.5` 半速运行。倍率只改变发送时的时间和导数，不修改 JSON：两倍速时间减半，速度乘 2，加速度乘 4。原速约 `124.880 s`，两倍速约 `62.440 s`，实际时长还受驱动速度缩放影响；按键监听不改变点间时序。

大于 1 倍速需要现场机器人模型 / MoveIt 提供六关节有效速度、加速度上限，缺失或超限会拒绝。可以用 `--joint-velocity-limits`、`--joint-acceleration-limits` 分别补充六个现场确认值，顺序与 JSON 一致、单位 rad/s 和 rad/s²；不填写猜测值，也不能放宽已知模型上限。旧程序的 `0.1 / 0.075` 是规划倍率，不能作为这些绝对限值。

## 4. 日志与常见问题

每次 `--out` 必须是新的目录。结果写入 `run_report.json` 和 `states.jsonl`，不要只看退出码：

| 状态 | 含义 |
| --- | --- |
| `completed` | 执行成功，末点和停稳检查通过 |
| `operator_stopped` | 人工终止，并已确认停稳，不代表整条轨迹完成 |
| `cancelled_before_start` | 开始前人工取消，未发送轨迹 |
| `fault_latched` | 执行或停止确认异常，查看报告原因 |

若提示配对哈希不符，恢复配对的原文件或在项目中重新核验、构建，不手工修改脚本哈希。若缺少 ROS / MoveIt 模块，检查运行终端加载的环境和 Python 解释器。状态超时、起点不符和 action 不可用应按报告处理，不能以增大容差代替现场排查。

## 5. 数据和验证范围

`trajectory.json` 仅含六个 `joint_names` 和 16006 个 `points`。每点的 `positions / velocities / accelerations / time_from_start` 分别使用 rad、rad/s、rad/s²、s。脚本没有内嵌轨迹数组，仅保留配对用的 SHA256。轨迹包含规划好的进场、沿线和退场；脚本不额外规划起点或返回运动。

这是已批准的 `geometry_only` 位置回放客户端，**不包含 TCP 跟踪、力阈值和独立看门狗监控，也不是恒力控制**。几何回放与 Mock 检查不能证明固定牙模接触运行、避碰、动力学或实机验证已通过。

本机 MuJoCo 重放记录与动画保存在项目 `outputs/ur7e_delivery_replay_20261116/`；该目录是仿真产物，工作站控制不需要它。交互查看原始场景与规划可在本项目根目录运行：

```powershell
.\.venv\Scripts\python.exe -m planning.view --plan outputs/planning/session_straight_return_35_m20_15_20261116 --speed 1
```

MuJoCo 查看器中空格是暂停 / 继续，N / P 切换路点，R 重新播放，与真机脚本空格终止的含义不同。上述查看器读取原始规划目录；本次重放验收另直接读取交付 JSON 并核对其来源与插值，结果见该输出目录的报告。

2026-11-16 本次重放结果：MuJoCo 3.14.0 下，交付 JSON 与原导出的位置、速度、加速度及时间逐项一致；全部路点及每段三个内部位置共 64021 个采样的运动学核对通过，采样关节未越限，末点与首点关节角完全一致。相对原规划的最大 TCP 位置误差约 `1.18e-9 mm`。这些是相同仿真模型中的数据一致性结果，不表示实机精度。

`overview.gif` 显示完整机械臂，`closeup.gif` 显示牙面附近运动，均为完整轨迹的 **5 倍速预览**，不是把控制轨迹改成 5 倍速。`stages.png` 是关键阶段截图，`replay_report.json` 保存量化结果；未进行动力学积分、碰撞验收或真机运动。
