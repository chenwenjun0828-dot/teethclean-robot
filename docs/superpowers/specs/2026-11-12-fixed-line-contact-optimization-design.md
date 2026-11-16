# 固定牙龈线上的密集节点姿态优化方案

日期：2026-11-12

状态：用户讨论中的修订草案。采用每节点三个姿态变量、初始化锁定 IK 分支、优化中保持该分支；只进行节点级优化和验收。尚未实现或进行性能验证。

## 1. 本轮确定的范围

给定单一区域的合理牙龈线，将其视为固定的位置目标；密集采样后，每个节点仅优化工具朝向。整区节点联合优化，尽量接近统一参考朝向，节点姿态变化平顺，各节点满足接触与碰撞要求。

本轮决定：

- 输入牙龈线合理，规划器只读使用，不检查其是否在牙面上，不做投影、裁剪、补线、偏移或重新拟合。
- 工作点目标沿原线，`standoff_mm=0`。
- 工作段按工具表面到指定工作点的欧氏距离自动划分，球形区域半径由 `working_segment_radius_mm` 配置，默认 1.0 mm。
- 指定工具工作段允许与已加载的上下颌整个牙齿模型接触，不按目标牙、当前节点邻域或牙位标签限制接触对象；物理目标仍为不穿透，数值验收使用显式配置的 `contact_tolerance_mm`。
- 每个节点三个姿态参数作为连续优化变量。
- 只对给定节点进行优化约束和最终节点验收。
- 本轮不检查节点之间的碰撞、姿态偏差、跟线误差或关节插值超限。
- 本轮不采用基于区间碰撞的自适应约束点加密，也不优化关节样条控制节点。
- IK 采用两阶段策略：初始化时沿参考位姿序列跟踪并锁定一条参考关节序列和分支；优化时在该固定分支内求局部 IK。
- 关节配置是姿态变量的派生量，不增加连续优化变量；加入关节参考跟踪和离散平滑目标。
- warm-start 和软惩罚用于提高收敛性；固定分支由显式标签、关节角展开和非奇异局部跟踪规则共同约束，不能只靠“取最近解”。

密集节点作为连续运动的离散近似。验收结论限定为保存的这些节点；节点数量本身不能推出节点之间也满足全部约束。因此报告明确区分节点通过与连续路径通过，当前只输出前者。

## 2. 输入、变量和派生量

| 项目 | 定义 | 角色 |
| --- | --- | --- |
| 源曲线 `p_line(s)` | perception 提供的区域牙龈线 | 固定输入 |
| 节点源参数 `s_i` | 一次只读采样得到的递增参数 | 本次求解中固定 |
| 节点目标位置 `p_i` | `world @ p_line(s_i)` | 固定输入 |
| 三个输入角度 | `pose_angles_deg` | 生成参考朝向 |
| 参考朝向 `R_ref` | 同一区域参考系下的固定旋转矩阵 | 固定输入 |
| 姿态扰动 `delta_i` | 相对 `R_ref` 的三维旋转向量 | 连续优化变量 |
| 节点朝向 `R_i` | `R_ref @ Exp(delta_i)` | 由变量计算 |
| 节点位姿 `T_i` | 由固定 `p_i` 和变量生成的 `R_i` 组成 | 优化结果 |
| 初始化参考位姿 `T_i_ref` | 固定 `p_i` 与统一 `R_ref` 组成的位姿 | 固定输入的派生量 |
| 参考关节序列 `q_i_ref` | 沿 `T_i_ref` 顺序跟踪得到的关节配置 | 初始化完成后固定 |
| 分支标签 `mu` | 同一条正规解析分支的稳定标签 | 初始化完成后固定 |
| 节点关节配置 `q_i` | `F_mu(T_i)`，固定分支内的局部逆运动学 | 优化中的派生量 |
| 已接受迭代的 `q_i_prev` | 上一个已接受外层迭代的节点关节配置 | 仅作为局部 IK 数值初值 |
| 连续插值和时序 | 后续机器人运动表示 | 不作为本轮节点间验收对象 |

假设当前 TCP 就是指定工作点。若以后定义非零的工作点偏置，则 TCP 平移需按姿态补偿，使实际工作点仍为 `p_i`；这不改变原牙龈线。本轮按零偏置设计。

## 3. 参考朝向和姿态变量

### 3.1 参考朝向

复用当前 `simple` 的区域固定参考方式：

1. 使用 `representative_source()` 选取原区域源线累计长度中点附近的真实采样点。
2. 使用代表牙面法线与世界重力建立参考系。
3. 用输入的三个角度调用已有 `tool_rotation()`，得到 `R_ref`。
4. 全区域使用相同的 `R_ref`。

三个输入角度是用户给定参考朝向的方式。优化变量使用相对此朝向的小旋转，不直接将旋转向量分量解释成原来的三个欧拉角。

### 3.2 每节点三个变量

设共有 `N` 个节点：

\[
D=[\delta_0,\ldots,\delta_{N-1}]\in\mathbb{R}^{N\times3},\qquad
R_i=R_{ref}\operatorname{Exp}([\delta_i]_\times).
\]

姿态初值取 `delta_i=0`，表示所有节点都使用参考朝向。初始化先对这些位姿求出参考关节序列；只有整段参考 IK 初始化成功，才启动本次连续优化。参考姿态发生碰撞不等于初始化失败，碰撞由后续优化修正。

内部角度使用弧度。使用 `max_reference_deviation_deg` 转成旋转向量长度上限：

\[
\|\delta_i\|_2\leq\theta_{max}.
\]

在限制到小于 180 度的参考邻域内，这个长度就是实际相对旋转角。建议起点仍为 5 度，允许用户调整。它与旧 simple 的三个输入角分别 ±5 度不同，报告中使用 `orientation_bound_kind="so3_geodesic"`。

## 4. 密集节点采样

只读调用 `path.samples(cfg.pose_spacing_mm * 0.001)` 或等价的源曲线采样接口，保留首尾和源分界点。采样不修改曲线系数、牙位顺序或源长度。

同一组节点用于：优化变量、节点约束、结果保存、独立节点验收。不存在另一套区间内部碰撞验收点，也不在求解后通过增加检查点扩大本次通过结论。

节点间距使用现有 `pose_spacing_mm`，可先试 0.1 mm，再比较 0.05 mm、0.25 mm 下的计算量和节点结果。该建议是计算网格起点，不是连续无碰撞证据。

粗略地，长度为 `L`、间距为 `h` 时，节点数量约为 `L/h+1`，连续变量数量约为 `3*(L/h+1)`。实际节点数还受源分界点和采样实现影响。例如 50 mm 的线按约 0.1 mm 采样，约有 501 个节点、1503 个姿态变量。

节点参数在本次求解中固定，不将节点沿线位置作为优化变量，也不允许删掉难以求解的节点。

差分要求源参数严格递增。只读采样时可以按数值容限合并相同源参数的重复记录并保存来源映射，不删除不同参数的目标节点，也不因两个位置相近就合并。若独立参数少于三个，则报告 `insufficient_pose_nodes`，不通过移动或补造源线修复。

## 5. 节点级目标函数

首版目标包括工具姿态和关节配置两部分：

\[
J(D)=w_R J_{ref}+w_1 J_{step}+w_2 J_{smooth}
     +w_q J_{q,ref}+w_{q1}J_{q,step}+w_{q2}J_{q,smooth}.
\]

### 5.1 接近参考朝向

\[
J_{ref}=\sum_i a_i\frac{\|\delta_i\|^2}{\theta_{scale}^2}.
\]

`a_i` 为归一化节点积分权重，`theta_scale` 初始使用姿态上限。这样改变节点密度时，不会仅因节点更多而改变目标项的相对权重。

### 5.2 相邻变化和二阶平滑

为了处理源分界点造成的不等间距，在归一化源参数 `u_i=s_i/path.end` 上定义：

\[
h_i=u_{i+1}-u_i,\qquad
v_i=\frac{\delta_{i+1}-\delta_i}{h_i},\qquad
b_i=\frac{2(v_i-v_{i-1})}{h_i+h_{i-1}}.
\]

节点参考权重取梯形积分权重：端点为相邻 `h/2`，内部节点为 `(h_(i-1)+h_i)/2`，再除以全部节点权重之和得到 `a_i`。二阶项内部权重取 `c_i=(h_(i-1)+h_i)/2`。定义：

\[
J_{step}=\frac{\sum_i h_i\|v_i\|^2}{\theta_{scale}^2\sum_i h_i},
\qquad
J_{smooth}=\frac{\sum_{i=1}^{N-2}c_i\|b_i\|^2}
{\theta_{scale}^2\sum_{i=1}^{N-2}c_i}.
\]

这些是统一参考切空间中的离散姿态平滑项，适合当前小角度邻域，不要求构造节点间姿态曲线。

如果需要限制相邻节点的实际旋转跳变，还可增加节点对约束：

\[
angle(R_i^T R_{i+1})\leq\theta_{step,max}.
\]

这个约束只比较两个已保存节点，不检查它们之间的插值姿态。最大步长参数先设为可选，避免没有确认的硬阈值直接改变可行域。

### 5.3 关节参考跟踪

固定分支内，令 `q_i(D)=F_mu(T_i(delta_i))`。定义固定关节尺度矩阵 `S_q=diag(q_scale_j)`，尺度为正，内部单位为弧度：

\[
J_{q,ref}=\sum_i a_i
\left\|S_q^{-1}(q_i(D)-q_i^{ref})\right\|^2.
\]

该项使当前关节配置保持在参考配置附近，减少局部 IK 的大幅变化。它是软目标，不等同于“关节不能动”，也不能代替分支硬约束。`q_i_ref` 在本次优化中不更新；`q_i_prev` 只改善求解初值，不能替代目标函数里的固定参考。

关节尺度用于统一不同关节的惩罚量级，不是关节限位或允许偏离范围。若后续需要硬参考关节邻域，必须作为单独参数和约束引入，不能隐含在归一化中。

### 5.4 关节一阶变化和二阶平滑

对与姿态相同的 `u_i`、`h_i`，定义：

\[
z_i=S_q^{-1}q_i,\qquad
v_i^q=\frac{z_{i+1}-z_i}{h_i},\qquad
b_i^q=\frac{2(v_i^q-v_{i-1}^q)}{h_i+h_{i-1}}.
\]

使用与第 5.2 节相同的 `h_i`、`c_i` 及各自权重和，累加 `||v_i^q||^2` 和 `||b_i^q||^2`，分别得到 `J_q,step` 和 `J_q,smooth`；关节已经经过 `S_q` 归一化，不再除以 `theta_scale`。三类关节目标分别表达“不要远离参考”“相邻关节配置不要突变”“关节变化趋势平顺”。

这些量是沿源参数的几何差分，不是时间速度或加速度。时间分配仍属于后续阶段。

所有关节差分在统一展开后的关节角上计算，不能对相邻差值取模来隐藏接近 `2*pi` 的实际跳转。

## 6. 节点级硬约束

### 6.1 固定位置和参考姿态边界

节点位置在构造位姿时固定，不作为连续优化变量。姿态满足 `||delta_i|| <= theta_max`。每次固定分支 IK 成功后，用 FK 检查关节解是否达到目标位姿，并记录位置和旋转数值容限。

这里的 FK 检查只针对节点，不扩展到节点之间。

### 6.2 指定工作段允许与整个牙齿模型接触

工作段采用以指定工作点为球心的球形区域，默认半径 1.0 mm，通过 `working_segment_radius_mm` 调整。这里的 1 mm 是三维欧氏距离的半径，不是直径，也不是沿弯曲工具轴线量取的长度。

令 `c_work` 为工具 TCP 局部坐标系中的指定工作点，本轮为 `[0, 0, 0]`；`v_k` 为同一坐标系下的工具网格顶点，内部长度单位为米：

\[
r=10^{-3}\,\texttt{working\_segment\_radius\_mm},\qquad
\mathcal{W}_r=\{x\in\text{tool surface}:\|x-c_{work}\|_2\leq r\}.
\]

首版使用原始三角面片的保守离散划分。三角面 `f` 只有三个顶点都位于球内或球面上，才归为工作段：

\[
f\in F_{work}\quad\Longleftrightarrow\quad
\max_{k\in\operatorname{vertices}(f)}\|v_k-c_{work}\|_2\leq r.
\]

所有其他面片归为非工作段，包括一部分在球内、另一部分在球外的边界面片。首版不裁切这些面片，不按面片中心或任意一个顶点位于球内就放行；两个分组互斥且覆盖全部工具三角面，不能丢弃边界面片。因此离散工作段是球内工具表面的保守子集，其边界精度受工具网格分辨率影响。

划分仅作用于指定工具几何 `ur_end_scaler_geom`。先将网格顶点经实际几何变换转换到 TCP 局部米制坐标，再计算距离，不能混用 STL 毫米坐标与仿真米制坐标，也不能把球内的传感器、安装件或其他机器人部件并入工作段。该局部分组随工具刚体运动，不按每个节点的世界坐标重新划分。

`working_segment_radius_mm` 必须为有限正数，默认值为 1.0。用户可以在每次求解前调整半径；本次优化中半径和面片分组固定，不作为优化变量。划分结果为空时报告 `empty_working_segment`，不自动扩大半径。

指定工作点仍固定在输入牙龈线节点上，接触许可适用于上述工作段与全部已加载牙齿模型：

| 工具或机器人部位 | 接触对象 | 规则 |
| --- | --- | --- |
| 指定工作段 | 当前目标牙的任意区域 | 允许零间隙接触，不允许穿透 |
| 指定工作段 | 邻牙、其他牙和对颌 | 允许零间隙接触，不允许穿透 |
| 非工作段、杆身、传感器和安装件 | 全部牙齿模型 | 继续按各部位的不穿透和间隙规则检查 |
| 机器人连杆及禁止的自碰撞对 | 对应障碍物或机器人部件 | 继续按禁止接触规则检查 |

接触许可按完整的已加载上下颌牙齿网格定义，不再要求属于当前目标牙，也不限制在节点周围的小邻域。网格中的未标注面片同样适用此范围；`Geometry.face_tooth_ids` 和原面索引可以用于诊断，但不再用于限制工作段的接触对象。因此不需要牙位边界处的双目标接触许可或目标牙邻域掩码。

工作段按上述半径规则自动生成明确的工具面片分组，不再要求手工标注。保存原始面片索引并绑定工具网格指纹、网格到 TCP 的变换、工作点和半径；更换网格、TCP 定义或半径后重新生成分组。不能仅根据世界坐标中的接触点靠近 TCP，就忽略其他部件的碰撞。工作段对任意牙面仍需检查穿透和错误侧接触，不能整体跳过它与牙颌的碰撞查询。

允许整个工作段接触不意味着要求它处处接触；必须沿输入线保持的仍是指定工作点。其他工作段位置可以接触或分离。这一调整不修改原线、固定节点或 IK 策略。

#### 数值接触容差

新增 `ContactPolicy.contact_tolerance_mm`，专门用于指定工作段与全部已加载牙齿模型的接触判断。参数必须显式填写，为有限的非负数，不接受布尔值、负数、NaN 或无穷大；`0.0` 表示不接受负间隙。当前不设未经校准的默认值，第 12 节的 `0.01` mm 仅为原型配置示例。

令 `d_ik` 为节点 `i` 的第 `k` 个相关几何查询返回的可靠局部有符号间隙：正值表示分离，零表示接触，负值表示穿透侧的几何重叠。内部统一使用米，容差只换算一次：

\[
\varepsilon_c=10^{-3}\,\texttt{contact\_tolerance\_mm}.
\]

| 原始间隙 | 数值分类 | 工作段是否通过该项检查 |
| --- | --- | --- |
| `d_ik > epsilon_c` | 分离 | 是 |
| `-epsilon_c <= d_ik <= epsilon_c` | 容差内接触 | 是，保留原始负间隙记录 |
| `d_ik < -epsilon_c` | 超出容差的穿透 | 否 |

物理目标仍是 `d_ik >= 0`；数值优化中的对应硬约束写为 `g_ik(D) = d_ik(D) + epsilon_c >= 0`，违反量为 `max(0, -epsilon_c - d_ik)`。该函数由规划模块的碰撞适配器构造，用户只配置容差。优化过程中允许有违反约束的试探节点并尝试修复；最终独立验收必须逐项满足同一阈值，不能仅凭优化器的成功标志通过，也不能再叠加求解器的可行性容差来扩大接触带。

例如仅为解释边界，取 `contact_tolerance_mm=0.01`：原始间隙为 `-0.005` mm 或恰好 `-0.01` mm 时按容差内接触接受，`-0.02` mm 则拒绝。接受轻微负间隙只表示“容差内不穿透”，不能报告为精确零穿透或证明真实工具没有进入牙体。

该参数与 `working_segment_radius_mm` 相互独立，不扩大工作段，不移动牙龈线节点，不改变 `standoff_mm=0`，也不自动放宽非工作段、机器人或自碰撞的间隙规则。容差在一次求解中固定，不作为优化变量，失败后不自动增大。

上述分类以几何查询结果有效、局部符号可信且没有错误侧接触为前提。开放网格无法可靠判断某处符号、查询失败或侧别不明确时，报告该节点接触无法验证；不能仅因无符号距离接近零，就用容差将其判为合法接触。

### 6.3 工具碰撞与机械臂碰撞的依赖不同

工具在节点的刚体位姿 `T_i` 已知，因此工具网格与双颌之间的接触/碰撞可直接检查，不依赖选哪个机器人 IK 分支。

机械臂本体的形状位置依赖关节配置。同一个工具位姿可对应不同的机械臂碰撞结果，因此不能仅凭 `T_i` 宣称整机无碰撞。

整机约束评价使用确定的 `q_i=F_mu(T_i)`，不在每次评价时搜索“任意一个无碰撞分支”。固定分支中的关节限位、FK 精度和整机节点碰撞必须通过，即使另一个分支存在无碰撞解，也不在本次优化中换支。

### 6.4 固定分支和局部 IK 有效域

每个节点均要求：

1. 关节解具有锁定的 `mu` 标签，并使用与参考序列一致的关节角展开。
2. `q_lower <= q_i <= q_upper`，并满足节点 FK 位置和旋转容限。
3. 经统一长度尺度归一化的 TCP 雅可比最小奇异值高于配置下限，保持局部逆映射可用。
4. 若启用相邻关节步长上限，则对两个节点的实际展开关节角施加 `max(abs(q_(i+1)-q_i)) <= joint_step_rad`。

分支标签和关节参考惩罚承担不同职责：前者限制允许的解，后者在该解族中偏好较小变化。分支标签相同也不自动保证远离奇异点，因此需要单独的非奇异检查和局部步长控制。

固定分支中无解、越出关节边界或进入奇异邻域的试探步应被拒绝或缩小，而不是改选其他分支。邻点关节步长约束仅比较已有节点，不包含它们之间的插值检查。

## 7. 碰撞查询接口

优化阶段需要几何距离/局部分离信息；最终节点验收使用原始网格和相同接触策略。当前 `check()` 碰撞时的 `-1` 失败标记不能作为可导距离或穿透深度。

建议接口如下，均为拟新增接口：

```python
class NodeCollisionModel:
    def tool_pose_distances(self, pose_world, context, pair_ids):
        """Return fixed-order local tool-distance models for optimization."""
        raise NotImplementedError

    def check_tool_node(self, pose_world, context):
        """Classify tool/scan contacts at one saved node."""
        raise NotImplementedError

    def check_robot_node(self, joints, context):
        """Check the full robot at one specified IK solution."""
        raise NotImplementedError
```

一次求解中的节点、几何对顺序和约束向量维度固定。不能随近邻查询结果改变返回长度。可以先枚举几何对，再用包围体筛选昂贵查询。

指定工作段与非工作段按第 6.2 节的固定半径和全顶点规则分组，边界混合面片完整归入非工作段，不得丢失。不能在第一个接触被判合法后，忽略同一节点上的其他禁止接触或超出容差的工作段穿透。每个节点都要覆盖工作段与全部已加载牙齿网格的查询，不能仅查询当前目标牙。优化和验收共用同一个 `ContactPolicy` 及其 `contact_tolerance_mm`，查询保留原始有符号间隙，不能先将负值截断为零。开放口扫网格的全局内外符号及零距离梯度不作未经验证的假设。

这些查询全部在节点执行。没有 `check_edge()`、样条中点碰撞检查或节点间连续碰撞证明步骤。

## 8. 代码中的位姿构造与问题接口

### 8.1 节点位姿构造

```python
import numpy as np
from scipy.spatial.transform import Rotation


def build_node_poses(points_world_m, reference_rotation, deltas_rad):
    points = np.asarray(points_world_m, float)
    deltas = np.asarray(deltas_rad, float)
    if points.shape != deltas.shape or points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Expected matching (N, 3) points and rotation vectors")
    rotations = reference_rotation @ Rotation.from_rotvec(deltas).as_matrix()
    poses = np.repeat(np.eye(4)[None, :, :], len(points), axis=0)
    poses[:, :3, :3] = rotations
    poses[:, :3, 3] = points
    return poses
```

优化器的连续变量是 `deltas_rad.reshape(-1)`。节点坐标只读进入该函数，不被更新。

### 8.2 核心问题接口

```python
class NodeOrientationProblem:
    def objective(self, x):
        """Orientation and fixed-branch joint reference/smoothness costs."""
        raise NotImplementedError

    def objective_jacobian(self, x):
        raise NotImplementedError

    def orientation_bound_residual(self, x):
        """One norm-bound residual per saved node."""
        raise NotImplementedError

    def tool_separation_residual(self, x):
        """Contact-aware tool constraints evaluated at saved nodes only."""
        raise NotImplementedError

    def robot_separation_residual(self, x):
        """Full-robot constraints at the derived fixed-branch joint nodes."""
        raise NotImplementedError

    def joint_limit_residual(self, x):
        raise NotImplementedError

    def joint_step_residual(self, x):
        """Optional differences between saved neighboring joint nodes."""
        raise NotImplementedError

    def decode_poses(self, x):
        raise NotImplementedError
```

IK 与机械臂相关的评价通过第 9 节的固定分支适配器接入。一次 `evaluate(x)` 统一产生位姿、关节节点、目标项和约束项，避免不同回调使用不同种子得到不一致结果。

节点参考项是逐点项，一阶项只连接相邻节点，二阶项只连接相邻三个节点，可以利用这些结构计算梯度和 Hessian。密集节点可能产生上千连续变量，不能默认对每个变量都运行一次全节点碰撞差分。

求解外层需要支持固定分支 IK 有效域中的试探步拒绝，优先按稀疏 SQP/信赖域结构设计。SciPy `trust-constr` 保留为后端候选，但接入前必须解决“试探位姿没有该分支 IK”时的评价契约，不能直接依赖异常、`NaN` 或固定大惩罚让通用求解器自行恢复。具体规则见第 9.6 节。

## 9. 两阶段 IK：初始化锁定，优化保持

### 9.1 问题的数学形式

第一阶段产生本次求解固定的数据 `({q_i_ref}, mu)`。第二阶段的连续变量仍然只有 `D`，关节配置通过固定分支逆映射得到：

\[
T_i(D)=\begin{bmatrix}R_{ref}\operatorname{Exp}([\delta_i]_\times)&p_i\\0&1\end{bmatrix},
\qquad q_i(D)=F_\mu(T_i(D)).
\]

`F_mu` 只定义在该分支可达、关节限位内且远离奇异点的局部区域。本问题是在这个区域内优化，不声称固定分支在全部姿态上都有解。

因此 `N=1148` 时，连续变量仍为 `3444` 个，而不是再增加 `6888` 个关节变量。初始化中的分支尝试是一次性的离散工作，不进入每轮连续优化。

### 9.2 第一阶段：沿统一参考姿态建立参考关节序列

1. 对全部固定节点构造 `T_i_ref=[R_ref, p_i]`，对应 `delta_i=0`。
2. 使用调用方提供的起始关节配置 `q_start`；没有提供时，使用现有 `robot.home` 作为初始偏好。这只是分支排序参考，不表示进场运动已经规划。
3. 在首节点枚举带稳定标签的 IK 候选，按距离起始配置、关节限位余量和非奇异性确定尝试顺序。
4. 对一个起始标签 `mu`，沿节点顺序跟踪：`q_0_ref` 使用起始候选，`q_i_ref` 使用 `q_(i-1)_ref` 作为 warm-start，并只接受同一 `mu` 内的解。
5. 在每个参考节点检查关节角展开、限位、FK 精度、非奇异条件，以及已启用的相邻关节步长上限；不检查节点之间的运动。
6. 某条起始分支中途不能完成时，可在本次初始化内尝试下一条起始分支。找到整段有效的参考序列后，立即锁定 `mu` 和全部 `q_i_ref`，不再进行分支竞争。

初始化记录工具与整机碰撞诊断，但不以“参考姿态已经无碰撞”作为启动优化的前提。否则会把本应由姿态优化修复的碰撞提前拒绝。参考序列必须满足 IK 有效域要求，碰撞约束允许从不可行状态开始优化。

如果全部起始分支都无法沿统一参考姿态完成跟踪，则返回 `reference_ik_initialization_failed`，记录首个失败节点及原因。即使在参考姿态附近可能存在其他可行姿态，本首版也不自动改变参考角度、移动节点或放宽关节/奇异阈值；该失败只说明当前初始化策略没有成功，不是整个姿态优化问题无解的证明。

### 9.3 分支标签和关节角展开

现有 `Robot._branches()` 按肩部分支、腕部轴向选择和肘部分支生成 UR 解析解，`Robot.solutions()` 再按种子距离排序。返回列表下标会随种子、限位筛选和候选消失而变化，不能用作 `mu`。

拟新增稳定标签 `mu=(shoulder_choice, wrist_choice, elbow_choice)`，在解析生成步骤就附加标签，再进行 FK 和限位筛选。标签应对应生成器中的三个解析选择，而不是未经验证地使用某个关节角的正负号。需要为标签定义保存版本，避免算法调整后误读旧结果。

关节角的 `2*pi` 等价表示与解析分支标签分开处理：

- 初始化沿前一个节点选择连续展开，且始终满足机器人实际关节限位。
- 优化中沿该节点的固定参考展开选择局部连续表示；原始 `atan2` 跨越正负 `pi` 时，允许对应展开整数改变以维持实际角度连续。
- 不把数值上相差 `2*pi` 的关节角直接当作可以瞬时互换的机器人配置。
- 保存初始展开信息和最终实际角度；目标函数、限位和差分都使用实际展开角度。

全程锁定的是一个正规分支及其局部连续角度表示，不是 IK 候选列表中的编号。接近分支合并或退化位置时，即使标签仍可计算，也不能绕过非奇异条件继续跟踪。

### 9.4 第二阶段：在固定分支内评价每个姿态

对外层优化的一个试探 `D_trial`：

1. 用固定 `p_i` 和试探 `delta_i` 构造全部 `T_i`。
2. 对每个节点，首选该节点上一个已接受迭代的 `q_i_prev`，第一次使用 `q_i_ref`，进行固定 `mu` 内的局部 IK。
3. 若首选种子仅发生数值不收敛，先用该节点的固定 `q_i_ref` 进行同分支重试；仍不成功则返回带节点索引的无效评价，不能转到其他分支。
4. 对解检查标签、展开、关节限位、FK 误差和非奇异条件，得到 `q_i(D_trial)`。
5. 用该关节序列计算关节参考/平滑项及机械臂节点碰撞；工具碰撞仍可直接由 `T_i` 计算。

初始化的 warm-start 来自前一个空间节点；优化中的 warm-start 来自同一个节点的上一个已接受迭代。优化中不重新做全路径图搜索，也不让每个节点的种子递归依赖本轮前一节点的解。

`q_i_ref` 始终不变。`q_i_prev` 仅在外层接受整个试探步后统一更新；被拒绝的线搜索点、数值差分点和单个节点的临时成功结果不能污染它。

同一个 `D` 的目标和约束必须一致，不能随回调顺序或 warm-start 历史改变。为此固定分支解和角度展开必须确定，参考种子作为稳定回退；相同 `D` 使用同一份带状态的评价结果。若两个合格种子得到不一致的同分支解，应报告分支/展开歧义，而不是取其中成本更低的一个。

### 9.5 局部 IK 的实现边界

拟新增 `FixedBranchIK`，不直接把现有 `robot.solutions(target, seed)[0]` 当成固定分支求解器。现有排序只表达距离种子的偏好，没有提供显式分支保证。

局部数值 IK 可以使用机器人已有的 `fk()` 和 `jacobian()`，在 warm-start 周围进行限步长、带关节边界的阻尼迭代；每次接受的内部步都必须符合固定分支规则。只给求解器一个初值，不能替代分支检查。

当前 UR 模型已经支持解析解，因此可以进一步提供 `solve_branch(target, mu, reference_lift)`：只计算锁定的解析分支并恢复局部连续角度表示，再用独立 FK 检查。这可以作为局部逆映射的精确实现，或局部数值迭代失败时的同分支回退。它不会重新排序和选择其他分支。

首版只在具有经过验证的稳定分支标签的机器人上启用该模式。通用机器人若只能提供无标签数值 IK，应报告不支持严格固定分支，不能悄悄退化为最近解策略。

以下为拟新增接口示意，不是现有可调用实现：

```python
class FixedBranchIK:
    def initialize_reference(self, reference_poses, q_start):
        """Track once; return immutable reference joints and branch metadata."""
        raise NotImplementedError

    def evaluate_nodes(self, poses, reference, accepted_joints):
        """Return fixed-branch joints or a structured invalid-trial result."""
        raise NotImplementedError

    def joint_sensitivities(self, poses, joints):
        """Return (N, 6, 3) derivatives on regular fixed-branch nodes."""
        raise NotImplementedError
```

### 9.6 无效试探步和碰撞不可行初值

外层应区分两种失败：

| 情况 | 处理 |
| --- | --- |
| IK 在锁定分支中有效，但有禁止碰撞 | 保留真实约束违反量，允许从碰撞不可行状态迭代 |
| 锁定分支无解、无法可靠展开、FK 不合格或进入奇异邻域 | 当前试探无法正常评价；缩小外层步长/信赖域并重试 |

每轮 SQP/信赖域在已接受点建立局部模型，生成姿态试探；先做固定分支 IK 有效域检查，再计算完整目标和碰撞约束。无效 IK 试探只影响步长和诊断，不修改参考数据或已接受的关节缓存。

若线搜索次数或最小步长达到上限，返回 `ik_domain_blocked` 或 `local_ik_numerical_failure`，同时保存最后一个已接受点、失败节点和残差。若所有试探均可评价但碰撞无法降低，则按约束违反量报告优化未找到可行解。停止条件同时检查目标/步长收敛和硬约束，不以代价下降替代可行性。

不能对 IK 失败返回全零约束、上一轮关节配置或固定大常数成本后声称这是同一连续目标。也不能未经接口适配就将这类结构化无效结果交给只接受有限数值回调的通用求解器。若使用 SciPy 后端，需要明确实现上述有效域控制，或先证明选用的局部区域内所有试探都能评价。

外层重新选择另一个分支属于新的独立初始化与求解，不属于本次优化中的恢复动作。首版只报告这种重启需求，不自动换支继续。

### 9.7 关节目标如何反传到三个姿态变量

在锁定分支且远离奇异点时，使用现有 `robot.jacobian(q_i)` 得到世界坐标 TCP 雅可比 `J_i`，其行顺序为平移、旋转。对固定工作点：

\[
J_i\,dq_i=
\begin{bmatrix}0_{3\times3}\\B_i\end{bmatrix}d\delta_i,
\qquad
G_i=\frac{\partial q_i}{\partial\delta_i}
=\operatorname{solve}\left(J_i,
\begin{bmatrix}0_{3\times3}\\B_i\end{bmatrix}\right).
\]

其中 `B_i=R_ref J_l(delta_i)`，`J_l` 是 SO(3) 左雅可比，表达 `R_ref Exp(delta_i)` 的世界坐标微小旋转；在 `delta_i=0` 时为 `R_ref`。不能把输入的三个角度导数或一个固定单位阵直接替代一般姿态下的 `B_i`。

利用 `G_i`，通过链式法则将关节参考偏差、相邻变化和二阶平滑的导数传回 `delta_i`。每个 `G_i` 只有 `6x3`；单节点关节配置只依赖自己的三个姿态变量，相邻节点耦合只来自差分成本和已启用的节点对约束。因此可保持块稀疏结构。

实现时解线性方程而不是显式求逆。用于奇异阈值的雅可比应先按固定特征长度 `ell` 将平移行除以 `ell`，再计算奇异值；米与旋转单位混合的原始矩阵不能直接套用没有尺度定义的阈值。阻尼 IK 可用于内部求解，但不能把阻尼近似逆直接宣称为精确的隐函数导数。

首版可用三个方向的局部差分校验 `G_i`：每次扰动只重算本节点同分支 IK，不能每扰动一个变量就重新求全部节点 IK 和碰撞。碰撞距离若没有可靠解析导数，也采用对应节点的局部差分或局部分离模型。

### 9.8 评价和接受的伪流程

```text
reference = initialize_reference(T_ref, q_start)
D = zeros(N, 3)
accepted = evaluate(D, reference, seeds=reference.q_ref)

while budget_remains:
    model = linearize_at(accepted)
    step = solve_sparse_local_subproblem(model)
    for alpha in decreasing_step_sizes:
        trial = evaluate(D + alpha * step,
                         reference,
                         seeds=accepted.q)
        if trial.ik_valid and merit_accepts(accepted, trial):
            D = trial.D
            accepted = trial
            break
    else:
        stop_with_diagnostics()
    if convergence_and_constraint_tolerances_met(accepted):
        break

audit_same_saved_nodes(accepted, reference)
```

`merit_accepts` 同时考虑原始目标和真实约束违反量；参考配置允许碰撞，因此不能要求所有中间试探都已经满足全部碰撞约束。所有数值初值、回退和缓存规则都服从固定 `mu`。

## 10. 修订后的完整流程

```text
读取 PreparedRegion、配准、机器人与三个参考角度
                         |
              保存原线快照与指纹
                         |
            只读密集采样固定节点 p_i
                         |
         构造统一 R_ref 和工具接触策略
                         |
      构造全部参考位姿 T_i_ref=[R_ref,p_i]
                         |
       初始化顺序跟踪 IK，锁定 q_i_ref 和 mu
                         |
       定义每个节点三个姿态变量 delta_i
                         |
     在固定 mu 内由每个试探姿态派生关节节点
                         |
    联合优化姿态/关节参考、平滑和节点碰撞约束
                         |
      同一批节点独立验收 FK、分支、接触与整机
                         |
       保存姿态节点、关节节点、参考和诊断
                         |
       [后续：进出场、时间分配和执行验证]
```

上图的“派生关节节点”和“联合优化”构成每轮评价的循环；参考初始化只发生在循环外。独立验收直接检查保存的最终关节节点，不重新调用最近解选择来替换它们。

不包含整条样条独立审计、不包含区间加密、不包含节点间姿态/碰撞检查。后续可以对选定关节序列做进出场和时间分配，但这些产物需要自己的验证，不作为本轮节点通过的隐含结论。

若优化失败，保留所有输入节点，报告失败节点的源参数、牙位、位姿和碰撞对；不删点、不移线、不自动放宽姿态上限。

## 11. 模块边界和与现有实现的关系

建议新增模式名仍为 `contact_opt`，但报告中明确 `formulation="dense_node_orientation"`。

```text
planning/contact_opt/
    config.py       节点间距、姿态边界、目标权重和接触容限
    reference.py    原线只读采样、R_ref 和节点位姿构造
    ik.py           参考关节跟踪、锁定分支、局部 IK 与敏感度
    policy.py       可配置半径的工作段分组与整个牙齿模型接触规则
    distance.py     工具节点距离、整机节点碰撞适配
    problem.py      3N 变量、离散目标与节点约束
    solve.py        稀疏外层、无效 IK 试探拒绝、接受点缓存
    audit.py        相同节点上的独立验收
```

`ik.py` 负责两阶段 IK，不负责碰撞或姿态目标权重。`problem.py` 通过统一评价获得同一批 `T_i` 和 `q_i`，再组织姿态/关节目标、工具/整机约束及块稀疏导数。

`planning/infrastructure/robot.py` 拟增加带稳定标签的候选、单分支求解和展开接口，复用既有 FK 与雅可比。新增接口不能改变旧 `solutions()` 的返回契约，旧模式的候选图搜索也不因新模式而被替换。

必要入口调整包括 `planning/config.py`、CLI/pipeline 参数透传和 `planning/pipeline.py` 分发。新模式的保存契约需要记录原线指纹、固定节点和姿态参数。

上一版的关节样条优化、`JointSpline.from_knots()`、区间加密以及基于整条样条的约束不再是本轮实施内容。现有 `simple`、`continuous` 的碰撞和执行验收代码保持其既有语义；本轮没有授权关闭它们的运行检查。

已有 TOPPRA 和关节轨迹结构可以作为之后对已选定关节序列进行时间分配的工具。不能因节点姿态优化通过，就宣称现有轨迹验证流程也已通过。

## 12. 参数草案

下列格式是拟新增配置，当前 CLI 尚未支持。

```json
{
  "planning_mode": "contact_opt",
  "formulation": "dense_node_orientation",
  "region": "outer_front",
  "motion_mode": "contact",
  "standoff_mm": 0.0,
  "pose_angles_deg": [6.37, 13.26, -0.4],
  "pose_spacing_mm": 0.1,
  "orientation_bound_kind": "so3_geodesic",
  "max_reference_deviation_deg": 5.0,
  "weight_reference": 1.0,
  "weight_orientation_step": 0.01,
  "weight_orientation_smoothness": 0.1,
  "weight_joint_reference": 0.1,
  "weight_joint_step": 0.01,
  "weight_joint_smoothness": 0.1,
  "joint_step_rad": 0.15,
  "validation_scope": "nodes_only",
  "ik": {
    "policy": "initialize_then_lock",
    "start_joints_rad": null,
    "branch_switch_during_optimization": false,
    "seed_policy": "previous_accepted_then_reference",
    "reference_collision_free_required": false,
    "joint_scale_deg": [10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
    "jacobian_length_scale_m": 0.5,
    "min_scaled_jacobian_singular_value": 0.001,
    "max_local_joint_step_rad": 0.05,
    "max_trial_backtracks": 12
  },
  "execute": false
}
```

工作段半径、非工作段间隙和接触数值容差通过单独的 `ContactPolicy` 配置。`working_segment_radius_mm` 是用户可调整参数，默认 1.0 mm；`contact_tolerance_mm` 为必须显式填写的可调整参数。配置入口应透传实际值并保存到请求和接触策略中，不能在碰撞实现中硬编码。指定工作段与全部已加载牙齿模型的物理最小间隙为零，数值接触判定按第 6.2 节执行；非工作段按部位配置间隙，远离工作点的杆身可以保留正余量。

接触策略中的范围字段示意如下，同样是拟新增格式：

```json
{
  "allowed_tool_region": "work_point_sphere",
  "working_segment_radius_mm": 1.0,
  "working_segment_face_rule": "all_vertices_inside",
  "allowed_dental_surface_scope": "all_loaded_dental_meshes",
  "working_segment_clearance_mm": 0.0,
  "contact_tolerance_mm": 0.01,
  "allow_penetration": false
}
```

将 `working_segment_radius_mm` 改为其他有限正数，即重新按该半径生成工作段；`working_segment_face_rule` 在首版固定为 `all_vertices_inside`，用于记录可复现的面片归类规则。半径只划分工具工作段，不是牙面上的接触邻域半径，不缩小整个牙齿模型的接触许可范围。改变半径不会改变指定工作点的固定位置约束。

`contact_tolerance_mm=0.01` 是未校准的数值示例，不是已验证默认值；实际值需结合网格分辨率、距离查询误差和已知接触/穿透算例确定。`allow_penetration=false` 表示不设置有意穿透的物理目标，不表示数值验收绝不接受任何负间隙；容差内接受的小负值必须如实记录。缺少或错误填写容差时返回 `invalid_contact_tolerance`，不静默使用该示例或自动放宽阈值。

权重、雅可比特征长度、奇异阈值和步长均为原型参数示例，需在本机器人上校准，不能解释为已验证默认值。`max_local_joint_step_rad` 控制数值 IK 的内部迭代步，`joint_step_rad` 则限制相邻空间节点的关节差，两者不能混用。`joint_scale_deg` 只做目标归一化，不构成参考关节硬邻域。

固定分支、参考序列不更新、参考初值允许碰撞是本方案的语义，不是建议在一次失败后自动关闭的选项。配置中保留这些字段用于记录和复现，首版只接受与本方案一致的值。

节点间距先做计算规模比较；加密节点是重新配置并重新求解同一个节点问题，不解释为新增连续路径验收。

## 13. 输出和验收结论

建议首版节点产物：

```text
request.json                  固定输入、参考角度和优化配置
perception_reference.json     输入线快照与来源指纹
orientation_reference.json    R_ref、代表点、重力及角度定义
ik_reference.npz              s_i、q_i_ref、初始化角度展开信息
ik_reference.json             mu、标签版本、起始配置、限位和雅可比尺度
contact_policy.json           工作段半径/划分规则、接触容差、工具指纹及整牙齿模型接触许可
tool_contact_regions.npz      原始工作/非工作面片索引及 TCP 局部工作点
pose_nodes.npz                s_i、p_i、delta_i、R_i、T_i
joint_nodes.npz               s_i、q_i、FK 误差、分支及非奇异诊断
optimization_report.json      目标各项、收敛、失败节点与耗时
node_audit.json               逐节点姿态、工具碰撞等验收
node_audit.csv                可逐节点定位的诊断
planning_report.json          能力和验收范围汇总
```

姿态节点、关节节点和参考序列共享同一组 `s_i`、顺序与输入指纹，并保存机器人模型指纹、关节名称及单位。最终独立验收检查保存的 `q_i`，不重新挑选更容易通过检查的 IK 解。

逐节点验收包括：固定目标位置、参考姿态边界、锁定分支、实际角度展开、关节限位、FK、非奇异性、已启用的相邻关节步长，以及工具接触和整机碰撞。目标 `T_i` 和实际 `FK(q_i)` 对应的工具位姿都需要符合接触规则；FK 容差不能代替接触容差或用来允许非工作端穿透。

`node_audit.json/csv` 保存实际 `contact_tolerance_mm`、各节点工作段最小原始有符号间隙 `working_min_signed_gap_mm`、最大几何负间隙 `working_max_negative_gap_mm=max(0, -working_min_signed_gap_mm)`、超出容差的最大违反量 `working_max_violation_mm=max(0, working_max_negative_gap_mm-contact_tolerance_mm)`，以及接触分类、最差几何对/面片和查询有效性。目标位姿与 FK 位姿分别记录；无法验证时标记无效，不用零值伪装通过。容差内的负间隙同样保留，报告结论使用“工作段在所配容差内通过”，不称为精确零穿透。

建议报告至少包含：

```json
{
  "formulation": "dense_node_orientation",
  "validation_scope": "nodes_only",
  "ik_policy": "initialize_then_lock",
  "ik_reference_initialized": true,
  "ik_branch_locked": true,
  "tool_nodes_verified": true,
  "robot_nodes_verified": true,
  "joint_node_sequence_verified": true,
  "continuous_path_verified": false,
  "execution_ready": false
}
```

这里是假设“参考初始化、锁定分支与全部工具/整机节点均通过”的报告示例，不是当前已完成的运行结果。实际布尔值必须来自对应检查，不应因为优化器返回成功就写为真。`joint_node_sequence_verified` 仅表示保存的离散关节序列满足本轮节点和已配置的节点对规则，不代表任何关节插值已经验收。

优化报告另行保存六类目标项、最大参考姿态偏差、最大参考关节偏差、关节一/二阶差分统计、最小归一化雅可比奇异值、最大约束违反量，以及参考初始化/局部 IK/敏感度/碰撞/外层求解/独立验收耗时。记录数值 IK 回退次数、无效试探节点和拒绝次数，用于分辨计算瓶颈与局部可行域问题。

即使工具和整机节点均通过，本轮仍写 `validation_scope="nodes_only"`，不将其改称为连续轨迹无碰撞。当前讨论阶段也不导出一个冒充已验证执行轨迹的 `trajectory.npz`。

## 14. 按当前边界安排实施

| 阶段 | 内容 | 可交付结果 |
| --- | --- | --- |
| A | 原线只读采样、R_ref、每节点三参数 | 固定节点及可解释的姿态表示 |
| B | 稳定分支标签、参考顺序跟踪、角度展开 | 固定 q_i_ref、mu 和初始化诊断 |
| C | 按可配置半径划分工具工作段、节点工具/整机接触分类 | 工作/非工作面片分组及节点碰撞结果 |
| D | 固定分支局部 IK、敏感度和六类目标项 | 3N 变量、包含关节成本的节点问题 |
| E | 稀疏外层、无效 IK 试探拒绝、接受点缓存 | 完整的固定分支节点优化原型 |
| F | 同一批节点独立验收、结果保存和可视化 | 节点姿态、关节序列和诊断报告 |
| G | 接入所需的进出场与时间分配 | 范围明确的运动产物 |

A 到 F 构成本轮设计的节点优化流程；G 是后续运动产物阶段。不会把节点原型描述为已完成可执行的整机轨迹规划。

本轮必要验证集中在：

- 源线不变、节点位置固定、旋转向量和参考边界正确、非均匀节点差分权重正确。
- 解析候选顺序变化不会改变标签；原始角度跨越正负 pi 时展开连续，且不越过实际关节限位。
- 初始化只运行一次；参考姿态存在碰撞但 IK 正规可达时可以进入优化；参考跟踪失败有明确节点和原因。
- 优化时每个节点只接受锁定分支，其他分支有解也不能掩盖当前分支失败；参考目标不随接受点更新。
- 被拒绝的试探和梯度差分不更新 warm-start 缓存；同一 D 在不同回调顺序及种子下给出一致结果。
- 关节敏感度与小幅同分支局部差分一致；接近奇异点或关节边界时试探被拒绝并可回退。
- 工作段默认半径为 1.0 mm，调整参数可重建分组；单位换算和 TCP 局部坐标正确，球面上的顶点按包含处理，边界混合面片归非工作段，两组互斥且覆盖全部工具面片，其他部件不因位于球内而被放行。
- 指定工作段与目标牙其他区域、邻牙和对颌的接触都允许；超出 `contact_tolerance_mm` 的工作段穿透及非工作段的非法接触仍被拒绝，整机检查使用保存的实际关节节点。
- 接触容差必填且为有限非负数，正确换算毫米与米；正负容差边界包含在接触带内，超出负边界拒绝，零容差拒绝负间隙；查询无效或符号不明不因接近零而放行。优化和独立验收使用相同阈值，不叠加其他容差，不截断原始负间隙，不自动增大容差。
- 节点产物保存加载一致，IK/碰撞失败不被优化器成功标志覆盖，报告不扩大验证范围。

不再把“发现两个节点之间的碰撞”或“自动加密并修复区间”列为本轮通过条件，也不为其编写本轮测试。

## 15. 与上一版相比的决定

| 项目 | 当前决定 |
| --- | --- |
| 连续优化变量 | 从每节点六关节角改为每节点三姿态参数 |
| 位置与源线 | 固定，直接使用输入 |
| 工作段几何定义 | 工作点为球心，半径 working_segment_radius_mm 默认 1.0 mm，可调整；全部顶点在球内的工具三角面属于工作段 |
| 工作段接触范围 | 全部已加载上下颌牙齿模型，物理目标不穿透，数值验收允许容差带 |
| 数值接触容差 | contact_tolerance_mm 必须显式配置，为有限非负数；工作段原始间隙不小于其负值才可通过 |
| 几何表示 | 密集的目标位姿节点序列 |
| 平滑性 | 姿态和关节节点各自的一阶变化、二阶差分 |
| 关节参考目标 | 跟踪初始化后固定的 q_i_ref |
| 约束范围 | 只在固定节点评价 |
| 节点之间的检查 | 本轮不考虑 |
| 区间加密/关节样条优化 | 移出本轮方案 |
| IK 初始化 | 统一 R_ref 的整段顺序跟踪，成功后锁定 q_i_ref 和 mu |
| IK 优化评价 | 同节点 warm-start 的固定分支局部逆映射 |
| 分支保持 | 显式稳定标签、角度展开、限位和非奇异条件 |
| IK 失败恢复 | 缩小试探步；不自动换支、移线或放宽姿态界 |
| 通过结论 | 离散节点通过，不声明连续轨迹通过 |

## 16. 相关现有入口

`planning/simple.py` 提供区域代表点；`planning/pose_search/constraints.py` 提供参考朝向构造；`perception/regions/continuous.py` 提供原线求值和只读采样；`planning/infrastructure/robot.py` 提供无标签 IK 候选、FK 和世界坐标 TCP 雅可比；`planning/infrastructure/collision.py` 提供现有原始网格基础设施。固定分支标签、局部逆映射和接触适配器均是拟新增能力。

现有优化器候选能力可查 [SciPy trust-constr](https://docs.scipy.org/doc/scipy/reference/optimize.minimize-trustconstr.html) 和 [NonlinearConstraint](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.NonlinearConstraint.html)。固定分支有效域、接触距离和试探步拒绝仍需要本方案自己的适配；通用连续求解器不自动提供这些语义。
