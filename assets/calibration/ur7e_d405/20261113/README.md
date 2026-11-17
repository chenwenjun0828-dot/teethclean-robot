# UR7e / D405 标定记录

这是一份有日期的固定标定归档，不是所有拍摄姿态通用的相机到基座矩阵。

- `handeye_25.yaml`：原始声明为 `wrist_3_link` ← `camera_color_frame` 的手眼记录，原样保留。
- `handeye_inventory.json`：两份原始手眼记录的来源、矩阵和对比；本批采用标签 `25`。
- `camera_extrinsics.json`：D405 序列号 `335122273144` 的深度/彩色 SDK 内部外参，以及按标准 ROS 约定推导的帧转换。
- `provenance.json`：从本次实验交接包归档的来源说明。原始来源路径只用于追溯，不是规划加载依赖。

矩阵平移单位米。手眼相机安装发生变化后需重新标定；本归档不包含工具 TCP 标定。
相机到基座的关系还需要对应拍摄时的末端位姿，保存在 `assets/perception/sessions/20261113_220209/` 中。

当前 session 按用户确认使用 `color_optical` 解释：同一组手眼数值直接作为彩色光学帧到末端的矩阵，不额外加入 ROS 轴转换。有效记录在 session 的 `handeye_frame_interpretation.json`。原始 YAML、inventory、SDK 数值均未改写；原标定现场 TF 尚未独立核实。
