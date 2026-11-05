# 自动牙齿点云配准设计

用户已确认继续开发，并要求所有对外变换统一为口扫模型到相机。
依据用户提供的《口腔局部点云与口扫模型自动配准算法文档.md》实现第一版。

## 算法与接口

新增 `perception/registration/automatic.py`，使用 Open3D 完成统计离群点过滤、
体素下采样、法向估计、FPFH 特征、互相匹配的 RANSAC 和 Point-to-Plane ICP。
默认体素 0.5 mm，法向半径为体素的 2 倍，FPFH 半径为 5 倍，
RANSAC 距离阈值为 1.5 倍，100000 次迭代、0.999 置信度，
ICP 距离阈值为体素大小，50 次迭代。
ICP 使用比特征点云更细的点云，默认体素为粗配准体素的一半。

`register_clouds_automatic(scan_cloud, camera_cloud, options=None)` 返回
`(T_camera_scan, metrics)`。内部以局部相机点云为 source、完整口扫为 target，
粗配准和精配准变换在返回前取逆；报告中的初始变换同样为口扫到相机。
Fitness 和 RMSE 的统计方向仍为局部相机点云到完整口扫，并在报告中明确标注。
算法不读取标志点、采集的仿真真值或机器人位姿，不修改输入点云。

## 项目接入

保留人工配准方法，默认方法继续为 manual。配置、采集配准接口和 pipeline
增加 automatic 方法选择；自动分支不加载或调用点选窗口。
自动方法和人工标志点输入同时出现时明确报错。
自动结果沿用 `T_camera_scan_initial/estimated.npy/.txt`、
`scan_registered_camera_mm.ply`、牙龈线坐标文件和 `registration_report.json`。
自动报告标记方法及零个人工标志点，不创建伪造的人工对应点文件。

新增 `registration/main.py` 独立点云入口，可读取 PLY/PCD 口扫和相机点云，
保存相同方向的变换、已变换口扫、报告、静态叠加预览，并可打开三维窗口。
参数放在配置中的 `automatic_registration` 对象中，也支持独立运行时覆盖。

## 错误与验证

拒绝非有限坐标、空点云、退化点云、无效参数、预处理后点数不足，
以及 RANSAC/ICP 未建立足够对应点的结果；异常沿用 pipeline 的停止及失败报告。
第一版不实现牙齿区域分割、多尺度 ICP 或复杂配准置信度判定。
Fitness 高和 RMSE 低本身不能保证重复牙齿结构的定位正确。

验证包括已知大角度刚体变换的部分重叠合成点云、坐标方向及输入不变性、
离群点及无效输入、自动分支不选点、保存格式与牙龈线读取、CLI、
算法失败不进入轨迹执行、现有全项目回归测试。
另使用已有牙模和采集点云运行自动方法，仿真真值只用于运行后的误差评价。
