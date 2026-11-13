# 离线基座坐标转换

本目录保存可提交 Git 的计算源码，不依赖 `outputs/` 中的 Python 脚本。
无需 ROS 或 MuJoCo；需要项目感知依赖（NumPy、SciPy、Open3D）。

1. `ur7e_fk.py`：从六个度制关节角计算 UR7e 标称正运动学，区分控制器 Base 与 ROS base_link。
2. `convert_points.py`：组合拍摄位姿、手眼、D405 内部外参和口扫配准，导出位置点和矩阵。
3. `export_regions.py`：将已准备的六区域转成显式 controller_base 的位置、法向、切向。

```powershell
python -m perception.coordinate_conversion.ur7e_fk --joint-observation <关节记录.json> --out <新的FK输出目录>

python -m perception.coordinate_conversion.convert_points `
  --mandibular-run <下颌采集配准目录> `
  --maxillary-run <上颌采集配准目录> `
  --fk-report <fk_report.json> `
  --handeye-inventory <handeye_inventory.json> --handeye-label 25 `
  --camera-extrinsics <camera_extrinsics.json> --out <新的转换目录>

python -m perception.coordinate_conversion.export_regions --conversion-dir <含regions的转换目录>
```

转换支持仅指定一颌；两颌一起指定时，必须属于同一原始采集和机器人拍摄位姿。
输出矩阵平移和点坐标均为米，`nominal` 表示未加入实机出厂运动学修正。
`convert_points` 只转换完整点集，不会自动生成区域；中间仍需运行 `perception.regions` 准备区域。
`export_regions` 读取对应颌的 `regions/`，并核对区域保存的位姿与外部矩阵一致。
上颌如因 SDK 浮点旋转误差不能通过区域刚体校验，应使用经正交化并记录来源的 `_rigid_m.npy`；不要修改平移或放宽区域检查。

CSV 是规划几何输入，不是已经确定 TCP 姿态、时间和碰撞可行性的真机轨迹。
