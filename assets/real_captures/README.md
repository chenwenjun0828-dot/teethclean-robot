# 真实相机点云样例

`dental_phantom_legacy.ply` 是原桌面 `4.ply` 的原样副本，名称不包含未经确认的拍摄日期。

- SHA-256：`7dbabe592306c72b19577e2873b43661eca36da49229a0dd7c9cb420d3ef90b8`
- 原始拍摄时间、设备序列号和标定信息未随该 PLY 提供。
- 原项目 `teethclean_robot/perception/registration/manual_real.py` 将其解释为米制、OpenGL 相机坐标，并裁剪下颌。
- 这些单位和坐标轴是历史实验假设，尚未独立标定；文件本身未缩放或翻转。
- 历史下颌 ROI（转换到 CV 相机毫米后）：最小 `[-50,25,100]`，最大 `[50,70,450]`。

在项目根目录运行 `python -m perception.registration.manual_real --source sample --view`。
更换实物必须同时更换匹配的口扫，不能假设仓库口扫适合任意牙模。

详细接口见 [人工配准入口说明](../../perception/registration/MANUAL_REAL.md)。
