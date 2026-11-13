"""口腔与牙齿的统一参数文件：修改数值并保存，重新启动预览或仿真后生效。

运行 python -m pipeline.oral 统一预览；按 S 或使用 --save 保存当前两组参数。
完整流程命令行参数只覆盖本次运行；预览使用 --save 才会写回文件。
尺寸和坐标单位为毫米，角度单位为度。
"""

# 白色椭圆口缘 / 面罩
# enabled：是否显示面罩；OBS-WA 优化也据此启用/关闭口缘入口约束。
# width_mm：净开口全宽，范围 10～110 mm。
# height_mm：净开口全高，范围 10～130 mm。
# center_mm：[横向 X，上下 Y，前后 Z]，均为牙模口扫坐标。
# distance_mm：从牙模最前端到开口平面的 +Z 方向间距，必须 >= 0。
#   设置数字时，自动计算前后 Z，center_mm 的 X/Y 仍生效。
#   改为 None 时，直接使用 center_mm 的 Z 定位。
MOUTH = {
    'enabled': False,
    'width_mm': 70.0,
    'height_mm': 45.0,
    'center_mm': [0.0, 10.0, 45.0],
    'distance_mm': 14.554636197417643,
}

# 上下颌牙齿开合（上颌固定，下颌旋转）
# opening_deg：范围 -10～40°；0 为原始姿态，正值张大、负值收小。
# hinge_center_mm：旋转横轴中心的口扫坐标，是可调仿真近似。
JAW = {
    'opening_deg': 0.0,
    'hinge_center_mm': [0.0, 10.0, -40.0],
}
