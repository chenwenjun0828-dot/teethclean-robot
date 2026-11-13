# 下颌开合实现验证

日期：2026-11-05。

## 已完成

- 新增 config/jaw_opening.json、pipeline/jaw.py、pipeline/preview_jaw.py。
- 场景加载、完整流程参数和报告已接入下颌角度；现有口缘位置按更新后牙模重新计算。
- 配置默认仍为 0°，不自动保存验证中使用的 5° 或 15°。
- 保留工作区原有口缘相关改动；本次修改文档、场景加载、完整流程，并在 mouth.py 中增加位置刷新能力。

## 自动检查

使用项目 .venv/Scripts/python.exe 运行 unittest discover，共通过 32 项：

| 测试文件 | 数量 |
| --- | ---: |
| test_jaw_opening.py | 6 |
| test_mouth_opening.py | 5 |
| test_perception_frames.py | 5 |
| test_planning_control.py | 4 |
| test_global_rgbd.py | 3 |
| test_tcp_tooth_alignment.py | 9 |

新测试覆盖基准恢复、往返绝对角度、上颌与机器人状态不变、真实移动静态几何的接触检测、逐颌相机变换、实际网格到口缘平面距离、配置与非法输入，以及完整流程传递同一更新后场景和写入角度报告。

另检查预览 41° 和完整流程 NaN 命令行输入均被拒绝。git diff --check 通过。

## 真实渲染和完整流程

- outputs/jaw_opening/angle_0.png 与 angle_15.png 均已生成并检查：上颌保持固定，15° 下颌向下张开。
- 下颌 5°：真实提取、RGB-D 采集、自动配准及完整几何模式轨迹执行成功，70 个目标空间采样点，ICP RMSE 0.1542 mm，轨迹位置 RMS 0.0143 mm，位置最大误差 0.205 mm，姿态最大误差 0.379°。报告：outputs/jaw_opening/pipeline_5_validation/pipeline_report.json。
- 下颌 15°：真实采集 59150 个有效点，自动配准 ICP RMSE 0.1381 mm；规划在路径采样索引 25 处被现有 IK 检查拒绝，位置残差 17.53 mm、姿态残差 6.100°。未绕过可达性检查。报告：outputs/jaw_opening/pipeline_15_validation/pipeline_report.json。

几何模式轨迹执行不构成严格牙模接触碰撞验证。独立测试已验证开合更新后碰撞几何随姿态移动；本次未宣称所有角度、所有路径均可达或无接触。

## 使用入口

预览：python -m pipeline.preview_jaw --opening-deg 15。

完整流程已验证示例：python -m pipeline.run --jaw mandibular --registration-method automatic --jaw-opening-deg 5。

预览支持 J/L 调节、R 复位、S 保存、M 显隐口缘、C 恢复正视角；也支持 --save、--no-mouth 和 --render。实际交互按键由预览程序实现，本轮通过无窗口真实渲染验证显示，未自动驱动桌面窗口按键。
