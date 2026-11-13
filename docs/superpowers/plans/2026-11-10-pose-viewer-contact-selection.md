# 牙面接触点选择实施计划

设计：`docs/superpowers/specs/2026-11-10-pose-viewer-contact-selection-design.md`，已获用户确认。

1. 提取接触参考基与坐标换算至 `contact_math.mjs`，保持现有角度计算接口和默认姿态。
2. 增加选点开关、恢复命令和口扫坐标显示；拾取仅作用于可见牙模，区分点击与拖动。
3. 将工具、辅助图形和对应标注变换到新接触参考系，处理模型范围切换与加载失败。
4. 验证接触数学与默认姿态兼容性，并用浏览器检查桌面、手机选点及取景。
5. 更新根 README 与查看器说明，说明局部网格法线及仅预览的边界。

本地没有可用的 writing-plans 技能，沿用项目已有计划目录记录实施步骤。

## 完成记录

- 接触参考基、口扫坐标换算、页面选点与模型范围切换已实现，默认姿态与角度记忆保留。
- `python -m unittest planning.test_pose_angles planning.test_view -q`：24 项通过。
- Playwright 已验证桌面与手机选点、拖动及背景不误选、角度保持、恢复默认、范围切换、刷新及加载失败重试；无页面脚本错误。
- 三种范围的截图、canvas 像素与布局检查通过；结果保存在 `tmp/pose_contact_checks/`。
- 根 README 与查看器 README 已补充接触点操作和坐标参考边界。
