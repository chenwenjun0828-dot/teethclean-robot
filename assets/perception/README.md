# Perception Results

## 正式规划输入入口

本批上下颌六区域读取 [sessions/20261113_220209/README.md](sessions/20261113_220209/README.md)，
清单为 `sessions/20261113_220209/planning_input_manifest.json`。
`outputs/` 仅为历史采集与实验导出，不是本批规划读取入口。

```text
assets/perception/
├── mandibular/          下颌口扫局部牙龈提取源数据
├── maxillary/           上颌口扫局部牙龈提取源数据
├── tooth_segmentation/  两颌分割预测源数据
└── sessions/
    └── 20261113_220209/ 本批基座坐标规划输入及位姿记录
```

源数据保持口扫局部坐标；session 中的位置点已变换到机器人 Base，按 12 个区域保存。
两者是不同处理阶段，不应互相覆盖。本次不再单独使用顶层 `assets/perception_sessions/`。
固定手眼与设备内部外参另存于 `assets/calibration/ur7e_d405/20261113/`。

## Source reference outputs

Checked-in gingival-line OBJ/CSV files are reference perception outputs for the
mandibular and maxillary scans. The CrossTooth prediction snapshots used by the
default offline pipeline live under `tooth_segmentation/` so a fresh run does
not depend on generated files in `outputs/`. Provenance JSON files record source
and topology metadata; new run artifacts belong in a pipeline output directory
unless the `--no-output` policy is selected.
