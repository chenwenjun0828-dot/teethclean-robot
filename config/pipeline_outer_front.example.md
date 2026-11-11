# 六区域参数示例

入口：`config/pipeline_outer_front.json`。

```powershell
.\.venv\Scripts\python.exe -m pipeline --params config/pipeline_outer_front.json --dry-run
.\.venv\Scripts\python.exe -m pipeline --params config/pipeline_outer_front.json --output outputs/pipeline/my_front
```

| 参数 | 示例值 | 含义 |
| --- | --- | --- |
| `jaw` | `mandibular` | 当前附带六区域数据的下颌 |
| `region` | `outer_front` | 本次只执行一个区域 |
| `planning_mode` | `simple` | 整区固定姿态，允许有界小角度修正 |
| `pose_angles_deg` | `[6.37, 13.26, -0.4]` | 固定 Z 角、Y 方位角、工具 X 翻转角 |
| `jaw_opening_deg` | 20 | 下颌张口角，单位度 |
| `sweep_trim_mm` | 1 | 仅整区两端各裁剪 1 mm |
| `pose_spacing_mm` | 0.25 | 姿态采样间距上限，曲率处会加密 |
| `offset_mm` | 1.8 | 非接触 TCP 偏移，单位 mm |
| `speed_mm_s` | 1 | 清扫速度上限，不是保证的恒定速度 |
| `check_mode` | `full` | 碰撞、轨迹审计和动力学验收 |
| `duration` | 未设置 | 由关节、TCP、角速度和加速度约束分配时间 |

当前 `outer_front` 源曲线约 54.587 mm，保留约 52.587 mm，即 96.34%。
这是该区域的几何覆盖率，不代表全口或实际接触覆盖率。相邻牙齿之间不裁断、不补线。

扫描、六区域数据和分割预测必须匹配；未附上颌六区域数据，缺少时直接报错。
`--dry-run` 校验输入和配置，不验证可达性或执行效果；每次完整运行仍需通过全部审计。
实际端到端验证状态见 `docs/integration/2026-11-11-six-region-cleanup.md`。

此参数已通过本次自动配准、规划、碰撞和动力学验收，总时长约 66.68 秒。
未指定 `duration` 时使用 planning 默认关节与角速度约束；显式指定总时长仍须重新验收。
