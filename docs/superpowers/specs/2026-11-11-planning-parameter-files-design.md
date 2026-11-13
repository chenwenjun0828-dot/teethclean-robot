# Planning 命令参数文件

用户要求 planning 命令与 pipeline 一样从 config 参数文件读取运行参数。
所有现有 planning CLI 提供可选 `--params`，继续支持原来的命令行形式。

复用 `pipeline.parameters` 的 argparse schema、重复键/未知参数检查及路径规则，
补充必填参数从文件读取、整数与自定义类型校验。优先级为默认值 < JSON < 显式 CLI；
JSON 内相对路径与参数文件路径相对于项目根目录，原有 CLI 路径语义不变。
参数文件明确指定的区域或牙颌仍必须与 PreparedRegion 一致，不能被自动推断覆盖。

覆盖 planning、pose_angles、view、preview、sweep_preview、segment_map、retime、execute，
包括指向这些实现的旧兼容入口。在 config 提供独立参数示例，两个同参数诊断图命令共用 preview 文件。
常规布尔开关支持负向 CLI 覆盖；已有 no_output/no_browser 分别用 --save-output/--browser 反向覆盖。

示例采用当前前牙区参数，依赖先前感知输出，并指定独立 planning 输出目录。
执行示例默认沿用保存时序；只有显式关闭 original_timing 才按 duration 重计时。
验证覆盖参数合并、错误输入、跨来源互斥、必填参数、区域一致性、各入口转发、真实 dry-run 和导出。
