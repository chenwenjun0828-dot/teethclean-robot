# 力控入口与安装复现修复

日期：2026-11-11。范围为当前六区域版本的力控默认入口，以及完整仿真流程的环境安装。

## 力控入口

- `python -m control.force` 默认执行当前连续区域轨迹，必须传入 `--source` 和 `--out`。
- 显式 `--mode trajectory` 保留；旧 `--mode probe` 和 `--sweep-mm` 在参数解析时明确拒绝。
- 删除旧分段格式的探针选点、短移搜索、调度、绘图与独立 CLI，移除失效导入和历史默认输出目录。
- `contact_experiment.py` 保留当前力控复用的碰撞检查、固定牙模检查、CSV 输出，以及停止保护测试使用的独立探针原语。
- 所有力控、碰撞和原始扫描牙面接触验收条件继续生效。

默认入口实际运行命令：

```powershell
python -m control.force `
  --source outputs/integration_1111/fresh_environment_verified/tracking `
  --out outputs/integration_1111/force_default_fresh_environment_verified
```

使用新建验证环境执行，无旧探针导入异常。报告正常落盘；在 7.141 秒由
`Predicted contact force reaches 2.5 N guard` 停止，退出码为 2，
`force_control_passed=false`。预测被拒绝的步骤没有执行。
这证明入口与保护流程可用，不代表牙面恒力控制已经通过。

另设 0.1 秒时限的启动检查也正常停止、保存报告，并明确记录时限耗尽和路径未完成。

## 环境依赖

`requirements-perception-lock.txt` 补入 `python-fcl`、`toppra`、`ecos`、`scipy`、
`Cython` 和 `setuptools`，并将 Matplotlib、Plotly 的锁定版本对齐到已验证环境。
锁文件包含当前完整流程的 84 项直接及间接依赖。

验证环境为 Windows x64、CPython 3.11.9，新建于系统临时目录，未复用项目 `.venv` 的包。
使用锁文件安装成功，`pip check` 返回 `No broken requirements found`。

[TOPPRA 0.6.3](https://pypi.org/project/toppra/0.6.3/) 只有源码发行包。
初次安装检查使用了本机缓存的编译包，随后又以 `--no-cache-dir --force-reinstall --no-deps toppra==0.6.3`
重新下载源码、构建并安装成功；本机 C++ 工具链为 Visual Studio Build Tools 2022，版本 17.14.37827.9。
README 已加入构建工具、Python 环境、锁文件安装、依赖检查和可选相机/动画依赖说明。

## 验收结果

- 新环境 `python -m unittest discover -s tests -p 'test_*.py' -q`：225 项通过。
- 新环境 `python -m unittest discover -s planning -t . -p 'test_*.py' -q`：102 项通过。
- TOPPRA 无缓存重建后，`python -m unittest planning.test_fixed_path -q`：7 项通过。
- README 中的 `pipeline.run --params config/pipeline_outer_front.json --dry-run`：通过。
- 新环境完整主流程 `pipeline.run --params config/pipeline_outer_front.json --output outputs/integration_1111/fresh_environment_verified --headless`：完成。

主流程使用自动配准，RMSE 0.1485 mm；位置控制仿真轨迹时长 66.6769 秒，
最大位置误差 0.08728 mm，最小实际间隙 0.15670 mm。
结果位于 `outputs/integration_1111/fresh_environment_verified/`。

上述验证覆盖附带的下颌 `outer_front` 输入和无窗口仿真；未据此扩大到全口、真实相机或真实机器人验收。
