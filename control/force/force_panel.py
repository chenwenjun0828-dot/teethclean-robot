"""Read-only dashboard INSIDE the right side of the MuJoCo window.

The viewer owns cloned model/data so its UI cannot alter the executing simulation.
Rendering is throttled independently of the 1 kHz controller.
"""
import copy
import time
from functools import lru_cache
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

def _copy_display_state(destination, model, source):
    if hasattr(mujoco, 'mj_copyData'):
        mujoco.mj_copyData(destination, model, source)
    else:
        flags = int(mujoco.mjtState.mjSTATE_INTEGRATION)
        state = np.empty(mujoco.mj_stateSize(model,flags))
        mujoco.mj_getState(model,source,state,flags)
        mujoco.mj_setState(model,destination,state,flags)
        mujoco.mj_forward(model,destination)


PHASES = {'transfer': '转移', 'acquire': '建立接触', 'trace': '沿线洁治', 'release': '卸力退离'}


@lru_cache(maxsize=4)
def _font(size):
    path = next((str(p) for p in (Path('C:/Windows/Fonts/msyh.ttc'),
        Path('C:/Windows/Fonts/simhei.ttf')) if p.exists()), None)
    return ImageFont.truetype(path, size) if path else ImageFont.load_default(size=size)


def panel_rows(row, force, path, timestep, progress, status='运行中', peak=None):
    """Prepare display values without modifying the controller or its log row."""
    def value(key, unit='', digits=3):
        v = row.get(key)
        return '--' if v is None else f'{float(v):.{digits}f}{unit}'
    phase = row.get('phase', '')
    return [
        ('运行', None),
        ('状态', status),
        ('阶段 / 牙 ID', f"{PHASES.get(phase, phase or '--')} / {row.get('tooth', '--')}"),
        ('路径段 / 进度', f"{int(row.get('segment', -1))+1} / {100*progress:.1f}%"),
        ('仿真时间', value('time_s', ' s')),
        ('力传感器', None),
        ('补偿后接触合力', value('compensated_force_N', ' N')),
        ('指令施加后合力', value('applied_command_force_N', ' N')),
        ('牙面法向分力', value('compensated_force_normal_N', ' N')),
        ('当前目标力', value('force_target_N', ' N')),
        ('控制量', getattr(force,'force_component','resultant')),
        ('未滤波历史峰值', '--' if peak is None else f'{peak:.3f} N'),
        ('原始 Fx / Fy / Fz [N]', ' / '.join(value(k) for k in ('Fx_N', 'Fy_N', 'Fz_N'))),
        ('原始 Tx / Ty / Tz [N m]', ' / '.join(value(k, digits=4) for k in ('Tx_Nm', 'Ty_Nm', 'Tz_Nm'))),
        ('控制参数 · 只读', None),
        ('设定目标 / 硬停止', f'{force.target_force_N:g} / {force.force_stop_N:g} N'),
        ('卸力 / 预测拦截', f'{path.soft_force_N:g} / {path.predictive_force_N:g} N'),
        ('虚拟质量 / 阻尼', f'{force.virtual_mass_kg:g} kg / {force.damping_Ns_m:g} N s/m'),
        ('低通时间 / 采样率', f'{force.filter_time_s*1000:g} ms / {1/timestep:.0f} Hz'),
        ('法向速度 / 行程限制', f'{force.max_normal_speed_m_s*1000:g} mm/s / +/-{force.max_displacement_m*1000:g} mm'),
        ('跟踪与接触', None),
        ('低通反馈力', value('filtered_force_N', ' N')),
        ('法向位置修正', value('normal_correction_mm', ' mm')),
        ('TCP 速度', value('actual_tcp_speed_mm_s', ' mm/s')),
        ('切向误差', value('tangent_error_mm', ' mm')),
        ('接触数 / 意外接触', f"{row.get('contact_count', '--')} / {row.get('unexpected_contact_count', '--')}"),
        ('牙模漂移', value('jaw_translation_drift_mm', ' mm', 6)),
    ]


def _chart(draw, bounds, title, samples, keys, colors, unit, window_s, thresholds=()):
    from .force_history import CHANNELS, peak_envelope
    left, top, right, bottom = bounds
    draw.rounded_rectangle(bounds, radius=8, fill='#17283c')
    draw.text((left+12, top+7), title, font=_font(17), fill='#dcecff')
    end = max(window_s, float(samples[-1, 0])) if len(samples) else window_s
    start = end-window_s
    x0, x1, y0, y1 = left+48, right-15, top+48, bottom-24
    data = [samples[:, CHANNELS.index(k)+1] for k in keys] if len(samples) else []
    values = np.concatenate(data) if data else np.array([0.])
    low = min(float(values.min()), 0.)
    high = max(float(values.max()), .01)
    if thresholds:
        high = max(high, max(v for v, _, _ in thresholds))
    pad = max((high-low)*.1, .005)
    low, high = low-pad, high+pad
    def yp(v):
        return y1-(v-low)/(high-low)*(y1-y0)
    for v in np.linspace(low, high, 4):
        y = yp(v)
        draw.line((x0, y, x1, y), fill='#30445a')
        draw.text((left+3, y-7), f'{v:.2f}', font=_font(12), fill='#91a7be')
    for value, label, color in thresholds:
        y = yp(value)
        for x in range(x0, x1, 8):
            draw.line((x, y, min(x+4, x1), y), fill=color)
        draw.text((x1-55, y-14), label, font=_font(12), fill=color)
    for key, color, vals in zip(keys, colors, data):
        xs, lows, highs = peak_envelope(samples[:, 0], vals, start, end, x1-x0+1)
        for x, lo, hi in zip(xs, lows, highs):
            draw.line((x0+int(x), yp(lo), x0+int(x), yp(hi)), fill=color, width=1)
        if len(xs)>1:
            draw.line([(x0+int(x), yp(v)) for x, v in zip(xs, highs)], fill=color, width=1)
    labels = {'compensated_force_N': '反馈', 'applied_command_force_N': '施加后', 'force_target_N': '目标',
              'Fx_N': 'Fx', 'Fy_N': 'Fy', 'Fz_N': 'Fz', 'Tx_Nm': 'Tx', 'Ty_Nm': 'Ty', 'Tz_Nm': 'Tz'}
    x = left+12
    for i, (key, color) in enumerate(zip(keys, colors)):
        label = labels[key]
        if data:
            label += f' {data[i][-1]:.3f}'
        draw.text((x, top+28), label, font=_font(13), fill=color)
        x += 160
    draw.text((x0, bottom-20), f'{start:.1f}', font=_font(12), fill='#91a7be')
    draw.text((x1-65, bottom-20), f'{end:.1f} s', font=_font(12), fill='#91a7be')
    draw.text((right-55, top+8), unit, font=_font(13), fill='#91a7be')


def render_panel(rows, width=520, height=980, alarm=False, history=None, window_s=10., thresholds=None):
    """Parameters and three rolling charts share the same right-hand column."""
    canvas = Image.new('RGB', (580, 1000), '#101c2c')
    draw = ImageDraw.Draw(canvas)
    values = dict(rows)
    def line(y, label, value):
        draw.text((22, y), label, font=_font(17), fill='#aebed0')
        shown = str(value)
        while draw.textlength(shown, font=_font(18)) > 345 and len(shown)>2:
            shown = shown[:-2]+'…'
        draw.text((558-draw.textlength(shown, font=_font(18)), y), shown, font=_font(18), fill='#edf5ff')
    draw.text((20, 14), '力传感器 / 实时监测', font=_font(25), fill='#e8f3ff')
    draw.text((20, 51), f'碰撞开启 · 牙模固定 · 最近 {window_s:g} 秒 · 参数只读', font=_font(15), fill='#93aeca')
    line(79, '状态', values.get('状态', '--'))
    line(107, values.get('阶段 / 牙 ID', '--'), '时间 '+values.get('仿真时间', '--'))
    line(135, '路径段 / 进度', values.get('路径段 / 进度', '--'))
    for i, (label, key) in enumerate([('接触合力', '补偿后接触合力'), ('当前目标', '当前目标力'), ('全程峰值', '未滤波历史峰值')]):
        x = 16+i*184
        draw.rounded_rectangle((x, 169, x+176, 237), radius=8, fill='#20334a')
        draw.text((x+12, 176), label, font=_font(15), fill='#aac6df')
        draw.text((x+12, 200), values.get(key, '--'), font=_font(23), fill='#ffac72' if alarm else '#8fe0c8')
    for y, key in zip(range(248, 381, 22), ('设定目标 / 硬停止', '卸力 / 预测拦截', '虚拟质量 / 阻尼',
        '低通时间 / 采样率', '法向速度 / 行程限制', '法向位置修正', '切向误差')):
        line(y, key, values.get(key, '--'))
    line(405, '接触数 / 意外接触', values.get('接触数 / 意外接触', '--'))
    samples = np.asarray(history) if history is not None else np.empty((0, 10))
    charts = [
        ('接触合力 / 目标', ('compensated_force_N', 'applied_command_force_N', 'force_target_N'), ('#5ab7ff', '#ffa766', '#63deb0'), 'N'),
        ('原始三轴力', ('Fx_N', 'Fy_N', 'Fz_N'), ('#5ab7ff', '#ffa766', '#63deb0'), 'N'),
        ('原始三轴力矩', ('Tx_Nm', 'Ty_Nm', 'Tz_Nm'), ('#5ab7ff', '#ffa766', '#63deb0'), 'N m'),
    ]
    for i, (title, keys, colors, unit) in enumerate(charts):
        _chart(draw, (14, 430+i*177, 566, 600+i*177), title, samples, keys, colors, unit, window_s,
               thresholds if i==0 and thresholds else ())
    draw.text((20, 966), '原始信号含重力/惯性；接触合力已做模型补偿', font=_font(15), fill='#7f99b6')
    return np.asarray(canvas.resize((max(1, int(width)), max(1, int(height))), Image.Resampling.LANCZOS)).copy()


class ForcePanel:
    def __init__(self):
        self.viewer = None
        self.last_draw = -float('inf')
        self.peak = 0.
        self.rows = []
        self.alarm = False
        self.backdrop = mujoco.MjvFigure()
        self.backdrop.figurergba[:] = (.063, .110, .173, 1.)
        self.backdrop.panergba[:] = self.backdrop.figurergba
        self.backdrop.flg_ticklabel[:] = False
        self.backdrop.gridsize[:] = 0
        from .force_history import ForceHistory
        self.history = ForceHistory()

    def set_config(self, force, path, timestep):
        self.force, self.path, self.timestep = force, path, timestep
        from .force_history import ForceHistory
        self.history = ForceHistory(sample_rate_hz=1/timestep)
        self.thresholds = [(path.soft_force_N, f'{path.soft_force_N:g} N', '#ccb55f'),
                           (path.predictive_force_N, f'{path.predictive_force_N:g} N', '#ffad62'),
                           (force.force_stop_N, f'{force.force_stop_N:g} N', '#ff757c')]

    def open(self, model, data, jaw):
        from mujoco import viewer
        self.model, self.data = copy.copy(model), mujoco.MjData(model)
        _copy_display_state(self.data, self.model, data)
        self.viewer = viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False)
        with self.viewer.lock():
            self.viewer.cam.lookat[:] = data.xpos[model.body('dental_'+jaw).id]
            self.viewer.cam.distance = .45
            self.viewer.cam.azimuth, self.viewer.cam.elevation = 100, -20

    def is_running(self):
        return self.viewer is not None and self.viewer.is_running()

    def update(self, data, row, progress, status='运行中', force_draw=False):
        self.peak = max(self.peak, row['compensated_force_N'], row['applied_command_force_N'])
        self.history.append(dict(row,
            compensated_force_N=row.get('regulated_force_N',row['compensated_force_N']),
            applied_command_force_N=row.get('applied_regulated_force_N',row['applied_command_force_N'])))
        running = self.is_running()
        now = time.monotonic()
        if not force_draw and (not running or now-self.last_draw < .05):
            return
        self.last_draw = now
        self.alarm = max(row['compensated_force_N'], row['applied_command_force_N']) >= self.path.soft_force_N or status.startswith('已停止')
        self.rows = panel_rows(row, self.force, self.path, self.timestep, progress, status, self.peak)
        if not running:
            return
        with self.viewer.lock():
            _copy_display_state(self.data, self.model, data)
        self.viewer.sync(state_only=True)
        viewport = self.viewer.viewport
        if viewport is not None and viewport.width > 0 and viewport.height > 0:
            width = min(560, max(1, int(viewport.width*.42)))
            height = viewport.height
            panel = render_panel(self.rows, width, height, self.alarm, self.history.arrays(), self.history.window_s, self.thresholds)
            rect = mujoco.MjrRect(viewport.left+viewport.width-width, viewport.bottom, width, height)
            # The native figure establishes 2D GL state before user images render.
            # drawPixels alone inherits depth testing and close geometry hides it.
            self.viewer.set_figures((rect, self.backdrop))
            self.viewer.set_images((rect, panel))

    def finish(self, data, row, report, output):
        reason = report['stop_reason']
        short_reason = ('时间上限' if 'time budget' in reason else
                        '用户关闭窗口' if 'viewer closed' in reason else
                        '碰撞保护' if 'collision' in reason.lower() else
                        '力保护' if 'force' in reason.lower() else '保护触发')
        status = ('路径完成 / 验收通过' if report['force_control_passed'] else '路径完成 / 力控未达标') if report['execution_completed'] else '已停止 / '+short_reason
        self.update(data, row, report['retained_path_progress_fraction'], status, force_draw=True)
        if self.rows:
            Image.fromarray(render_panel(self.rows, 580, 1000, self.alarm, self.history.arrays(), self.history.window_s, self.thresholds)).save(output/'force_panel.png')

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
