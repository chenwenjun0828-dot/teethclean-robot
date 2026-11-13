"""Show a reconstructed sweep and its original-source/connector accounting."""
import json
from pathlib import Path
import numpy as np
from ..motion_planning.contract import load_sweep_reference


def render(directory, output=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.ticker import MaxNLocator
    directory = Path(directory)
    output = directory if output is None else Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = json.loads((directory/'planning_report.json').read_text(encoding='utf-8'))
    path = load_sweep_reference(directory, report)
    with np.load(directory/'geometry.npz', allow_pickle=False) as saved:
        source = saved['points_scan_m']*1000
        stroke_ids = saved['stroke_id']
    origin = source.mean(0)
    _, _, vt = np.linalg.svd(source-origin, full_matrices=False)
    basis = vt[:2].T
    if basis[0, 0] < 0: basis[:, 0] *= -1
    project = lambda points: (points-origin)@basis
    fig = plt.figure(figsize=(13, 6.8), layout='constrained')
    fig.set_constrained_layout_pads(w_pad=.12, h_pad=.08, wspace=.16)
    axes = [fig.add_subplot(121, projection='3d'), fig.add_subplot(122)]
    for sid in np.unique(stroke_ids):
        line = source[stroke_ids == sid]
        axes[0].plot(*line[:, [0, 2, 1]].T, color='#c3c9cf', lw=1.2)
        axes[1].plot(*project(line).T, color='#c3c9cf', lw=1.4)
    offset = report['config']['standoff_mm']*.001
    for part in path.parts:
        grid = np.linspace(part['start'], part['end'], 160)
        line = np.array([path.evaluate(s) for s in grid])*1000
        tcp = np.array([path.position(s, offset) for s in grid])*1000
        color = '#177a82' if part['kind'] == 'source' else '#d87823'
        axes[0].plot(*tcp[:, [0, 2, 1]].T, color=color, lw=2.2)
        axes[1].plot(*project(line).T, color=color, lw=2.4)
        if part['kind'] == 'source':
            p = project(line[len(line)//2:len(line)//2+1])[0]
            axes[1].annotate(f"ID {part['tooth0']}", p, xytext=(0, 12), textcoords='offset points',
                             ha='center', fontsize=9, color='#23424a')
    ends = np.array([path.evaluate(s) for s in (0., path.end)])*1000
    axes[1].scatter(*project(ends).T, c=['#198754', '#ad2c3b'], s=50, zorder=5)
    for p, label, shift in zip(project(ends), ['START', 'END'], [(-10, -22), (12, -22)]):
        axes[1].annotate(label, p, xytext=shift, textcoords='offset points', ha='center', fontsize=9)
    axes[0].set(xlabel='Scan X (mm)', ylabel='Scan Z (mm)', zlabel='Scan Y (mm)',
                title=f'TCP path at {report["config"]["standoff_mm"]:g} mm offset')
    axes[0].set_box_aspect(np.maximum(np.ptp(source[:, [0, 2, 1]], axis=0), 1))
    for axis in (axes[0].xaxis, axes[0].yaxis, axes[0].zaxis):
        axis.set_major_locator(MaxNLocator(nbins=3))
        axis.label.set_fontsize(9)
    axes[0].tick_params(labelsize=8, pad=1)
    axes[0].view_init(elev=26, azim=-65)
    axes[1].set(xlabel='Common plane coordinate 1 (mm)', ylabel='Common plane coordinate 2 (mm)',
                title='Gingival reference: one continuous path')
    axes[1].set_aspect('equal', adjustable='datalim'); axes[1].margins(.12, .25)
    axes[1].grid(alpha=.18)
    axes[1].legend(handles=[Line2D([0], [0], color='#c3c9cf', label='Original source'),
                            Line2D([0], [0], color='#177a82', label='Retained original arc'),
                            Line2D([0], [0], color='#d87823', label='Synthetic smooth connector')],
                   loc='upper center', fontsize=9)
    jaw = json.loads((directory/'geometry.json').read_text(encoding='utf-8'))['jaw']
    source_length = report['source_region_length_mm']
    title = (f'{jaw.capitalize()} | one {report.get("region", "outer_front")} sweep | no internal retreat / re-approach\n'
             f'Original retained {report["retained_length_mm"]:.2f} / {source_length:.2f} mm '
             f'({report["coverage_fraction"]:.1%})  +  connectors {report["synthetic_bridge_length_mm"]:.2f} mm\n'
             f'Planning validated: {report.get("execution_ready", False)} | '
             f'Dynamics verified: {report.get("fast_dynamics_verified", False)} | simulation only')
    policy = report.get('sweep_options', {}).get('roll_policy')
    if policy is not None:
        axes_audit = report.get('trace_audit', {}).get('yz_axis_audit', {})
        title += (f'\nY/Z corridor: {policy["max_deviation_deg"]:g} deg; '
                  f'measured Y max: {axes_audit.get("max_y_deviation_deg", float("nan")):.2f} deg; '
                  f'roll variation: {report.get("roll_summary", {}).get("total_roll_variation_deg", float("nan")):.0f} deg')
    fig.suptitle(title, fontsize=12)
    destination = output/'continuous_sweep.png'
    fig.savefig(destination, dpi=170)
    fig.savefig(output/'continuous_sweep.svg')
    plt.close(fig)
    return destination


def main(argv=None):
    import argparse
    from pipeline.parameters import add_parameter_file, parse_parameters
    parser = argparse.ArgumentParser(description=__doc__)
    add_parameter_file(parser)
    parser.add_argument('--plan', type=Path, required=True)
    args = parse_parameters(parser, argv)
    print(render(args.plan).resolve())


if __name__ == '__main__':
    main()
