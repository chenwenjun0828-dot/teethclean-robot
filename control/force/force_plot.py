"""Offline force-run plots, independent of the servo and live display."""
import numpy as np


def plot(output, rows, config):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = np.array([r['time_s'] for r in rows])
    fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    ax[0].plot(t, [r['compensated_force_N'] for r in rows], lw=.8, label='Unfiltered compensated force')
    ax[0].plot(t, [r['applied_command_force_N'] for r in rows], lw=.5, alpha=.6, label='After command application')
    ax[0].plot(t, [r['force_target_N'] for r in rows], '--', lw=.8, label='Force target')
    ax[0].axhline(config.predictive_force_N, ls=':', c='orange', label='Predictive guard')
    ax[0].axhline(3., ls='--', c='red', label='Maximum 3 N')
    ax[0].set_ylabel('Contact force (N)'); ax[0].legend(loc='upper right', fontsize=8)
    ax[1].plot(t, [r['source_time_s'] for r in rows], label='Source path progress')
    ax[1].set_ylabel('Source time (s)'); ax[1].legend()
    for name in ('Fx_N', 'Fy_N', 'Fz_N'):
        ax[2].plot(t, [r[name] for r in rows], lw=.6, label=name)
    ax[2].set_ylabel('Raw sensor (N)'); ax[2].set_xlabel('Executed simulation time (s)'); ax[2].legend()
    for axis in ax: axis.grid(alpha=.2)
    fig.suptitle('Full retained-path attempt | collisions ON | fixed jaws | convex-hull contact model')
    fig.tight_layout(); fig.savefig(output/'force_tracking.png', dpi=160); plt.close(fig)
