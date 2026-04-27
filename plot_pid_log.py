#!/usr/bin/env python3
"""
plot_pid_log.py — live tail-and-plot for pid_vtol.py output

Usage:
    python plot_pid_log.py [pid_log.csv]    # live mode (re-reads every 500 ms)
    python plot_pid_log.py --static         # one-shot plot of completed run

Six panels: roll (phi), pitch (theta), yaw (psi), altitude, throttles, flaps differential.
Commands shown as dashed, actuals as solid.

Deps: matplotlib (pip install matplotlib in your venv)
No pandas needed.
"""

import sys
import csv
import os
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

LOG = Path(sys.argv[-1] if sys.argv[-1].endswith('.csv') else 'pid_log.csv')
STATIC = '--static' in sys.argv
REFRESH_MS = 500


def read_csv(path):
    """Re-read the whole CSV. Cheap at 30 Hz log rate over short runs."""
    cols = {}
    if not path.exists() or path.stat().st_size == 0:
        return cols
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k, v in row.items():
                if k is None or v is None:
                    continue
                try:
                    cols.setdefault(k, []).append(float(v))
                except ValueError:
                    pass
    return cols


def setup_axes():
    fig, axes = plt.subplots(6, 1, figsize=(12, 13), sharex=True)
    fig.canvas.manager.set_window_title(f'PID log — {LOG.name}')
    fig.tight_layout(pad=2.0)
    return fig, axes


PANELS = [
    {
        'title': 'Roll  (phi, deg)',
        'series': [('phi_cmd', '--'), ('phi', '-')],
    },
    {
        'title': 'Pitch  (theta, deg)',
        'series': [('theta_cmd', '--'), ('theta', '-')],
    },
    {
        'title': 'Yaw  (psi, deg)',
        'series': [('psi_cmd', '--'), ('psi', '-')],
    },
    {
        'title': 'Altitude  (ft)',
        'series': [('alt_cmd', '--'), ('alt', '-')],
    },
    {
        'title': 'Throttles  (0–1)',
        'series': [('thrFR', '-'), ('thrFL', '-'),
                   ('thrAR', '-'), ('thrAL', '-')],
        'ylim': (-0.05, 1.05),
    },
    {
        'title': 'Flaps Differential  (yaw control)',
        'series': [('flapsR', '-'), ('flapsL', '-'), ('yaw_diff_flaps', '--')],
    },
]


def draw(axes, d):
    if not d or 't' not in d:
        return
    t = d['t']
    for ax, panel in zip(axes, PANELS):
        ax.clear()
        ax.grid(alpha=0.3)
        for col, style in panel['series']:
            if col in d:
                ax.plot(t, d[col], style, label=col, linewidth=1.2)
        ax.legend(loc='upper right', fontsize=8, ncol=2)
        ax.set_title(panel['title'], loc='left', fontsize=10)
        if 'ylim' in panel:
            ax.set_ylim(panel['ylim'])
    axes[-1].set_xlabel('time (s)')


def main():
    if not LOG.exists():
        print(f'  waiting for {LOG} to appear...')
    fig, axes = setup_axes()

    if STATIC:
        draw(axes, read_csv(LOG))
        plt.show()
        return

    def update(_frame):
        draw(axes, read_csv(LOG))

    _anim = FuncAnimation(fig, update, interval=REFRESH_MS, cache_frame_data=False)
    plt.show()


if __name__ == '__main__':
    main()
