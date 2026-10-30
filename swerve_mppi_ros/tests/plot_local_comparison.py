"""Compare local numerical recordings without treating them as Gazebo results."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def render(main, before, after, output):
    roots = [Path(p) for p in (main, before, after)]
    groups = [{r['scenario']: r for r in csv.DictReader((root/'metrics.csv').open())}
              for root in roots]
    names = list(groups[2])
    labels = ['main 35b082b', 'PR before optimization', 'Current optimization']
    colors = ['#2563eb', '#e58a2b', '#0d9488']
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    x = np.arange(len(names))
    for ax, field, factor, title in [
            (axes[0, 0], 'path_peak_m', 100, 'Peak XY deviation [cm]'),
            (axes[0, 1], 'duration_s', 1, 'Simulation duration [s]')]:
        for index, (group, label, color) in enumerate(zip(groups, labels, colors)):
            bars = ax.bar(x+(index-1)*.25, [factor*float(group[n][field]) for n in names],
                          width=.24, color=color, label=label)
            for bar, name in zip(bars, names):
                if group[name]['status'] != 'passed':
                    bar.set_hatch('////')
                    bar.set_edgecolor('#334155')
        if field == 'path_peak_m':
            ax.axhline(10, ls='--', lw=1, color='#b91c1c', label='10 cm functional gate')
        ax.set_title(title, loc='left', weight='bold')
        ax.set_xticks(x, names, rotation=25, ha='right')
        ax.grid(axis='y', alpha=.2)
        ax.legend(fontsize=8)
    for root, label, color in zip(roots, labels, colors):
        data = np.atleast_1d(np.genfromtxt(root/'crab_right.csv', delimiter=',', names=True))
        early = data['t'] <= 8
        axes[1, 0].plot(data['t'][early], 100*data['path_error'][early], color=color, label=label)
        data = np.atleast_1d(np.genfromtxt(root/'figure_eight.csv', delimiter=',', names=True))
        last = data['t'] >= data['t'][-1]-8
        request = np.hypot(data['request_vx'], data['request_vy'])
        axes[1, 1].plot(data['t'][last]-data['t'][-1], request[last], color=color, label=label)
    axes[1, 0].axhline(10, ls='--', lw=1, color='#b91c1c')
    axes[1, 0].set(title='crab_right: startup position error', xlabel='Simulation time [s]',
                   ylabel='XY deviation [cm]')
    axes[1, 1].set(title='figure_eight: final translation request',
                   xlabel='Time relative to recording end [s]', ylabel='Requested speed [m/s]')
    for ax in axes[1]:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    counts = [sum(g[n]['status'] == 'passed' for n in names) for g in groups]
    fig.suptitle('Local numerical comparison | functional passes: '+
                 ' / '.join(f'{label}: {count}/{len(names)}' for label, count in zip(labels, counts))+
                 '\nSame XY inputs, routes >=12 m, seed 42, 1 m/s, independent 0.12 s servo',
                 fontsize=15, weight='bold', y=.985)
    fig.text(.5, .012, 'Hatching = failed functional case. Main follows supplied intermediate yaw; '+
             'PR/current ignore it.\nNo ROS/Gazebo validation. Strict quality checks and realtime acceptance remain separate.',
             ha='center', fontsize=10, color='#475569')
    fig.tight_layout(rect=(0, .06, 1, .92), h_pad=2.5)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=145)
    plt.close(fig)
    table = output.with_suffix('.csv')
    with table.open('w') as f:
        writer = csv.writer(f)
        writer.writerow(['scenario', 'version', 'status', 'duration_s', 'path_peak_cm',
                         'final_position_cm', 'final_yaw_rad', 'compute_p99_ms'])
        for name in names:
            for label, group in zip(labels, groups):
                row = group[name]
                writer.writerow([name,label,row['status'],row['duration_s'],
                                 100*float(row['path_peak_m']),100*float(row['final_position_m']),
                                 row['final_yaw_rad'],row['compute_p99_ms']])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ('main', 'before', 'after', 'output'):
        parser.add_argument('--'+flag, required=True)
    args = parser.parse_args()
    render(args.main, args.before, args.after, args.output)
