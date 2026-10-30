"""Plot numerical closed-loop recordings, explicitly distinct from Gazebo."""
import argparse
import csv
import math
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from tracking_quality import geometric_reference


def render(root):
    root = Path(root)
    with (root/'metrics.csv').open() as f:
        metrics = list(csv.DictReader(f))
    passed = sum(row['status'] == 'passed' for row in metrics)
    settings = json.loads((root/'experiment.json').read_text()) if (root/'experiment.json').exists() else {}
    setup = (f"seed 42, minimum route {settings.get('minimum_length', 'original')} m, "
             f"speed cap {settings.get('max_speed', .8)} m/s, servo {settings.get('servo_time', .12)} s")
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9,
                         'axes.spines.top': False, 'axes.spines.right': False})
    columns = min(4, len(metrics))
    row_count = math.ceil(len(metrics)/columns)
    fig, axes = plt.subplots(row_count, columns, figsize=(4.8*columns, 4.4*row_count+2), squeeze=False)
    for index, (ax, row) in enumerate(zip(axes.flat, metrics), 1):
        name = row['scenario']
        reference = np.loadtxt(root/'references'/f'{name}.csv', delimiter=',', ndmin=2)
        data = np.genfromtxt(root/f'{name}.csv', delimiter=',', names=True)
        data = np.atleast_1d(data)
        color = '#15803d' if row['status'] == 'passed' else '#c2413a'
        ax.plot(reference[:, 0], reference[:, 1], '--', color='#64748b', lw=1.5)
        if len(data):
            ax.plot(data['x'], data['y'], color=color, lw=1.8)
            scale = max(.15, np.ptp(reference[:, 0]), np.ptp(reference[:, 1])) * .07
            indexes = np.unique(np.linspace(0, len(data)-1, 7).astype(int))
            ax.quiver(data['x'][indexes], data['y'][indexes],
                      np.cos(data['yaw'][indexes])*scale, np.sin(data['yaw'][indexes])*scale,
                      angles='xy', scale_units='xy', scale=1, color='#d97706', width=.006)
        gx, gy, yaw = reference[-1]
        ax.scatter([gx], [gy], marker='*', s=65, color='#2563eb', zorder=5)
        ax.arrow(gx, gy, .15*math.cos(yaw), .15*math.sin(yaw), head_width=.04,
                 length_includes_head=True, color='#2563eb')
        ax.scatter([reference[0, 0]], [reference[0, 1]], s=15, color='#0f172a')
        ax.set_title(f'{index:02d}  {name}   |   {row["status"].upper()}', color=color,
                     loc='left', fontsize=9, weight='bold')
        ax.set_xlabel(f'{float(row["duration_s"]):.1f}s | peak {100*float(row["path_peak_m"]):.1f}cm | '
                      f'final yaw {math.degrees(float(row["final_yaw_rad"])):.1f} deg', fontsize=8)
        ax.set_aspect('equal', adjustable='datalim')
        ax.margins(.20)
        ax.grid(alpha=.18)
    for ax in list(axes.flat)[len(metrics):]:
        ax.set_visible(False)
    fig.suptitle(f'XY-derived heading + independent final yaw  |  {passed}/{len(metrics)} passed\n'
                 f'NUMERICAL CLOSED LOOP  |  {setup}\n'
                 'Shared chassis executor + independent servo plant; no ROS / Gazebo / odometry\n'
                 'Dashed: XY route   Orange arrows: body yaw   Blue star/arrow: final pose',
                 fontsize=14, weight='bold', y=.992)
    fig.tight_layout(rect=(0, 0, 1, .951), h_pad=2)
    fig.savefig(root/'heading_32_case_overview.png', dpi=130)
    plt.close(fig)

    selected = [n for n in ['final_yaw', 'reverse', 'crab_right', 'figure_eight', 'slalom']
                if any(r['scenario'] == n for r in metrics)]
    if not selected:
        return
    fig, axes = plt.subplots(len(selected), 3, figsize=(18, max(5, len(selected)*3.8)), squeeze=False)
    for axs, name in zip(axes, selected):
        row = next(r for r in metrics if r['scenario'] == name)
        data = np.atleast_1d(np.genfromtxt(root/f'{name}.csv', delimiter=',', names=True))
        source = np.loadtxt(root/'references'/f'{name}.csv', delimiter=',', ndmin=2)
        if not len(data):
            continue
        geom = np.array(geometric_reference(source.tolist()))
        arc = np.r_[0, np.cumsum(np.hypot(np.diff(geom[:, 0]), np.diff(geom[:, 1])))]
        progress = np.clip(arc[-1]-data['remaining'], 0, arc[-1])
        yaw = np.interp(progress, arc, np.unwrap(geom[:, 2]))
        axs[0].plot(source[:, 0], source[:, 1], '--', color='#64748b', label='XY reference')
        axs[0].plot(data['x'], data['y'], color='#0891b2', label='Measured plant')
        axs[0].scatter([source[-1, 0]], [source[-1, 1]], marker='*', color='#2563eb', s=80)
        axs[0].set_aspect('equal', adjustable='datalim')
        axs[0].set_title(f'{name}: {row["status"]}', loc='left', weight='bold')
        axs[0].set(xlabel='x [m]', ylabel='y [m]')
        axs[1].plot(data['t'], np.degrees(np.unwrap(data['yaw'])), label='Body yaw', color='#0891b2')
        axs[1].plot(data['t'], np.degrees(yaw), '--', label='Geometric travel yaw', color='#64748b')
        goal_yaw = source[-1, 2]
        goal_yaw += round((data['yaw'][-1]-goal_yaw)/(2*math.pi))*2*math.pi
        axs[1].axhline(math.degrees(goal_yaw), color='#2563eb', ls=':', label='Final yaw')
        axs[1].set(xlabel='Simulation time [s]', ylabel='Yaw [deg]')
        axs[2].plot(data['t'], data['goal_weight'], color='#7c3aed', label='Final yaw weight')
        axs[2].step(data['t'], data['branch_phase']/4, color='#d97706', alpha=.6,
                    label='Internal phase / 4')
        twin = axs[2].twinx()
        twin.plot(data['t'], data['request_wz'], color='#0891b2', alpha=.7, label='Requested wz')
        twin.set_ylabel('wz [rad/s]', color='#0891b2')
        axs[2].set(xlabel='Simulation time [s]', ylabel='Weight / normalized phase', ylim=(-.05, 1.1))
        for ax in axs:
            ax.grid(alpha=.18)
            ax.legend(loc='best', fontsize=7)
    fig.suptitle('Terminal handoff and selected cases\nNumerical plant recordings; see metrics for pass/fail',
                 fontsize=16, weight='bold')
    fig.tight_layout(rect=(0, 0, 1, .95), h_pad=2.5)
    fig.savefig(root/'heading_handoff_and_failures.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    render(parser.parse_args().directory)
