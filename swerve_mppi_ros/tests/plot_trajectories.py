"""Render recorded observations, never simulated or reconstructed trajectories."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from trajectory_scenarios import TITLES


def plot_round(directory, output=None):
    directory = Path(directory)
    manifest = json.loads((directory/'round.json').read_text())
    names = manifest['cases']
    columns = 4
    rows = math.ceil(len(names)/columns)
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(rows, columns, figsize=(19.2, rows*4.6), squeeze=False)
    summaries = []
    for ax, name in zip(axes.flat, names):
        case = directory/name
        metadata_path = case/'result.json'
        if not metadata_path.exists():
            ax.set_title(TITLES.get(name, name), loc='left', weight='bold')
            ax.text(.5, .5, 'FAILED / no recording', ha='center', transform=ax.transAxes,
                    color='#c03221')
            summaries.append(dict(scenario=name, status='failed', reason='no recording'))
            continue
        data = json.loads(metadata_path.read_text())
        with (case/'trajectory.csv').open() as file:
            samples = list(csv.DictReader(file))
        points = data['reference']
        rx, ry = [p[0] for p in points], [p[1] for p in points]
        ax.plot(rx, ry, '--', color='#2563b9', linewidth=2, zorder=2)
        if samples:
            tx = [float(p['truth_x_m']) for p in samples]
            ty = [float(p['truth_y_m']) for p in samples]
            ox = [float(p['odom_x_m']) for p in samples]
            oy = [float(p['odom_y_m']) for p in samples]
            ax.plot(ox, oy, ':', color='#64748b', linewidth=1.3, alpha=.8, zorder=3)
            ax.plot(tx, ty, color='#de6b24', linewidth=1.8, zorder=4)
            # Arrows show measured body yaw, not travel direction (crab/reverse differ).
            for i in sorted(set([0, len(samples)//3, 2*len(samples)//3, len(samples)-1])):
                angle = float(samples[i]['truth_yaw_rad'])
                ax.arrow(tx[i], ty[i], .12*math.cos(angle), .12*math.sin(angle),
                         width=.004, head_width=.035, length_includes_head=True,
                         color='#de6b24', alpha=.65, zorder=5)
            ax.scatter(tx[-1], ty[-1], s=24, color='#de6b24', zorder=6)
        ax.scatter([0], [0], marker='o', s=34, color='#111827', zorder=7)
        ax.arrow(0, 0, .2, 0, head_width=.045, width=.006,
                 color='#111827', length_includes_head=True, zorder=7)
        ax.scatter(rx[-1], ry[-1], marker='*', s=110, color='#2563b9', zorder=6)
        ga = points[-1][2]
        ax.arrow(rx[-1], ry[-1], .18*math.cos(ga), .18*math.sin(ga),
                 width=.004, head_width=.035, color='#2563b9', zorder=3)
        # Keep tiny in-place motions readable and do not distort lateral movement.
        xlim, ylim = ax.get_xlim(), ax.get_ylim()
        span = max(.7, xlim[1]-xlim[0], ylim[1]-ylim[0])*1.15
        mx, my = sum(xlim)/2, sum(ylim)/2
        ax.set(xlim=(mx-span/2, mx+span/2), ylim=(my-span/2, my+span/2),
               xlabel='Initial-body x [m]', ylabel='Initial-body y [m]')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=.18)
        passed = data['status'] == 'passed'
        ax.set_title(TITLES.get(name, name), loc='left', weight='bold', fontsize=10)
        ax.text(.98, .98, 'TRACK OK' if passed else 'FAIL', transform=ax.transAxes,
                ha='right', va='top', color='#15803d' if passed else '#c03221', weight='bold',
                bbox=dict(facecolor='white', alpha=.9, edgecolor='none'))
        if samples:
            length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))
            caption = (f"L {length:.1f} m | peak {data['path_peak_m']*100:.1f} cm\n"
                       f"RMS {data['path_rms_m']*100:.1f} cm | final {data['final_position_error_m']*100:.1f} cm\n"
                       f"yaw {math.degrees(data['final_yaw_error_rad']):.1f} deg | "
                       f"{data['wall_duration_s']:.1f} s | {len(samples)} observations")
            ax.text(.02, .02, caption, transform=ax.transAxes, fontsize=8,
                    bbox=dict(facecolor='white', alpha=.85, edgecolor='none'), zorder=9)
        summaries.append({k: v for k, v in data.items() if k != 'reference'})
    for ax in list(axes.flat)[len(names):]:
        ax.set_visible(False)
    passed = sum(item.get('status') == 'passed' for item in summaries)
    fig.suptitle(f"MPPI / Gazebo physical trajectories — Round {manifest['round']:02d}\n"
                 f"Speed limit {manifest.get('max_speed_m_s') or 0.8:g} m/s | Seed {manifest['seed']}  |  {passed}/{len(names)} tracking checks passed  |  "
                 f"ROS 2 Jazzy + Gazebo Harmonic  |  commit {manifest['commit'][:8]}",
                 fontsize=17, weight='bold', y=.995)
    handles = [Line2D([0], [0], color='#2563b9', linestyle='--', label='Reference path / goal ★'),
               Line2D([0], [0], color='#de6b24', label='Gazebo ground truth / body-yaw arrows'),
               Line2D([0], [0], color='#64748b', linestyle=':', label='Wheel odometry'),
               Line2D([0], [0], color='#111827', marker='o', label='Start / initial body +x')]
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(.5, .945),
               ncol=4, frameon=False, fontsize=10)
    fig.tight_layout(rect=(0, .04, 1, .925), h_pad=2.3, w_pad=2.0)
    fig.text(.5, .012, 'Fresh simulation per case; axes aligned with initial body; arrows show body yaw. '
             'Failed cases retain partial recordings.\n'
             'TRACK OK: path RMS <= 3 cm / peak <= 7.5 cm; heading, endpoint, request slew, stop fraction and duration gates. '
             'See result.json for every limit and failure.',
             ha='center', fontsize=9, color='#475569')
    output = Path(output) if output else directory/f"round_{manifest['round']:02d}.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.stem+'.tmp.png')
    fig.savefig(temporary, format='png', dpi=160, facecolor='white')
    temporary.replace(output)
    plt.close(fig)
    (directory/'summary.json').write_text(json.dumps(dict(**manifest, results=summaries), indent=2)+'\n')
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    print(plot_round(args.directory, args.output))
