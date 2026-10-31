#!/usr/bin/env python3
"""Build historical local probes and isolate geometry/terminal policy changes."""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys

# Baseline controller/terminal logic stays untouched. Only the test caller
# replaces intermediate yaw and retains the original final yaw as an independent goal.
GEOMETRY_ADAPTER = r'''
    const Pose final_goal = points.back();
    std::vector<Pose> clean;
    for (const auto & p : points)
      if (clean.empty() || distance(clean.back(),p)>1e-8) clean.push_back(p);
    if (clean.size()>1) {
      points=clean;
      PathView raw(points);
      for (std::size_t i=0;i<points.size();++i) {
        double a=raw.vertex_arc(i), b=std::min(raw.length(),a+.3);
        b=std::min(b,raw.next_corner(a));
        if (i+1==points.size()) a=std::max(raw.vertex_arc(i-1),a-.3);
        auto pa=raw.at(a),pb=raw.at(b);
        points[i].yaw=std::atan2(pb.y-pa.y,pb.x-pa.x);
      }
    }
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--main-source', required=True, type=Path)
    parser.add_argument('--previous-source', required=True, type=Path)
    parser.add_argument('--chassis-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--cases', nargs='+', default=[
        'crab', 'crab_right', 'reverse_diagonal', 'lateral_zigzag',
        'figure_eight', 'straight', 'final_yaw'])
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    chassis = args.chassis_root.resolve()
    if (chassis/'swerve_chassis_core').is_dir():
        chassis = chassis/'swerve_chassis_core'
    recorder = (root/'swerve_mppi_ros/tests/record_portable.cpp').read_text()
    source = output/'recorder.cpp'
    source.write_text(recorder)
    adapter = recorder.replace('    Config config;', GEOMETRY_ADAPTER+'\n    Config config;')
    adapter = adapter.replace('      input.goal = path.tracking_goal();',
                              '      input.goal = path.tracking_goal();\n'
                              '      if (distance(input.goal, final_goal)<.001 && '
                              'path.tracking_goal_active()) input.goal=final_goal;')
    adapter = adapter.replace('checker.reached(state, path.goal(), time)',
                              'checker.reached(state, final_goal, time)')
    adapter = adapter.replace('wrap_angle(state.pose.yaw - path.goal().yaw)',
                              'wrap_angle(state.pose.yaw - final_goal.yaw)')
    geometry = output/'geometry_adapter.cpp'
    geometry.write_text(adapter)
    compiler = shutil.which('g++') or shutil.which('clang++')
    if not compiler:
        parser.error('a C++17 compiler is required')
    for name, snapshot, harness, defines in [
            ('main', args.main_source, source, []),
            ('geometry-only', args.main_source, geometry, []),
            ('previous', args.previous_source, source, ['-DSWERVE_HEADING_REFERENCE=1'])]:
        snapshot = snapshot.resolve()
        core = snapshot/'swerve_mppi_core'
        command = [compiler, '-O3', '-DNDEBUG', '-std=c++17', *defines,
                   '-I'+str(core/'include'),
                   '-I'+str(snapshot/'swerve_mppi_ros/include'), '-I'+str(chassis/'include'),
                   str(harness), *map(str, sorted((core/'src').glob('*.cpp'))),
                   *map(str, sorted((chassis/'src').glob('*.cpp'))), '-o', str(output/name)]
        subprocess.run(command, check=True)
    runner = root/'swerve_mppi_ros/tests/run_portable_recordings.py'
    for name, executable, flags in [('main', 'main', []),
                                     ('geometry-only', 'geometry-only', []),
                                     ('terminal-only', 'previous', ['--keep-path-yaw']),
                                     ('both-before', 'previous', [])]:
        subprocess.run([sys.executable, str(runner), '--executable', str(output/executable),
                        '--output', str(output/(name+'-results')), '--cases', *args.cases,
                        '--no-plot', *flags], check=True)


if __name__ == '__main__':
    main()
