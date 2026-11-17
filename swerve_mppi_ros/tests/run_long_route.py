"""One 100 m route in a single unchanged Nav2 FollowPath action.

This experiment extends only the recording harness. Controller/chassis parameters
match the 32-case benchmark (seed 42, 1 m/s, map path / odom control).
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys


NAME = 'realistic_100m'


def make_route():
    points = [(0., 0., 0.)]
    segments = []
    distance = 0.

    def segment(label, length, curvature=0.):
        nonlocal distance
        first = len(points)-1
        steps = max(1, math.ceil(length/.05))
        step = length/steps
        for _ in range(steps):
            x, y, a = points[-1]
            if abs(curvature) < 1e-12:
                x += step*math.cos(a)
                y += step*math.sin(a)
            else:
                b = a+curvature*step
                x += (math.sin(b)-math.sin(a))/curvature
                y += (math.cos(a)-math.cos(b))/curvature
                a = b
            points.append((x, y, a))
        segments.append(dict(label=label, start_index=first, end_index=len(points)-1,
                             start_arc_m=distance, end_arc_m=distance+length,
                             curvature_unscaled=curvature))
        distance += length

    def line(label, length):
        segment(label, length)

    def turn(label, radius, angle):
        segment(label, abs(radius*angle), math.copysign(1./radius, angle))

    def s_bend(label):
        for i, a in enumerate((math.pi/4, -math.pi/4, -math.pi/4, math.pi/4)):
            turn(f'{label}.{i+1}', 2.5, a)

    line('01 Long aisle / departure', 10)
    s_bend('02 Smooth S bend')
    line('03 Aisle connector', 4)
    turn('04 Wide left U turn', 3, math.pi)
    line('05 Returning aisle', 12)
    s_bend('06 Reverse-direction S bend')
    turn('07 Right U turn', 2, -math.pi)
    line('08 Upper aisle', 10)
    turn('09 Circular work area / one full lap', 2, 2*math.pi)
    line('10 Crossing approach', 4)
    turn('11 Right bend to crossing', 3, -math.pi/2)
    line('12 Southbound crossing of earlier aisle', 7)
    turn('13 Lower right bend', 2, -math.pi/2)
    line('14 Lower returning aisle', 8)
    turn('15 Left bend', 2, math.pi/2)
    line('16 Short lower aisle', 4)
    turn('17 Final left bend', 2, math.pi/2)
    line('18 Final straight / docking approach', 6)
    length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(points,points[1:]))
    scale = 100./length
    points = [(x*scale, y*scale, math.remainder(a,2*math.pi)) for x,y,a in points]
    # Intermediate tangent yaw remains metadata; current main ignores it.
    points[-1] = (*points[-1][:2], 0.)
    for s in segments:
        s['start_arc_m'] *= scale
        s['end_arc_m'] *= scale
        s['radius_m'] = scale/abs(s['curvature_unscaled']) if s['curvature_unscaled'] else None
    actual = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a,b in zip(points,points[1:]))
    return dict(scenario=NAME, title='Continuous realistic route / 100 m', reference=points,
                reference_length_m=actual, segments=segments, scale=scale,
                final_body_yaw_rad=0., body_yaw_policy='Intermediate yaw ignored; terminal yaw only',
                environment='Obstacle-free Gazebo physical chassis; complex reference geometry')


def main():
    route = make_route()
    if '--scenario' in sys.argv:
        # The unmodified physical acceptance program sees this one additional
        # path before it builds its argument choices and ordered checkpoints.
        from trajectory_scenarios import PATHS, TITLES
        PATHS.clear()
        PATHS[NAME] = route['reference']
        TITLES[NAME] = route['title']
        import gazebo_tracking
        gazebo_tracking.main()
        return
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--chassis-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'route.json').write_text(json.dumps(route,indent=2)+'\n')
    repo=Path(__file__).resolve().parents[2]
    revision=os.environ.get('GITHUB_SHA') or subprocess.check_output(
        ['git','rev-parse','HEAD'],cwd=repo,text=True).strip()
    (a.output/'experiment.json').write_text(json.dumps(dict(
        controller_base_commit='b317b210858c421ac890a1623d59f67d82796a9b',
        experiment_commit=revision, seed=42, max_speed_m_s=1., path_frame='map',
        control_frame='odom', action_count=1, timeout_s=1200.,
        reference_length_m=route['reference_length_m']),indent=2)+'\n')
    print(f"RUN {NAME}: {route['reference_length_m']:.6f} m; {len(route['reference'])} points",flush=True)
    from run_gazebo import run_case
    run_case(a.chassis_root.resolve(),a.output/NAME,NAME,seed=42,record=True,
             minimum_length=100.,max_speed=1.,path_frame='map',timeout_s=1200.,
             probe_script=Path(__file__).resolve())


if __name__=='__main__':
    main()
