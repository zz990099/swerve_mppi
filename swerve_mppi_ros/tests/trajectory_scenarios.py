"""Reference paths in the robot's initial body frame (metres, radians)."""
import math


LEGACY = {
    'cancel': [(0, 0, 0), (2, 0, 0)],
    'combined': [(0, 0, 0), (.8, .5, .6)],
    's_curve': [(0, 0, 0), (.4, .15, 0), (.8, -.15, 0), (1.2, 0, 0)],
    'straight': [(0, 0, 0), (1, 0, 0)],
    'crab': [(0, 0, 0), (0, .8, 0)],
    'reverse': [(0, 0, 0), (-.8, 0, 0)],
    'spin': [(0, 0, 0), (0, 0, .9)],
    'final_yaw': [(0, 0, 0), (.8, 0, -.9)],
    'curve': [(0, 0, 0), (.4, .1, .2), (.9, .25, .2), (1.3, .25, 0)],
    'out_and_back': [(0, 0, 0), (.8, 0, 0), (0, 0, 0)],
    'closed_loop': [(0, 0, 0), (.8, 0, 0), (.8, .8, 0), (0, .8, 0), (0, 0, 0)],
}
PATHS = dict(LEGACY)
PATHS['crab_right'] = [(0, 0, 0), (0, -.8, 0)]
# Dense smooth references: these differ from the legacy polygonal acceptance paths.
PATHS['smooth_s'] = [(1.8*t/40, .22*math.sin(2*math.pi*t/40), 0) for t in range(41)]
PATHS['smooth_curve'] = []
for i in range(41):
    t = i/40
    # Cubic Bezier, desired body yaw follows its tangent.
    x = 3*(1-t)**2*t*.5 + 3*(1-t)*t*t*1.1 + t**3*1.6
    y = 3*(1-t)*t*t*.6 + t**3*.6
    dx = 3*((1-t)**2*.5 + 2*(1-t)*t*.6 + t*t*.5)
    dy = 3*(2*(1-t)*t*.6)
    PATHS['smooth_curve'].append((x, y, math.atan2(dy, dx)))
PATHS['reverse_curve'] = [(-x, -y, heading) for x, y, heading in PATHS['smooth_curve']]


def arc(direction=1, sweep=2*math.pi, fixed_yaw=False):
    points = []
    for i in range(161):
        a = sweep*i/160
        points.append((.7*math.sin(a), direction*.7*(1-math.cos(a)),
                       0. if fixed_yaw else direction*a))
    return points


# Twenty additional stress cases, each starting at the origin. Body heading
# is independent of travel direction, as appropriate for a swerve chassis.
PATHS.update({
    'diagonal_left': [(0, 0, 0), (1, 1, 0)],
    'diagonal_right': [(0, 0, 0), (1, -1, 0)],
    'reverse_diagonal': [(0, 0, 0), (-1, .7, 0)],
    'l_turn_left': [(0, 0, 0), (1, 0, 0), (1, 1, 0)],
    'l_turn_right': [(0, 0, 0), (1, 0, 0), (1, -1, 0)],
    'zigzag': [(0, 0, 0), (.5, .35, 0), (1, -.35, 0), (1.5, .35, 0), (2, 0, 0)],
    'lateral_zigzag': [(0, 0, 0), (.3, .5, 0), (-.3, 1, 0), (.3, 1.5, 0), (0, 2, 0)],
    'u_turn': arc(sweep=math.pi),
    'hairpin_fixed_yaw': arc(sweep=math.pi, fixed_yaw=True),
    'circle_cw': arc(direction=-1),
    'circle_ccw': arc(),
    'figure_eight': [(math.sin(t), .5*math.sin(2*t), 0.)
                     for t in [2*math.pi*i/240 for i in range(241)]],
    'slalom': [(2*i/180, .18*math.sin(6*math.pi*i/180), 0.) for i in range(181)],
    'lateral_s': [(-y, x, 0.) for x, y, _ in PATHS['smooth_s']],
    'spiral_out': [((.2+.06*t)*math.cos(t)-.2, (.2+.06*t)*math.sin(t), 0.)
                   for t in [4*math.pi*i/240 for i in range(241)]],
    'cloverleaf': [(.8*math.sin(2*t)*math.cos(t), .8*math.sin(2*t)*math.sin(t), 0.)
                  for t in [2*math.pi*i/320 for i in range(321)]],
    'heading_sweep': [(2*i/80, 0., math.pi*i/80) for i in range(81)],
    'reverse_s': [(-x, -y, 0.) for x, y, _ in PATHS['smooth_s']],
    'stop_turn_go': [(0, 0, 0), (.7, 0, 0), (.7, 0, math.pi/2),
                     (.7, .7, math.pi/2), (.7, .7, math.pi), (0, .7, math.pi),
                     (0, .7, 3*math.pi/2), (0, 0, 3*math.pi/2)],
})

# A closed rounded rectangle: straight segments joined by tangent quarter arcs.
rounded = [(0., 0., 0.), (.75, 0., 0.)]
for cx, cy, start, end, straight in [
        (.75, .25, -math.pi/2, 0., (.75+ .25, .75, math.pi/2)),
        (.75, .75, 0., math.pi/2, (0., 1., math.pi)),
        (0., .75, math.pi/2, math.pi, (-.25, .25, 3*math.pi/2)),
        (0., .25, math.pi, 3*math.pi/2, None)]:
    for i in range(1, 21):
        a = start+(end-start)*i/20
        rounded.append((cx+.25*math.cos(a), cy+.25*math.sin(a), a+math.pi/2))
    if straight is not None:
        rounded.append(straight)
PATHS['rounded_rectangle'] = rounded

TITLES = {
    'straight': 'Forward straight', 'smooth_s': 'Smooth S bend / geometric heading',
    'smooth_curve': 'Regular curve / tangent body yaw',
    'crab': 'Perpendicular start / left', 'crab_right': 'Perpendicular start / right',
    'reverse': 'Opposite direction / reverse', 'reverse_curve': 'Reverse curve',
    'combined': 'Translation + rotation', 'spin': 'In-place rotation',
    'final_yaw': 'Translation + final heading', 'out_and_back': 'Out and back',
    'closed_loop': 'Ordered square loop',
    'diagonal_left': 'Diagonal / forward left',
    'diagonal_right': 'Diagonal / forward right',
    'reverse_diagonal': 'Diagonal / reverse left',
    'l_turn_left': 'Sharp L turn / left',
    'l_turn_right': 'Sharp L turn / right',
    'zigzag': 'Zigzag / geometric heading',
    'lateral_zigzag': 'Lateral zigzag',
    'u_turn': 'U turn / tangent body yaw',
    'hairpin_fixed_yaw': 'Hairpin / geometric heading',
    'circle_cw': 'Full circle / clockwise',
    'circle_ccw': 'Full circle / counterclockwise',
    'figure_eight': 'Figure eight / self crossing',
    'slalom': 'Slalom / three oscillations',
    'lateral_s': 'Lateral S bend',
    'spiral_out': 'Expanding spiral / two turns',
    'rounded_rectangle': 'Rounded rectangle / tangent yaw',
    'cloverleaf': 'Four-leaf / repeated crossings',
    'heading_sweep': 'Straight + final 180 deg rotation',
    'reverse_s': 'Reverse S bend',
    'stop_turn_go': 'Square / geometric heading + final yaw',
}
BENCHMARK_CASES = tuple(TITLES)


def ordered_checkpoints(points, spacing=.75):
    """Interior arc-length checkpoints; repeated crossings retain route order."""
    result = []
    total = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))
    travelled, target = 0., spacing
    for a, b in zip(points, points[1:]):
        length = math.hypot(b[0]-a[0], b[1]-a[1])
        if length < 1e-12:
            continue
        while target < total-1e-8 and target <= travelled+length+1e-8:
            u = min(1., (target-travelled)/length)
            result.append((a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1])))
            target += spacing
        travelled += length
    return result


def scaled_path(name, minimum_length=0.):
    """Scale translation only; retain body yaw and densify to <= 5 cm spacing."""
    if not math.isfinite(minimum_length) or minimum_length < 0:
        raise ValueError('minimum length must be finite and nonnegative')
    points = PATHS[name]
    if minimum_length == 0:
        return list(points)
    length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))
    if length < 1e-9:
        return list(points)
    scale = max(1., minimum_length/length)
    scaled = [(x*scale, y*scale, yaw) for x, y, yaw in points]
    result = [scaled[0]]
    for a, b in zip(scaled, scaled[1:]):
        count = max(1, math.ceil(math.hypot(b[0]-a[0], b[1]-a[1])/.05))
        for i in range(1, count+1):
            t = i/count
            result.append((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]),
                           a[2]+t*math.remainder(b[2]-a[2], 2*math.pi)))
    return result
