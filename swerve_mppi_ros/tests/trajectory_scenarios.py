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

TITLES = {
    'straight': 'Forward straight', 'smooth_s': 'Smooth S bend / fixed body yaw',
    'smooth_curve': 'Regular curve / tangent body yaw',
    'crab': 'Perpendicular start / left', 'crab_right': 'Perpendicular start / right',
    'reverse': 'Opposite direction / reverse', 'reverse_curve': 'Reverse curve',
    'combined': 'Translation + rotation', 'spin': 'In-place rotation',
    'final_yaw': 'Translation + final heading', 'out_and_back': 'Out and back',
    'closed_loop': 'Ordered square loop',
}
BENCHMARK_CASES = tuple(TITLES)
