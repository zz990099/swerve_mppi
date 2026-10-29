"""Fixed physical tracking gates; shared by recordings and offline baseline audit."""
import math


TRACKING_LIMITS = dict(path_peak_m=.10, final_position_error_m=.20,
                       final_yaw_error_rad=math.radians(5))
LIMITS = dict(**TRACKING_LIMITS, stopped_fraction=.55,
              max_request_linear_rate=1.05, max_request_angular_rate=1.5, compute_p99_ms=50.)


def reference_match(x, y, points):
    best = (float('inf'), points[0][2])
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        t = max(0., min(1., ((x-a[0])*dx+(y-a[1])*dy)/max(1e-12, dx*dx+dy*dy)))
        error = math.hypot(x-a[0]-t*dx, y-a[1]-t*dy)
        yaw = a[2]+t*math.remainder(b[2]-a[2], 2*math.pi)
        if error < best[0]:
            best = (error, yaw)
    return best


def evaluate(metadata):
    points = metadata['reference']
    length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))
    rotation = sum(abs(math.remainder(b[2]-a[2], 2*math.pi)) for a, b in zip(points, points[1:]))
    limits = dict(LIMITS)
    limits['wall_duration_s'] = max(8., 5+4*max(length/.8, rotation/.8))
    if metadata['scenario'] == 'cancel':
        limits.pop('final_position_error_m')
        limits.pop('final_yaw_error_rad')
    failures = []
    for key, bound in limits.items():
        value = metadata.get(key)
        if value is None or not math.isfinite(value) or value > bound:
            failures.append(f'{key}={value}, limit={bound}')
    if metadata.get('samples', 0) < 2:
        failures.append('fewer than two independent observations')
    return limits, failures
