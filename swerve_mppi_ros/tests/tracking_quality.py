"""Fixed physical tracking gates; shared by recordings and offline baseline audit."""
import math


TRACKING_LIMITS = dict(path_peak_m=.10, final_position_error_m=.20,
                       final_yaw_error_rad=math.radians(5))
LIMITS = dict(**TRACKING_LIMITS, stopped_fraction=.55,
              max_request_linear_rate=1.05, max_request_angular_rate=1.5, compute_p99_ms=50.)


def geometric_reference(points, lookahead=.3):
    """Independent XY-only metric reference; final pose yaw stays in the task."""
    clean = []
    for point in points:
        if not clean or math.hypot(point[0]-clean[-1][0], point[1]-clean[-1][1]) > 1e-8:
            clean.append(point)
    if len(clean) < 2:
        return clean
    result = []
    for i, a in enumerate(clean):
        if i == len(clean)-1:
            b = clean[i-1]
            angle = math.atan2(a[1]-b[1], a[0]-b[0])
        else:
            left, bx, by = lookahead, a[0], a[1]
            for j in range(i, len(clean)-1):
                p, q = clean[j], clean[j+1]
                length = math.hypot(q[0]-p[0], q[1]-p[1])
                u = min(1., left/length)
                bx, by = p[0]+u*(q[0]-p[0]), p[1]+u*(q[1]-p[1])
                left -= length
                if left <= 0 or j+2 == len(clean):
                    break
                r = clean[j+2]
                turn = abs(math.atan2((q[0]-p[0])*(r[1]-q[1])-(q[1]-p[1])*(r[0]-q[0]),
                                      (q[0]-p[0])*(r[0]-q[0])+(q[1]-p[1])*(r[1]-q[1])))
                if turn >= math.radians(15):
                    break
            angle = math.atan2(by-a[1], bx-a[0])
        result.append((a[0], a[1], angle))
    return result


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
