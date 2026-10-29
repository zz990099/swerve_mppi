"""Speed statistics from independent source-stamped truth observations."""
import math


def observed_speeds(samples):
    """Truth-derived linear speed; 0.2 s centred secants for colour only.

    Average is raw observed polyline distance / source simulation duration,
    including initial steering and stops. It is not displacement / duration.
    """
    t = [float(s['sim_time_s']) for s in samples]
    xy = [(float(s['truth_x_m']), float(s['truth_y_m'])) for s in samples]
    if any(b <= a for a, b in zip(t, t[1:])):
        raise ValueError('truth source stamps must increase')
    travelled = sum(math.dist(a, b) for a, b in zip(xy, xy[1:]))
    elapsed = t[-1]-t[0] if len(t) > 1 else 0.
    speeds = []
    left = right = 0
    for i, stamp in enumerate(t):
        while left < i and t[left] < stamp-.1:
            left += 1
        right = max(right, i)
        while right+1 < len(t) and t[right+1] <= stamp+.1:
            right += 1
        a, b = left, right
        if a == b:
            a, b = max(0, i-1), min(len(t)-1, i+1)
        speeds.append(math.dist(xy[a], xy[b])/(t[b]-t[a]) if b > a else 0.)
    return speeds, travelled/elapsed if elapsed else 0., travelled, elapsed

