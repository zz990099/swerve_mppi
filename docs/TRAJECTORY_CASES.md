# Physical trajectory benchmark: 32 cases

CI executes one round with seed 42. Every translational reference is scaled to
12 m; the in-place rotation retains its 0.9 rad target. Both controller and
physical linear-speed caps are 1 m/s. Paths are in map, local control is in odom,
and optional ideal truth map-to-odom localization compensates raw wheel drift.

| # | Case | Description |
|---|---|---|
| 1 | `straight` | Forward straight |
| 2 | `smooth_s` | Smooth S bend / geometric heading |
| 3 | `smooth_curve` | Regular curve / tangent body yaw |
| 4 | `crab` | Perpendicular start / left |
| 5 | `crab_right` | Perpendicular start / right |
| 6 | `reverse` | Opposite direction / reverse |
| 7 | `reverse_curve` | Reverse curve |
| 8 | `combined` | Translation + rotation |
| 9 | `spin` | In-place rotation |
| 10 | `final_yaw` | Translation + final heading |
| 11 | `out_and_back` | Out and back |
| 12 | `closed_loop` | Ordered square loop |
| 13 | `diagonal_left` | Diagonal / forward left |
| 14 | `diagonal_right` | Diagonal / forward right |
| 15 | `reverse_diagonal` | Diagonal / reverse left |
| 16 | `l_turn_left` | Sharp L turn / left |
| 17 | `l_turn_right` | Sharp L turn / right |
| 18 | `zigzag` | Zigzag / fixed body yaw |
| 19 | `lateral_zigzag` | Lateral zigzag |
| 20 | `u_turn` | U turn / tangent body yaw |
| 21 | `hairpin_fixed_yaw` | Hairpin / fixed body yaw |
| 22 | `circle_cw` | Full circle / clockwise |
| 23 | `circle_ccw` | Full circle / counterclockwise |
| 24 | `figure_eight` | Figure eight / self crossing |
| 25 | `slalom` | Slalom / three oscillations |
| 26 | `lateral_s` | Lateral S bend |
| 27 | `spiral_out` | Expanding spiral / two turns |
| 28 | `rounded_rectangle` | Rounded rectangle / tangent yaw |
| 29 | `cloverleaf` | Four-leaf / repeated crossings |
| 30 | `heading_sweep` | Straight + final 180 deg rotation |
| 31 | `reverse_s` | Reverse S bend |
| 32 | `stop_turn_go` | Square / stop, rotate, translate |

All routes require ordered interior checkpoints (0.75 m spacing, 0.20 m capture
radius). Repeated intersections retain their traversal order. Acceptance also
requires action completion, peak deviation <= 10 cm, endpoint <= 20 cm and final
yaw <= 5 degrees. Failed/partial runs remain in the montage. No controller or
physical chassis algorithms are changed by this test expansion.

External intermediate yaw no longer constrains body heading. Names retain their
original XY shapes, with a separate final yaw. Repeated intermediate XY/yaw
markers do not request rotations. See GEOMETRIC_HEADING_VALIDATION.md.
