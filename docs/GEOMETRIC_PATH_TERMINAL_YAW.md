# Geometric path and terminal yaw

## Contract

The path is an ordered sequence of positions. Intermediate pose yaw is ignored
by reference speed, seed generation, steering-branch handling and normal tracking
cost. The final pose yaw remains part of goal completion.

Consecutive points at the same position are collapsed. This removes intermediate
rotation events while preserving the last point and therefore the final goal
orientation. A path containing only one position still supports in-place final
rotation.

The goal critic enables final yaw only after measured ordered remaining distance
enters `goal_heading_activation_distance`. Within that region it also scores
Euclidean endpoint error, preventing final orientation from buying a shortcut
through the geometric route. The controller supplies a final-pose seed containing
position feedback and yaw correction; MPPI still scores and selects it alongside
braking, slow, recovery and sampled candidates.

## Regression coverage

- A reference with arbitrary intermediate yaw has the same speed and unit twist
  as the same geometric line with different yaw values.
- Intermediate coincident-position yaw changes no longer create route sections;
  the final yaw is retained.
- Portable closed loops no longer gate body yaw against translating waypoint yaw;
  final yaw remains required by `GoalChecker`.
- The existing 32-case physical workflow retains speed-gradient trajectory plots
  and endpoint yaw checks.

## Known local result before physical CI

Core, support, Python recording and the 28 ordinary portable closed loops pass.
The dedicated long-route stagnation regression exposes terminal-position drift
while rotating on `heading_sweep`; its 10 cm corridor gate can be exceeded. This
is recorded rather than hidden or used to relax the gate. The physical 32-case
recording is authoritative for the resulting path and endpoint behavior.
