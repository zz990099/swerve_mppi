# GoalOnly translation from Spin (0.20.2)

The physical ROS/Gazebo corridor regression exposed a local minimum after a
completed arc: a new 0.5 m translation with a different final yaw began in Spin.
The finite prediction horizon charged mode switching and alignment against a
short translation; repeatedly improving yaw was cheaper than selecting a
translating mode. The controller remained authorized and healthy but never
advanced position.

The standalone `spin_translation` regression starts in measured, confirmed Spin,
with canonical steering, pose (0.93, 0.21, 0.49), GoalOnly target
(1.43, 0.21, 0.89), and the same four corridor obstacles. Against the 0.20.1
library, seed 42 with the sampled ProfileRunner plant failed after 40 simulation
seconds: position error 0.5 m, zero switches, no completion. This reproduces the
algorithm failure independently of ROS, Gazebo, scheduling or tire dynamics.

For an ordered, eligible terminal GoalOnly segment starting in Spin, the planner
now uses its existing checked position-capture policy. This first brakes and
waits for minimum mode dwell, requests a translating mode, and eventually
settles final yaw. Full transition, current context and complete-stop validation
remain mandatory. FollowPath tracking, corner eligibility, mode request receipts,
command timing and all physical/kinematic tolerances are unchanged. This is not
permission to bypass obstacles or mechanically align while driving.

CTest registers direct, execution-guarded and sampled-profile variants with all
five existing seeds. Additional public Controller regressions require stopped
waiting during mode dwell and absent authorization when the current footprint
collides. The entire existing test matrix remains enabled.

Local validation: GCC 13.3 C++17 Release with `-Werror` passed all 58 CTests,
including the fifteen new behavior runs, both public safety checks, the installed
consumer and allocation benchmark. The official pinned ROS format is checked
again by repository CI in Debug and Release. The simulation package must pin
this 0.20.2 commit before rerunning its physical terminal-yaw scenario.
