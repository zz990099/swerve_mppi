"""Physical path following; optional ideal truth localization via map -> odom."""
import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path as PathMessage
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import FollowPath
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from swerve_chassis_msgs.msg import ChassisState
from rclpy.qos import qos_profile_sensor_data

from trajectory_scenarios import PATHS, ordered_checkpoints, scaled_path
from trajectory_recording import Recording, stamp


def yaw(q):
    return math.atan2(2*q.w*q.z, 1-2*q.z*q.z)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--chassis-root', type=Path, required=True)
    parser.add_argument('--scenario', choices=tuple(PATHS))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--min-path-length', type=float, default=0.)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--path-frame', choices=('map', 'odom'), default='odom')
    args = parser.parse_args()
    if args.output and not args.scenario:
        parser.error('--output requires one --scenario')
    recording = None
    success = False
    failure_reason = ''
    sys.path.insert(0, str(args.chassis_root.resolve() / 'swerve_gazebo_sim' / 'test'))
    from gazebo_truth import GazeboTruth

    rclpy.init()
    node = rclpy.create_node('tracking_acceptance', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    odom = [None]
    execution = [None]
    compute_times = []
    def chassis_state(message):
        execution[0] = message
        if recording is not None:
            recording.execution(message)
    node.create_subscription(ChassisState, '/chassis_controller/state', chassis_state, qos_profile_sensor_data)
    if args.output:
        node.create_subscription(Odometry, '/odom', lambda m: recording.wheel_odometry(m) if recording else None, 10)
        node.create_subscription(TwistStamped, '/cmd_vel',
            lambda m: recording.command(m, node.get_clock().now().nanoseconds*1e-9)
            if recording is not None else None, 10)
    def diagnostics(msg):
        if recording is not None:
            recording.diagnostic(msg)
        for item in msg.status:
            for kv in item.values:
                if kv.key == 'compute_ms': compute_times.append(float(kv.value))
    node.create_subscription(DiagnosticArray, '/FollowPath/diagnostics', diagnostics, 10)
    node.create_subscription(Odometry, '/odom', lambda m: odom.__setitem__(0, m), 10)
    if args.output:
        node.create_subscription(PathMessage, '/FollowPath/prediction',
                                 lambda m: recording.prediction(m) if recording is not None else None, 10)
    if args.output and args.path_frame == 'map':
        node.create_subscription(TFMessage, '/tf',
            lambda m: recording.transforms(m) if recording else None, 100)
    client = ActionClient(node, FollowPath, '/follow_path')
    lifecycle = node.create_client(GetState, '/controller_server/get_state')
    pending_state = [None]
    truth = GazeboTruth('/ground_truth/odom', 'gz')

    def wait(predicate, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if recording is not None and odom[0] is not None:
                recording.observe(truth.history, odom[0], execution[0])
            if predicate():
                return
        raise AssertionError(f'tracking condition timed out, odom={odom[0]}')

    def active():
        if not client.server_is_ready() or not lifecycle.service_is_ready():
            return False
        if pending_state[0] is None:
            pending_state[0] = lifecycle.call_async(GetState.Request())
            return False
        if not pending_state[0].done():
            return False
        ready = pending_state[0].result().current_state.id == State.PRIMARY_STATE_ACTIVE
        pending_state[0] = None
        return ready

    try:
        wait(lambda: odom[0] is not None and truth.latest is not None and
             active() and (args.path_frame == 'odom' or
                           tf_buffer.can_transform('map', 'base_footprint', Time())), 90)
        # Lifecycle activation alone does not mean DDS actuator feedback has
        # arrived. Establish stopped healthy feedback before measuring a case.
        # These zero commands are sent only before any FollowPath goal.
        recovery = node.create_publisher(TwistStamped, '/cmd_vel', 10)
        healthy_since = [None]
        last_zero = [0.]
        def feedback_ready():
            now = node.get_clock().now().nanoseconds*1e-9
            if time.monotonic()-last_zero[0] >= .05:
                zero = TwistStamped()
                zero.header.stamp = node.get_clock().now().to_msg()
                zero.header.frame_id = 'base_footprint'
                recovery.publish(zero)
                last_zero[0] = time.monotonic()
            state = execution[0]
            healthy = (state is not None and state.feedback_valid and state.fault == 0
                       and state.generator_valid and -.01 <= now-stamp(state) < .1
                       and max(map(abs, state.wheel_speeds)) < .05
                       and node.count_subscribers('/chassis_controller/state') >= 2)
            if not healthy:
                healthy_since[0] = None
                return False
            if healthy_since[0] is None:
                healthy_since[0] = now
            return now-healthy_since[0] >= .5
        wait(feedback_ready, 10)
        node.destroy_publisher(recovery)
        initial = odom[0].pose.pose
        initial_truth = truth.latest.pose.pose
        rotation = (0. if args.path_frame == 'map' else
                    wrap(yaw(initial_truth.orientation)-yaw(initial.orientation)))
        origin = ((0., 0., 0., 0.) if args.path_frame == 'map' else
                  (initial.position.x, initial.position.y,
                   initial_truth.position.x, initial_truth.position.y))
        for name in PATHS:
            if args.scenario and name != args.scenario:
                continue
            points = scaled_path(name, args.min_path_length)
            print(f"START {name}", flush=True)
            base = truth.latest.pose.pose if args.path_frame == 'map' else odom[0].pose.pose
            a, bx, by = yaw(base.orientation), base.position.x, base.position.y
            message = PathMessage()
            message.header.frame_id = args.path_frame
            message.header.stamp = node.get_clock().now().to_msg()
            for x, y, heading in points:
                p = PoseStamped()
                p.header = message.header
                p.pose.position.x = bx+math.cos(a)*x-math.sin(a)*y
                p.pose.position.y = by+math.sin(a)*x+math.cos(a)*y
                p.pose.orientation.z = math.sin((a+heading)/2)
                p.pose.orientation.w = math.cos((a+heading)/2)
                message.poses.append(p)
            if args.output:
                recording = Recording(args.output, name, points, args.seed, odom[0].pose.pose,
                                      base if args.path_frame == 'map' else truth.latest.pose.pose, stamp(truth.latest))
                recording.metadata.update(path_frame=args.path_frame,
                    map_reference=[(p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation))
                                   for p in message.poses] if args.path_frame == 'map' else None,
                    localization_source='ideal truth map->odom' if args.path_frame == 'map' else 'none',
                    odometry_source='raw wheel')
                recording._write_metadata()
            goal = FollowPath.Goal()
            goal.path = message
            goal.controller_id = 'FollowPath'
            goal.goal_checker_id = 'goal_checker'
            future = client.send_goal_async(goal)
            wait(future.done, 5)
            handle = future.result()
            assert handle.accepted, 'Nav2 rejected FollowPath'
            result = handle.get_result_async()
            start = time.monotonic()
            if name == 'cancel':
                wait(lambda: math.hypot(odom[0].pose.pose.position.x-bx, odom[0].pose.pose.position.y-by)>.15, 20)
                canceled=handle.cancel_goal_async()
                wait(canceled.done,5)
                assert canceled.result().goals_canceling, 'cancel not accepted'
                wait(result.done,10)
                assert result.result().status == GoalStatus.STATUS_CANCELED
                wait(lambda: execution[0] is not None and max(map(abs,execution[0].wheel_speeds))<.05,5)
                print('PASS cancellation stops measured wheels',flush=True)
                continue
            # Evaluate route order with independent truth, including loops and
            # self intersections whose endpoint alone cannot prove completion.
            waypoints = []
            for item in message.poses:
                x, y = item.pose.position.x-origin[0], item.pose.position.y-origin[1]
                waypoints.append((origin[2]+math.cos(rotation)*x-math.sin(rotation)*y,
                                  origin[3]+math.sin(rotation)*x+math.cos(rotation)*y))
            checkpoints = ordered_checkpoints(waypoints)
            visited = 0
            checkpoint_stamp = -math.inf
            if recording is not None:
                recording.metadata.update(ordered_checkpoints_total=len(checkpoints),
                                          ordered_checkpoints_visited=0)
            tracking_errors=[]
            stopped_samples=0
            total_samples=0
            last_truth_stamp=None
            last_report=start

            def finished():
                nonlocal visited, checkpoint_stamp, stopped_samples, total_samples, last_truth_stamp, last_report
                if time.monotonic()-last_report > 5:
                    last_report=time.monotonic()
                    p=odom[0].pose.pose.position
                    print(f"TRACE {name} xy=({p.x:.5f},{p.y:.5f}) visited={visited} state={execution[0]}",flush=True)
                observation=truth.latest
                stamp=(observation.header.stamp.sec,observation.header.stamp.nanosec)
                if stamp!=last_truth_stamp:
                    last_truth_stamp=stamp
                    p=observation.pose.pose.position
                    errors=[]
                    for (ax,ay),(bx,by) in zip(waypoints,waypoints[1:]):
                        dx,dy=bx-ax,by-ay
                        t=max(0,min(1,((p.x-ax)*dx+(p.y-ay)*dy)/max(1e-12,dx*dx+dy*dy)))
                        errors.append(math.hypot(p.x-ax-t*dx,p.y-ay-t*dy))
                    if errors: tracking_errors.append(min(errors))
                    if execution[0] is not None:
                        total_samples+=1
                        stopped_samples+=max(map(abs,execution[0].wheel_speeds))<0.05
                for observed in truth.history:
                    observed_time = (observed.header.stamp.sec + observed.header.stamp.nanosec*1e-9)
                    if observed_time <= checkpoint_stamp:
                        continue
                    checkpoint_stamp = observed_time
                    if visited < len(checkpoints):
                        p = observed.pose.pose.position
                        qx, qy = checkpoints[visited]
                        if math.hypot(p.x-qx, p.y-qy) < .2:
                            visited += 1
                if recording is not None:
                    recording.metadata['ordered_checkpoints_visited'] = visited
                return result.done()

            wait(finished, max(75., 30+8*args.min_path_length))
            outcome = result.result()
            if recording is not None:
                recording.metadata['nav2_goal_success'] = (outcome.status == GoalStatus.STATUS_SUCCEEDED
                    and outcome.result.error_code == 0)
                recording.metadata.update(ordered_checkpoints_total=len(checkpoints),
                                          ordered_checkpoints_visited=visited)
            assert outcome.status == GoalStatus.STATUS_SUCCEEDED and outcome.result.error_code == 0, outcome
            assert visited == len(checkpoints), \
                f'{name} completed before ordered checkpoints: {visited}/{len(checkpoints)}'
            target = message.poses[-1].pose
            measured = truth.latest.pose.pose
            x, y = target.position.x-origin[0], target.position.y-origin[1]
            tx = origin[2]+math.cos(rotation)*x-math.sin(rotation)*y
            ty = origin[3]+math.sin(rotation)*x+math.cos(rotation)*y
            error = math.hypot(measured.position.x-tx, measured.position.y-ty)
            angle = abs(wrap(yaw(measured.orientation)-yaw(target.orientation)-rotation))
            print(f'{name}: path_rms_m={math.sqrt(sum(e*e for e in tracking_errors)/max(1,len(tracking_errors))):.4f} '
                  f'path_peak_m={max(tracking_errors,default=0):.4f} stopped_fraction={stopped_samples/max(1,total_samples):.3f} '
                  f'wall_s={time.monotonic()-start:.2f} '
                  f'truth_position_error_m={error:.4f} truth_yaw_error_rad={angle:.4f}', flush=True)
            assert error <= .20 and angle <= math.radians(5), (name, error, angle)
        if compute_times:
            compute_times.sort()
            print('compute_ms percentiles:', {p:compute_times[min(len(compute_times)-1,int(len(compute_times)*p/100))] for p in (50,95,99)}, flush=True)
        print('PASS Nav2 FollowPath physical tracking and ordered goal completion', flush=True)
        success = True
    except BaseException as error:
        failure_reason = str(error)
        raise
    finally:
        if recording is not None:
            recording.close(success, failure_reason, compute_times)
        truth.close()
        tf_listener.unregister()
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
    if recording is not None and recording.metadata['status'] != 'passed':
        raise AssertionError(recording.metadata['quality_failures'])


if __name__ == '__main__':
    main()
