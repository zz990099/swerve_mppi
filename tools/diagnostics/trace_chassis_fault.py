"""Add fault-transition logs to the pinned chassis source used by this test."""
from pathlib import Path

path = Path('companion-chassis/swerve_chassis_controller/src/controller.cpp')
source = path.read_text()

def replace(old, new):
    global source
    if source.count(old) != 1:
        raise ValueError('Pinned chassis source differs from the diagnostic target')
    source = source.replace(old, new)

replace('''            if (msg->header.frame_id != body_frame_ || !valid_stamp(msg->header.stamp) ||
                t > now().seconds() + 0.01 || now().seconds() - t > cmd_timeout_ ||
                (have_command_ && t <= command_stamp_)) {
              stop(1);''', '''            const auto ros_now = now().seconds();
            const char * reason =
                msg->header.frame_id != body_frame_ ? "frame" :
                !valid_stamp(msg->header.stamp) ? "invalid_stamp" :
                t > ros_now + 0.01 ? "future_stamp" :
                ros_now - t > cmd_timeout_ ? "expired_stamp" :
                have_command_ && t <= command_stamp_ ? "duplicate_or_out_of_order" : nullptr;
            if (reason) {
              RCLCPP_WARN(
                get_logger(), "FAULT_TRACE command %s stamp=%.9f now=%.9f accepted=%.9f seq=%llu"
                " wall_age=%.6f frame=%s requested=(%.6f,%.6f,%.6f) fault_before=%u",
                reason, t, ros_now, command_stamp_, static_cast<unsigned long long>(sequence_),
                have_command_ ? age(command_wall_) : -1., msg->header.frame_id.c_str(),
                msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z,
                static_cast<unsigned>(fault_));
              stop(1);''')

replace('''    if (!core::finite(value) || msg.linear.z != 0 || msg.angular.x != 0 || msg.angular.y != 0) {
      stop(1);''', '''    if (!core::finite(value) || msg.linear.z != 0 || msg.angular.x != 0 || msg.angular.y != 0) {
      RCLCPP_WARN(get_logger(), "FAULT_TRACE invalid_twist stamp=%.9f now=%.9f", t, now().seconds());
      stop(1);''')

replace('''    if (!valid_stamp(msg.header.stamp) || t > now().seconds() + 0.01 ||
        now().seconds() - t > feedback_timeout_) {
      stop(2);''', '''    const auto ros_now = now().seconds();
    if (!valid_stamp(msg.header.stamp) || t > ros_now + 0.01 ||
        ros_now - t > feedback_timeout_) {
      RCLCPP_WARN(
        get_logger(), "FAULT_TRACE feedback stamp=%.9f now=%.9f accepted=%.9f fault_before=%u",
        t, ros_now, feedback_stamp_, static_cast<unsigned>(fault_));
      stop(2);''')

replace('''    if (!feedback_fresh()) {
      stop(2);
    } else if (!command_fresh()) {
      stop(1);''', '''    if (!feedback_fresh()) {
      if (!fault_) RCLCPP_WARN(
        get_logger(), "FAULT_TRACE feedback_watchdog now=%.9f stamp=%.9f wall_age=%.6f",
        t, feedback_stamp_, age(feedback_wall_));
      stop(2);
    } else if (!command_fresh()) {
      if (!fault_) RCLCPP_WARN(
        get_logger(), "FAULT_TRACE command_watchdog now=%.9f stamp=%.9f wall_age=%.6f",
        t, command_stamp_, age(command_wall_));
      stop(1);''')

path.write_text(source)
