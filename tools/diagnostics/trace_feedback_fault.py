"""Instrument the pinned chassis source for the isolation run without changing its gates."""
from pathlib import Path

path = Path('companion-chassis/swerve_chassis_controller/src/controller.cpp')
source = path.read_text()


def replace_once(old, new):
    global source
    assert source.count(old) == 1, old
    source = source.replace(old, new)


replace_once(
    '  void feedback(const sensor_msgs::msg::JointState & msg)\n',
    '''  void trace_feedback_fault(
      const char * reason, double incoming, double clock, const char * detail = "")
  {
    if (fault_ || sequence_ == 0) return;
    RCLCPP_WARN(get_logger(),
        "FEEDBACK_FAULT reason=%s incoming=%.17g clock=%.17g delta_ns=%.6f accepted=%.17g "
        "feedback_wall_age_ms=%.3f command=%.9f sequence=%llu "
        "request=(%.6f,%.6f,%.6f) detail=%s",
        reason, incoming, clock, (incoming - clock) * 1e9, feedback_stamp_,
        age(feedback_wall_) * 1e3, command_stamp_,
        static_cast<unsigned long long>(sequence_),
        requested_.vx, requested_.vy, requested_.wz, detail);
  }
  void feedback(const sensor_msgs::msg::JointState & msg)
''')
replace_once(
    '''    if (!valid_stamp(msg.header.stamp) || t > now().seconds() + 0.01 ||
        now().seconds() - t > feedback_timeout_) {
      stop(2);
      return;
    }''',
    '''    const bool valid = valid_stamp(msg.header.stamp);
    const double future_clock = valid ? now().seconds() : 0.;
    const bool future = valid && t > future_clock + 0.01;
    const double age_clock = valid && !future ? now().seconds() : 0.;
    const bool stale = valid && !future && age_clock - t > feedback_timeout_;
    if (!valid || future || stale) {
      trace_feedback_fault(
          !valid ? "stamp_format" : future ? "future_stamp" : "stale_stamp",
          t, future ? future_clock : age_clock);
      stop(2);
      return;
    }''')
replace_once(
    '''          w >= msg.velocity.size()) {
        stop(2);''',
    '''          w >= msg.velocity.size()) {
        trace_feedback_fault("joint_fields", t, now().seconds());
        stop(2);''')
replace_once(
    '''    if (!core::finite(next)) {
      stop(2);''',
    '''    if (!core::finite(next)) {
      trace_feedback_fault("nonfinite_joints", t, now().seconds());
      stop(2);''')
replace_once(
    '''      if (std::abs(a) > l_.steering + 0.03) {
        stop(2);''',
    '''      if (std::abs(a) > l_.steering + 0.03) {
        trace_feedback_fault("steering_range", t, now().seconds());
        stop(2);''')
replace_once(
    '''    if (!feedback_fresh()) {
      stop(2);''',
    '''    if (!feedback_fresh()) {
      trace_feedback_fault("tick_feedback_watchdog", feedback_stamp_, t);
      stop(2);''')
replace_once(
    '''      } catch (const std::exception &) {
        stop(2);''',
    '''      } catch (const std::exception & e) {
        trace_feedback_fault("generator_exception", feedback_stamp_, t, e.what());
        stop(2);''')
path.write_text(source)
