#include "swerve_mppi_ros/chassis_bridge.hpp"

#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mutex>
#include <optional>
#include <swerve_gazebo_sim/msg/chassis_command.hpp>

#include "swerve_mppi_ros/conversions.hpp"

namespace swerve_mppi_ros
{
using Steady = std::chrono::steady_clock;
using Context = swerve_gazebo_sim::msg::ChassisCommand;
using Observation = swerve_gazebo_sim::msg::ChassisState;
struct ChassisBridge::Runtime
{
  explicit Runtime(ChassisBridge & n) : node(n) {}
  ChassisBridge & node;
  std::mutex mutex;
  std::string body_frame;
  double timeout = 0.25;
  std::optional<Context> context;
  std::optional<Observation> state;
  Velocity command;
  double command_stamp = 0;
  Steady::time_point context_wall{}, state_wall{}, command_wall{};
  rclcpp::Publisher<Context>::SharedPtr commands;
  rclcpp::Subscription<Context>::SharedPtr contexts;
  rclcpp::Subscription<Observation>::SharedPtr states;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr unstamped;
  rclcpp::TimerBase::SharedPtr timer;
  bool fresh(double stamp, Steady::time_point wall) const
  {
    const double age = node.now().seconds() - stamp;
    return age >= -0.05 && age <= timeout &&
           std::chrono::duration<double>(Steady::now() - wall).count() <= timeout;
  }
  void emit()
  {
    // With stale feedback let the chassis watchdog stop, without renewing a
    // command against an unknown request state.
    if (!state || !fresh(seconds(state->header.stamp), state_wall)) return;
    Context out;
    out.header.stamp = node.now();
    out.header.frame_id = body_frame;
    out.mode = state->requested_mode;
    out.request_id = state->request_id;
    out.entry_velocity = state->accepted_entry_velocity;
    const bool consistent =
      context &&
      ((context->request_id > state->request_id && (state->confirmed || state->fault)) ||
       (context->request_id == state->request_id && context->mode == state->requested_mode &&
        context->entry_velocity == state->accepted_entry_velocity));
    if (consistent && fresh(seconds(context->header.stamp), context_wall)) {
      out = *context;
      out.header.stamp = node.now();
      // New modes are always zero-speed requests. A stale/missing velocity or
      // a zero context also holds zero, including cancellation and deactivate.
      const auto allowed = velocity(context->velocity);
      const bool drive =
        state->confirmed && !state->fault && context->request_id == state->request_id &&
        context->mode == state->actual_mode &&
        context->entry_velocity == state->accepted_entry_velocity &&
        fresh(command_stamp, command_wall) && permitted(static_cast<Mode>(out.mode), command) &&
        (allowed.vx != 0 || allowed.vy != 0 || allowed.wz != 0);
      out.velocity = drive ? twist(command) : geometry_msgs::msg::Twist{};
    }
    commands->publish(out);
  }
};
ChassisBridge::ChassisBridge(const rclcpp::NodeOptions & options)
: Node("mppi_chassis_bridge", options), runtime_(std::make_unique<Runtime>(*this))
{
  auto & r = *runtime_;
  r.body_frame = declare_parameter<std::string>("body_frame", "base_footprint");
  r.timeout = declare_parameter<double>("command_timeout", 0.25);
  const double frequency = declare_parameter<double>("publish_frequency", 20);
  if (
    !std::isfinite(r.timeout) || r.timeout <= 0 || !std::isfinite(frequency) || frequency < 1 ||
    frequency > 100)
    throw std::invalid_argument("invalid bridge timing");
  r.commands = create_publisher<Context>(
    declare_parameter<std::string>("command_topic", "chassis_controller/command"), 1);
  r.contexts = create_subscription<Context>(
    declare_parameter<std::string>("context_topic", "mppi/command_context"), 1,
    [&r](const Context & msg) {
      std::lock_guard<std::mutex> lock(r.mutex);
      try {
        if (
          msg.header.frame_id != r.body_frame || msg.mode > 2 ||
          !permitted(static_cast<Mode>(msg.mode), velocity(msg.entry_velocity)) ||
          !permitted(static_cast<Mode>(msg.mode), velocity(msg.velocity))) {
          r.context.reset();
          return;
        }
        r.context = msg;
        r.context_wall = Steady::now();
        r.emit();
      } catch (const std::invalid_argument &) {
        r.context.reset();
      }
    });
  r.states = create_subscription<Observation>(
    declare_parameter<std::string>("state_topic", "chassis_controller/state"),
    rclcpp::SensorDataQoS(), [&r](const Observation & msg) {
      std::lock_guard<std::mutex> lock(r.mutex);
      try {
        if (msg.header.frame_id != r.body_frame) return;
        static_cast<void>(feedback(msg, 1));
        r.state = msg;
        r.state_wall = Steady::now();
      } catch (const std::invalid_argument &) {
        r.state.reset();
      }
    });
  const auto topic = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  if (declare_parameter<bool>("enable_stamped_cmd_vel", true)) {
    r.stamped = create_subscription<geometry_msgs::msg::TwistStamped>(
      topic, 10, [&r](const geometry_msgs::msg::TwistStamped & msg) {
        std::lock_guard<std::mutex> lock(r.mutex);
        try {
          if (!msg.header.frame_id.empty() && msg.header.frame_id != r.body_frame) return;
          r.command = velocity(msg.twist);
          r.command_stamp = seconds(msg.header.stamp);
          r.command_wall = Steady::now();
          r.emit();
        } catch (const std::invalid_argument &) {
          r.command = {};
          r.emit();
        }
      });
  } else {
    r.unstamped = create_subscription<geometry_msgs::msg::Twist>(
      topic, 10, [&r](const geometry_msgs::msg::Twist & msg) {
        std::lock_guard<std::mutex> lock(r.mutex);
        try {
          r.command = velocity(msg);
        } catch (const std::invalid_argument &) {
          r.command = {};
        }
        r.command_stamp = r.node.now().seconds();
        r.command_wall = Steady::now();
        r.emit();
      });
  }
  r.timer = create_wall_timer(std::chrono::duration<double>(1 / frequency), [&r]() {
    std::lock_guard<std::mutex> lock(r.mutex);
    r.emit();
  });
}
ChassisBridge::~ChassisBridge() = default;
}  // namespace swerve_mppi_ros
