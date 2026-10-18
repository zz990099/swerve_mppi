#include "swerve_mppi_ros/nav2_controller.hpp"

#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <mutex>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <swerve_gazebo_sim/msg/chassis_command.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "swerve_mppi/controller.hpp"
#include "swerve_mppi_ros/conversions.hpp"
#include "swerve_mppi_ros/costmap_critic.hpp"
#include "swerve_mppi_ros/ordered_goal_checker.hpp"

namespace swerve_mppi_ros
{
using Steady = std::chrono::steady_clock;
struct Nav2Controller::Runtime
{
  std::mutex mutex;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::string name, global_frame, body_frame, plan_frame;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  Config config, baseline;
  Critics critics;
  std::unique_ptr<Controller> controller;
  ChassisAdapter adapter;
  PathHandler path;
  std::optional<ChassisFeedback> measured;
  double state_stamp = 0, last_time = 0, frequency = 20, timeout = 0.25, transform_tolerance = 0.1;
  Steady::time_point state_wall{};
  bool active = false, map_enabled = true;
  rclcpp_lifecycle::LifecyclePublisher<swerve_gazebo_sim::msg::ChassisCommand>::SharedPtr context;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr prediction;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics;
  rclcpp::Subscription<swerve_gazebo_sim::msg::ChassisState>::SharedPtr states;

  template <class T>
  T parameter(const std::string & key, T value)
  {
    const auto full = name + "." + key;
    if (!node->has_parameter(full)) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = true;
      node->declare_parameter<T>(full, value, descriptor);
    }
    return node->get_parameter(full).get_value<T>();
  }
  void parameters()
  {
    auto number = [&](const char * key, double & value) { value = parameter<double>(key, value); };
    number("wheelbase", config.wheelbase);
    number("track", config.track);
    number("wheel_radius", config.wheel_radius);
    number("max_speed", config.max_speed);
    number("max_yaw_rate", config.max_yaw_rate);
    number("min_turn_radius", config.min_turn_radius);
    number("linear_accel", config.linear_accel);
    number("angular_accel", config.angular_accel);
    number("max_wheel_speed", config.max_wheel_speed);
    number("wheel_accel", config.wheel_accel);
    number("steer_rate", config.steer_rate);
    number("steer_limit", config.steer_limit);
    number("realign_angle", config.realign_angle);
    number("alignment_tolerance", config.alignment_tolerance);
    number("alignment_dwell", config.alignment_dwell);
    number("stopped_wheel_speed", config.stopped_wheel_speed);
    number("model_dt", config.model_dt);
    number("integration_dt", config.integration_dt);
    number("noise_linear", config.noise_linear);
    number("noise_angular", config.noise_angular);
    number("noise_correlation", config.noise_correlation);
    number("temperature", config.temperature);
    number("minimum_mode_dwell", config.minimum_mode_dwell);
    number("switch_penalty", config.switch_penalty);
    number("switch_hysteresis", config.switch_hysteresis);
    number("lookahead", config.lookahead);
    number("approach_distance", config.approach_distance);
    number("approach_gain", config.approach_gain);
    auto size = [&](const char * key, std::size_t & value) {
      auto v = parameter<int>(key, static_cast<int>(value));
      if (v < 1) throw std::invalid_argument(key);
      value = static_cast<std::size_t>(v);
    };
    size("horizon", config.horizon);
    size("samples", config.samples);
    size("iterations", config.iterations);
    config.seed = static_cast<std::uint32_t>(parameter<int>("seed", 42));
    if (node->has_parameter("controller_frequency"))
      node->get_parameter("controller_frequency", frequency);
    number("state_timeout", timeout);
    number("transform_tolerance", transform_tolerance);
    if (
      !std::isfinite(frequency) || frequency < 1 || frequency > 100 || !std::isfinite(timeout) ||
      timeout <= 0 || !std::isfinite(transform_tolerance) || transform_tolerance < 0)
      throw std::invalid_argument("invalid timing parameters");
    TrackingWeights weights;
    number("path_distance_weight", weights.path_distance);
    number("path_heading_weight", weights.path_heading);
    number("progress_weight", weights.progress);
    number("goal_position_weight", weights.goal_position);
    number("goal_heading_weight", weights.goal_heading);
    number("effort_weight", weights.effort);
    number("smoothness_weight", weights.smoothness);
    number("steering_change_weight", weights.steering_change);
    critics = tracking_critics(weights);
    map_enabled = parameter<bool>("use_costmap", true);
    double weight = parameter<double>("costmap_weight", 5);
    bool unknown = parameter<bool>("unknown_is_obstacle", true);
    if (map_enabled) critics.push_back(std::make_shared<CostmapCritic>(weight, unknown));
    baseline = config;
    controller = std::make_unique<Controller>(config, critics);
  }
  geometry_msgs::msg::PoseStamped transform(
    geometry_msgs::msg::PoseStamped source, const std::string & frame)
  {
    if (source.header.frame_id.empty()) throw nav2_core::ControllerTFError("missing pose frame");
    if (source.header.frame_id == frame) return source;
    try {
      const auto t = tf->lookupTransform(
        frame, source.header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(transform_tolerance));
      geometry_msgs::msg::PoseStamped out;
      tf2::doTransform(source, out, t);
      return out;
    } catch (const tf2::TransformException & e) {
      throw nav2_core::ControllerTFError(e.what());
    }
  }
  WireCommand publish(Command command)
  {
    auto wire = adapter.translate(command);
    swerve_gazebo_sim::msg::ChassisCommand msg;
    msg.header.stamp = node->now();
    msg.header.frame_id = body_frame;
    msg.mode = static_cast<std::uint8_t>(wire.command.mode);
    msg.request_id = wire.request_id;
    msg.entry_velocity = twist(wire.entry);
    msg.velocity = twist(wire.command.velocity);
    // Context never drives the chassis. Only Nav2's final cmd_vel can do that.
    context->publish(msg);
    return wire;
  }
  void stop()
  {
    if (controller) controller->reset();
    last_time = 0;
    if (active && measured && context) publish({{}, adapter.state().mode});
  }
  void report(const Result & result)
  {
    if (prediction->get_subscription_count()) {
      nav_msgs::msg::Path msg;
      msg.header.stamp = node->now();
      msg.header.frame_id = global_frame;
      for (auto p : result.prediction) {
        geometry_msgs::msg::PoseStamped item;
        item.header = msg.header;
        item.pose = pose_message(p);
        msg.poses.push_back(item);
      }
      prediction->publish(msg);
    }
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = node->now();
    diagnostic_msgs::msg::DiagnosticStatus item;
    item.name = node->get_fully_qualified_name() + std::string("/") + name;
    item.hardware_id = "swerve";
    item.level = result.status == Status::NoSolution ? item.WARN : item.OK;
    item.message = result.status == Status::Switching ? "mode transition" : "tracking";
    for (const auto & p : std::vector<std::pair<std::string, std::string>>{
           {"compute_ms", std::to_string(result.compute_ms)},
           {"cost", std::to_string(result.cost)},
           {"evaluated", std::to_string(result.evaluated)}}) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = p.first;
      value.value = p.second;
      item.values.push_back(value);
    }
    msg.status.push_back(item);
    diagnostics->publish(msg);
  }
};
Nav2Controller::Nav2Controller() : runtime_(std::make_unique<Runtime>()) {}
Nav2Controller::~Nav2Controller() = default;
void Nav2Controller::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  auto & r = *runtime_;
  std::lock_guard<std::mutex> lock(r.mutex);
  r.node = parent.lock();
  if (!r.node || !tf || !costmap) throw std::invalid_argument("missing Nav2 resources");
  r.name = std::move(name);
  r.tf = std::move(tf);
  r.costmap = std::move(costmap);
  r.global_frame = r.costmap->getGlobalFrameID();
  r.body_frame = r.costmap->getBaseFrameID();
  r.parameters();
  r.context = r.node->create_publisher<swerve_gazebo_sim::msg::ChassisCommand>(
    r.parameter<std::string>("context_topic", "mppi/command_context"), 1);
  r.prediction = r.node->create_publisher<nav_msgs::msg::Path>(r.name + "/prediction", 1);
  r.diagnostics =
    r.node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(r.name + "/diagnostics", 1);
  r.states = r.node->create_subscription<swerve_gazebo_sim::msg::ChassisState>(
    r.parameter<std::string>("state_topic", "chassis_controller/state"), rclcpp::SensorDataQoS(),
    [&r](const swerve_gazebo_sim::msg::ChassisState & msg) {
      std::lock_guard<std::mutex> guard(r.mutex);
      if (msg.header.frame_id != r.body_frame) return;
      try {
        auto f = feedback(msg, r.config.wheel_radius);
        r.measured = f;
        r.adapter.observe(f);
        r.state_stamp = seconds(msg.header.stamp);
        r.state_wall = Steady::now();
      } catch (const std::invalid_argument &) {
      }
    });
}
void Nav2Controller::activate()
{
  auto & r = *runtime_;
  std::lock_guard<std::mutex> lock(r.mutex);
  r.context->on_activate();
  r.prediction->on_activate();
  r.diagnostics->on_activate();
  r.active = true;
  r.stop();
}
void Nav2Controller::deactivate()
{
  auto & r = *runtime_;
  std::lock_guard<std::mutex> lock(r.mutex);
  r.stop();
  r.active = false;
  r.context->on_deactivate();
  r.prediction->on_deactivate();
  r.diagnostics->on_deactivate();
}
void Nav2Controller::cleanup()
{
  auto & r = *runtime_;
  std::lock_guard<std::mutex> lock(r.mutex);
  r.stop();
  r.active = false;
  r.states.reset();
  r.context.reset();
  r.prediction.reset();
  r.diagnostics.reset();
  r.controller.reset();
  r.critics.clear();
  r.measured.reset();
  r.path = PathHandler{};
  r.adapter = ChassisAdapter{};
  r.tf.reset();
  r.costmap.reset();
  r.node.reset();
}
void Nav2Controller::setPlan(const nav_msgs::msg::Path & message)
{
  auto & r = *runtime_;
  std::lock_guard<std::mutex> lock(r.mutex);
  r.stop();
  r.path = PathHandler{};
  try {
    if (message.header.frame_id.empty()) throw std::invalid_argument("path frame is missing");
    std::vector<Pose> candidate;
    for (const auto & p : message.poses) {
      if (!p.header.frame_id.empty() && p.header.frame_id != message.header.frame_id)
        throw std::invalid_argument("mixed path frames");
      candidate.push_back(pose(p.pose));
    }
    r.path.set(std::move(candidate));
    r.plan_frame = message.header.frame_id;
  } catch (const std::exception & e) {
    throw nav2_core::InvalidPath(e.what());
  }
}
geometry_msgs::msg::TwistStamped Nav2Controller::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & current, const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  auto & r = *runtime_;
  std::lock_guard<std::mutex> lock(r.mutex);
  auto ordered = dynamic_cast<OrderedGoalChecker *>(goal_checker);
  try {
    if (!r.active || !r.controller) throw nav2_core::ControllerException("MPPI is inactive");
    if (r.path.empty()) throw nav2_core::InvalidPath("no path");
    const double now = r.node->now().seconds();
    const double dt = r.last_time > 0 ? now - r.last_time : 1 / r.frequency;
    r.last_time = now;
    if (dt <= 0 || dt > 1) throw nav2_core::NoValidControl("control clock discontinuity");
    if (
      !r.measured || now - r.state_stamp < -0.05 || now - r.state_stamp > r.timeout ||
      std::chrono::duration<double>(Steady::now() - r.state_wall).count() > r.timeout)
      throw nav2_core::NoValidControl("chassis state is stale or missing");
    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = r.node->now();
    out.header.frame_id = r.body_frame;
    if (r.measured->fault) {
      if (ordered) ordered->setReady(false);
      r.controller->reset();
      r.publish({{}, r.measured->state.mode});
      return out;
    }
    Input input;
    input.state = r.adapter.state();
    input.state.pose = pose(r.transform(current, r.global_frame).pose);
    input.state.velocity = velocity(speed);
    input.dt = dt;
    const auto local = r.path.local(pose(r.transform(current, r.plan_frame).pose));
    auto transform_path_pose = [&](Pose p) {
      geometry_msgs::msg::PoseStamped source;
      source.header.frame_id = r.plan_frame;
      source.pose = pose_message(p);
      return pose(r.transform(source, r.global_frame).pose);
    };
    for (auto p : local) input.path.push_back(transform_path_pose(p));
    input.goal = transform_path_pose(r.path.goal());
    input.goal_active = r.path.goal_active();
    std::vector<std::uint8_t> grid;
    if (r.map_enabled) {
      auto map = r.costmap->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
      const auto width = map->getSizeInCellsX(), height = map->getSizeInCellsY();
      const auto count = static_cast<std::size_t>(width) * height;
      if (!count) throw nav2_core::NoValidControl("empty local costmap");
      grid.assign(map->getCharMap(), map->getCharMap() + count);
      input.map = {grid.data(),      width, height, map->getResolution(), map->getOriginX(),
                   map->getOriginY()};
    }
    auto result = r.controller->compute(input);
    if (result.status != Status::Ok && result.status != Status::Switching)
      throw nav2_core::NoValidControl("MPPI found no usable control");
    auto wire = r.publish(result.command);
    out.twist = twist(wire.command.velocity);
    if (ordered) ordered->setReady(input.goal_active && !r.adapter.state().switching);
    r.report(result);
    return out;
  } catch (const nav2_core::ControllerException &) {
    if (ordered) ordered->setReady(false);
    r.stop();
    throw;
  } catch (const std::exception & e) {
    if (ordered) ordered->setReady(false);
    r.stop();
    throw nav2_core::NoValidControl(e.what());
  }
}
void Nav2Controller::setSpeedLimit(const double & limit, const bool & percentage)
{
  auto & r = *runtime_;
  std::lock_guard<std::mutex> lock(r.mutex);
  if (!std::isfinite(limit) || limit < 0 || (percentage && limit > 100))
    throw nav2_core::ControllerException("invalid Nav2 speed limit");
  r.config = r.baseline;
  if (limit != nav2_costmap_2d::NO_SPEED_LIMIT)
    r.config.max_speed =
      std::min(r.baseline.max_speed, percentage ? r.baseline.max_speed * limit / 100 : limit);
  if (r.controller) r.controller = std::make_unique<Controller>(r.config, r.critics);
}
bool Nav2Controller::cancel()
{
  reset();
  return true;
}
void Nav2Controller::reset()
{
  auto & r = *runtime_;
  std::lock_guard<std::mutex> lock(r.mutex);
  r.stop();
  r.path = PathHandler{};
}
}  // namespace swerve_mppi_ros
PLUGINLIB_EXPORT_CLASS(swerve_mppi_ros::Nav2Controller, nav2_core::Controller)
