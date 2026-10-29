#include "swerve_mppi_ros/nav2_controller.hpp"

#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <mutex>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>
#include <pluginlib/class_list_macros.hpp>
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
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node() const
  {
    auto owner = parent.lock();
    if (!owner) throw nav2_core::ControllerException("Nav2 parent node expired");
    return owner;
  }
  std::string name, global_frame, body_frame, plan_frame;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  Config config, baseline;
  Critics critics;
  std::unique_ptr<swerve_mppi::Controller> controller;
  PathHandler path;
  std::optional<ChassisFeedback> measured;
  std::optional<Velocity> published_request;
  double last_output_time = 0;
  double state_stamp = 0, last_time = 0, frequency = 20, timeout = 0.25, transform_tolerance = 0.1;
  Steady::time_point state_wall{};
  bool active = false, map_enabled = true;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr prediction;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics;
  rclcpp::Subscription<swerve_chassis_msgs::msg::ChassisState>::SharedPtr states;

  template <class T>
  T parameter(const std::string & key, T value)
  {
    const auto full = name + "." + key;
    if (!node()->has_parameter(full)) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.read_only = true;
      node()->declare_parameter<T>(full, value, descriptor);
    }
    return node()->get_parameter(full).get_value<T>();
  }
  void parameters()
  {
    auto number = [&](const char * key, double & value) { value = parameter<double>(key, value); };
    number("wheelbase", config.geometry.wheelbase);
    number("track_width", config.geometry.track_width);
    number("wheel_radius", config.geometry.radius);
    number("max_speed", config.max_speed);
    number("max_yaw_rate", config.max_yaw_rate);
    number("physical_linear_speed", config.limits.linear_speed);
    number("physical_angular_speed", config.limits.angular_speed);
    number("linear_accel", config.limits.linear_acceleration);
    number("angular_accel", config.limits.angular_acceleration);
    number("max_wheel_speed", config.limits.wheel_speed);
    number("wheel_accel", config.limits.wheel_acceleration);
    number("steer_rate", config.limits.steering_rate);
    number("steer_limit", config.limits.steering);
    number("steering_tracking_tolerance", config.limits.tracking_tolerance);
    number("stopped_wheel_speed", config.limits.stopped_speed);
    number("max_residual", config.max_residual);
    number("position_scale", config.position_scale);
    number("heading_scale", config.heading_scale);
    number("path_corridor", config.path_corridor);
    number("heading_corridor", config.heading_corridor);
    number("cross_gain", config.cross_gain);
    number("model_dt", config.model_dt);
    number("integration_dt", config.integration_dt);
    number("steering_time_constant", config.steering_time_constant);
    number("noise_linear", config.noise_linear);
    number("noise_angular", config.noise_angular);
    number("noise_correlation", config.noise_correlation);
    number("temperature", config.temperature);
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
    if (node()->has_parameter("controller_frequency"))
      node()->get_parameter("controller_frequency", frequency);
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
    number("consistency_weight", weights.consistency);
    number("reference_velocity_weight", weights.reference_velocity);
    number("presteering_alignment_weight", weights.presteering_alignment);
    critics = tracking_critics(weights);
    map_enabled = parameter<bool>("use_costmap", true);
    double weight = parameter<double>("costmap_weight", 5);
    bool unknown = parameter<bool>("unknown_is_obstacle", true);
    if (map_enabled) {
      double radius = 0;
      for (const auto & point : costmap->getRobotFootprint())
        radius = std::max(radius, std::hypot(point.x, point.y));
      if (radius <= 0) throw std::invalid_argument("nonempty robot footprint required");
      critics.push_back(std::make_shared<CostmapCritic>(weight, unknown, radius));
    }
    baseline = config;
    controller = std::make_unique<swerve_mppi::Controller>(config, critics);
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
  void stop()
  {
    if (controller) controller->reset();
    published_request.reset();
    last_time = 0;
    last_output_time = 0;
  }
  void report(const Result & result)
  {
    if (prediction->get_subscription_count()) {
      nav_msgs::msg::Path msg;
      msg.header.stamp = node()->now();
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
    msg.header.stamp = node()->now();
    diagnostic_msgs::msg::DiagnosticStatus item;
    item.name = node()->get_fully_qualified_name() + std::string("/") + name;
    item.hardware_id = "swerve";
    item.level = result.status == Status::Ok ? item.OK : item.WARN;
    item.message = "continuous tracking";
    for (const auto & p : std::vector<std::pair<std::string, std::string>>{
           {"compute_ms", std::to_string(result.compute_ms)},
           {"cost", std::to_string(result.cost)},
           {"evaluated", std::to_string(result.evaluated)},
           {"invalid_state", std::to_string(result.diagnostics.invalid_state)},
           {"invalid_input", std::to_string(result.diagnostics.invalid_input)},
           {"invalid_control", std::to_string(result.diagnostics.invalid_control)},
           {"actuator_residual", std::to_string(result.diagnostics.actuator_residual)},
           {"generator_error", std::to_string(result.diagnostics.generator_error)},
           {"critic_rejected", std::to_string(result.diagnostics.critic_rejected)},
           {"collision_rejected", std::to_string(result.diagnostics.collision_rejected)},
           {"critic_errors", std::to_string(result.diagnostics.critic_errors)},
           {"controller_errors", std::to_string(result.diagnostics.controller_errors)},
           {"braking_selected", std::to_string(result.braking_selected)},
           {"presteering", std::to_string(result.presteering)},
           {"reference_speed", std::to_string(result.reference_speed)},
           {"effective_samples", std::to_string(result.effective_samples)}}) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = p.first;
      value.value = p.second;
      item.values.push_back(value);
    }
    auto add = [&](const std::string & key, double number) {
      diagnostic_msgs::msg::KeyValue value;
      value.key = key;
      value.value = std::to_string(number);
      item.values.push_back(value);
    };
    add("noise_scale", result.noise_scale);
    add("reorientation_active", result.reorientation_active);
    add("reference_limit", result.reference_limit);
    add("branch_distance", result.branch_distance);
    add("route_section", path.section());
    add("route_rotating", path.rotating());
    add("remaining_path_m", path.remaining());
    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
      const std::string prefix =
        std::array<std::string, 5>{"mean", "fresh", "brake", "recovery", "selected"}[i] + "_";
      const auto & d = result.candidates[i];
      add(prefix + "valid", d.valid);
      add(prefix + "cost", d.total);
      add(prefix + "path", d.tracking.path);
      add(prefix + "heading", d.tracking.heading);
      add(prefix + "velocity", d.tracking.velocity);
      add(prefix + "alignment", d.tracking.alignment);
      add(prefix + "effort", d.tracking.effort);
      add(prefix + "progress", d.tracking.progress);
      add(prefix + "goal", d.tracking.goal);
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
  r.parent = parent;
  if (parent.expired() || !tf || !costmap) throw std::invalid_argument("missing Nav2 resources");
  r.name = std::move(name);
  r.tf = std::move(tf);
  r.costmap = std::move(costmap);
  r.global_frame = r.costmap->getGlobalFrameID();
  r.body_frame = r.costmap->getBaseFrameID();
  r.parameters();
  r.prediction = r.node()->create_publisher<nav_msgs::msg::Path>(r.name + "/prediction", 1);
  r.diagnostics =
    r.node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(r.name + "/diagnostics", 1);
  r.states = r.node()->create_subscription<swerve_chassis_msgs::msg::ChassisState>(
    r.parameter<std::string>("state_topic", "chassis_controller/state"), rclcpp::SensorDataQoS(),
    [&r](const swerve_chassis_msgs::msg::ChassisState & msg) {
      std::lock_guard<std::mutex> guard(r.mutex);
      if (msg.header.frame_id != r.body_frame) return;
      try {
        auto f = feedback(msg, r.config);
        const double stamp = seconds(msg.header.stamp);
        if (r.node()->now().seconds() < r.state_stamp) {
          r.measured.reset();
          r.stop();
        }
        if (stamp > r.node()->now().seconds() + 0.01) {
          r.measured.reset();
          return;
        }
        if (r.measured && stamp <= r.state_stamp) return;
        r.measured = f;
        r.state_stamp = seconds(msg.header.stamp);
        r.state_wall = Steady::now();
      } catch (const std::invalid_argument &) {
        r.measured.reset();
      }
    });
}
void Nav2Controller::activate()
{
  auto & r = *runtime_;
  std::lock_guard<std::mutex> lock(r.mutex);
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
  r.prediction.reset();
  r.diagnostics.reset();
  r.controller.reset();
  r.critics.clear();
  r.measured.reset();
  r.path = PathHandler{};
  r.tf.reset();
  r.costmap.reset();
  r.parent.reset();
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
    const auto control_stamp = r.node()->now();
    const double now = control_stamp.seconds();
    const double dt = r.last_time > 0 ? now - r.last_time : 1 / r.frequency;
    r.last_time = now;
    if (dt <= 0 || dt > 1) throw nav2_core::NoValidControl("control clock discontinuity");
    if (
      !r.measured || !r.measured->feedback_valid || now - r.measured->feedback_stamp < -0.01 ||
      now - r.measured->feedback_stamp > r.timeout || now - r.state_stamp < -0.05 ||
      now - r.state_stamp > r.timeout ||
      std::chrono::duration<double>(Steady::now() - r.state_wall).count() > r.timeout)
      throw nav2_core::NoValidControl("chassis state is stale or missing");
    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = control_stamp;
    out.header.frame_id = r.body_frame;
    if (r.measured->fault) {
      if (ordered) ordered->setReady(false);
      r.controller->reset();
      r.published_request.reset();
      return out;
    }
    Input input;
    input.state = r.measured->state;
    // Feedback may still describe an older accepted command. Slew the next
    // published request from our actual previous output, while retaining the
    // measured generator snapshot as the actuator prediction initial state.
    if (r.published_request) input.state.requested = *r.published_request;
    input.state.pose = pose(r.transform(current, r.global_frame).pose);
    (void)velocity(speed);  // Validate Nav2 velocity; wheel feedback anchors actuator prediction.
    input.dt = dt;
    const auto previous_section = r.path.section();
    const auto local =
      r.path.local(pose(r.transform(current, r.plan_frame).pose), 3.0, input.state.velocity);
    if (r.path.section() != previous_section) r.controller->reset();
    auto transform_path_pose = [&](Pose p) {
      geometry_msgs::msg::PoseStamped source;
      source.header.frame_id = r.plan_frame;
      source.pose = pose_message(p);
      return pose(r.transform(source, r.global_frame).pose);
    };
    for (auto p : local) input.path.push_back(transform_path_pose(p));
    input.goal = transform_path_pose(r.path.tracking_goal());
    input.goal_active = r.path.tracking_goal_active(r.config.approach_distance);
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
    if (r.last_output_time > 0 && result.status == Status::Ok) {
      // Nav2 stamps its publication after this callback returns. Reserve one
      // simulation millisecond for that handoff; a later current publication
      // only increases the available interval.
      const double available =
        std::max(1e-6, r.node()->now().seconds() - r.last_output_time - .001);
      if (available < input.dt && !r.controller->retime(input, available, result))
        throw nav2_core::NoValidControl("retimed MPPI request failed validation");
    }
    r.report(result);
    if (result.status != Status::Ok)
      throw nav2_core::NoValidControl("MPPI found no usable control");
    out.twist = twist(result.command.velocity);
    r.published_request = result.command.velocity;
    r.last_output_time = r.node()->now().seconds();
    if (ordered) ordered->setReady(r.path.goal_active(r.config.approach_distance));
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
  if (r.controller) r.controller = std::make_unique<swerve_mppi::Controller>(r.config, r.critics);
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
