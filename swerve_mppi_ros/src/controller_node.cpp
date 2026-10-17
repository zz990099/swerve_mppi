#include "swerve_mppi_ros/controller_node.hpp"

#include <chrono>
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <limits>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <swerve_gazebo_sim/msg/chassis_command.hpp>
#include <swerve_gazebo_sim/msg/chassis_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "swerve_mppi/controller.hpp"
#include "swerve_mppi_ros/control_support.hpp"
#include "swerve_mppi_ros/costmap_critic.hpp"

namespace swerve_mppi_ros
{
namespace
{
using Steady = std::chrono::steady_clock;
double seconds(const builtin_interfaces::msg::Time & t) { return t.sec + t.nanosec * 1e-9; }
Velocity velocity(const geometry_msgs::msg::Twist & v)
{
  return {v.linear.x, v.linear.y, v.angular.z};
}
geometry_msgs::msg::Twist twist(Velocity v)
{
  geometry_msgs::msg::Twist out;
  out.linear.x = v.vx;
  out.linear.y = v.vy;
  out.angular.z = v.wz;
  return out;
}
Pose pose(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (
    !std::isfinite(p.position.x) || !std::isfinite(p.position.y) || !std::isfinite(norm) ||
    std::abs(norm - 1) > 0.01 || std::abs(q.x) > 0.01 || std::abs(q.y) > 0.01)
    throw std::invalid_argument("invalid planar pose");
  return {p.position.x, p.position.y, std::atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)};
}
}  // namespace
struct ControllerNode::Runtime
{
  explicit Runtime(ControllerNode & n) : node(n) {}
  ControllerNode & node;
  Config config;
  std::unique_ptr<Controller> controller;
  ChassisAdapter adapter;
  PathHandler path;
  GoalChecker goal;
  std::optional<ChassisFeedback> feedback;
  std::optional<nav_msgs::msg::Odometry> odom;
  double state_stamp = 0, odom_stamp = 0, last_time = 0, frequency = 20, timeout = 0.25;
  Steady::time_point state_wall{}, odom_wall{};
  bool complete = false, map_enabled = false;
  std::string odom_frame, body_frame;
  std::vector<std::uint8_t> grid;
  GridView map;
  rclcpp::Publisher<swerve_gazebo_sim::msg::ChassisCommand>::SharedPtr commands;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr prediction;
  rclcpp::Subscription<swerve_gazebo_sim::msg::ChassisState>::SharedPtr states;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr paths;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr maps;
  rclcpp::TimerBase::SharedPtr timer;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<tf2_ros::TransformListener> listener;
  void status(const std::string & message, const Result * result = nullptr)
  {
    diagnostic_msgs::msg::DiagnosticArray out;
    out.header.stamp = node.now();
    diagnostic_msgs::msg::DiagnosticStatus item;
    item.name = node.get_fully_qualified_name();
    item.hardware_id = "swerve";
    item.message = message;
    item.level = result && result->status == Status::NoSolution ? item.WARN : item.OK;
    if (result)
      for (const auto & pair : std::vector<std::pair<std::string, std::string>>{
             {"compute_ms", std::to_string(result->compute_ms)},
             {"cost", std::to_string(result->cost)},
             {"evaluated", std::to_string(result->evaluated)},
             {"mode", std::to_string(static_cast<int>(result->command.mode))}}) {
        diagnostic_msgs::msg::KeyValue value;
        value.key = pair.first;
        value.value = pair.second;
        item.values.push_back(value);
      }
    out.status.push_back(item);
    diagnostics->publish(out);
  }
  void publish(Command command)
  {
    const auto wire = adapter.translate(command);
    swerve_gazebo_sim::msg::ChassisCommand msg;
    msg.header.stamp = node.now();
    msg.header.frame_id = body_frame;
    msg.mode = static_cast<std::uint8_t>(wire.command.mode);
    msg.velocity = twist(wire.command.velocity);
    msg.request_id = wire.request_id;
    msg.entry_velocity = twist(wire.entry);
    commands->publish(msg);
  }
  void task(const nav_msgs::msg::Path & message)
  {
    try {
      if (message.poses.empty() || message.poses.size() > 4096 || message.header.frame_id.empty())
        throw std::invalid_argument("path is empty, oversized or missing a frame");
      std::vector<Pose> candidate;
      for (const auto & entry : message.poses) {
        auto source = entry;
        if (source.header.frame_id.empty()) source.header = message.header;
        if (source.header.frame_id != message.header.frame_id)
          throw std::invalid_argument("mixed path frames");
        if (source.header.frame_id != odom_frame) {
          geometry_msgs::msg::PoseStamped transformed;
          tf2::doTransform(
            source, transformed,
            tf->lookupTransform(odom_frame, source.header.frame_id, tf2::TimePointZero));
          source = transformed;
        }
        candidate.push_back(pose(source.pose));
      }
      path.set(std::move(candidate));
      goal.reset();
      controller->reset();
      complete = false;
      std_msgs::msg::Bool value;
      value.data = false;
      reached->publish(value);
      status("tracking new path");
    } catch (const std::exception & e) {
      path = PathHandler{};
      controller->reset();
      status(e.what());
    }
  }
  void map_message(const nav_msgs::msg::OccupancyGrid & msg)
  {
    try {
      auto origin = pose(msg.info.origin);
      if (
        msg.header.frame_id != odom_frame || std::abs(origin.yaw) > 1e-6 || !msg.info.width ||
        !msg.info.height ||
        static_cast<std::uint64_t>(msg.info.width) * msg.info.height != msg.data.size() ||
        !std::isfinite(msg.info.resolution) || msg.info.resolution <= 0)
        throw std::invalid_argument("invalid costmap geometry or frame");
      grid.clear();
      grid.reserve(msg.data.size());
      for (auto value : msg.data) {
        if (value < -1 || value > 100) throw std::invalid_argument("invalid occupancy cost");
        grid.push_back(
          value < 0 ? 255 : static_cast<std::uint8_t>(std::lround(value * 254.0 / 100)));
      }
      map = {grid.data(), msg.info.width, msg.info.height, msg.info.resolution, origin.x, origin.y};
    } catch (const std::exception & e) {
      grid.clear();
      map = {};
      status(e.what());
    }
  }
  void tick()
  {
    const double now = node.now().seconds(), dt = last_time > 0 ? now - last_time : 1 / frequency;
    last_time = now;
    if (now <= 0 || dt <= 0 || dt > 1) {
      controller->reset();
      goal.reset();
      return;
    }
    const auto wall = Steady::now();
    bool fresh = feedback && odom && now - state_stamp >= -0.05 && now - odom_stamp >= -0.05 &&
                 now - state_stamp <= timeout && now - odom_stamp <= timeout &&
                 std::chrono::duration<double>(wall - state_wall).count() <= timeout &&
                 std::chrono::duration<double>(wall - odom_wall).count() <= timeout;
    if (!fresh) {
      controller->reset();
      goal.reset();
      status("waiting for fresh state and odometry");
      return;
    }
    if (feedback->fault) {
      publish({{}, feedback->state.mode});
      controller->reset();
      goal.reset();
      status("zero request to initialize faulted chassis");
      return;
    }
    if (path.empty() || complete) {
      publish({{}, feedback->state.mode});
      return;
    }
    Input input;
    input.state = adapter.state();
    input.state.pose = pose(odom->pose.pose);
    input.state.velocity = velocity(odom->twist.twist);
    input.dt = dt;
    input.path = path.local(input.state.pose);
    input.goal = path.goal();
    input.map = map;
    if (goal.reached(input.state, input.goal, odom_stamp)) {
      complete = true;
      publish({{}, input.state.mode});
      std_msgs::msg::Bool value;
      value.data = true;
      reached->publish(value);
      status("goal reached");
      return;
    }
    auto output = controller->compute(input);
    publish(
      output.status == Status::Ok || output.status == Status::Switching
        ? output.command
        : Command{{}, input.state.mode});
    status(
      output.status == Status::Ok          ? "tracking"
      : output.status == Status::Switching ? "mode transition"
                                           : "no usable control",
      &output);
    if (prediction->get_subscription_count()) {
      nav_msgs::msg::Path p;
      p.header.stamp = node.now();
      p.header.frame_id = odom_frame;
      for (const auto & v : output.prediction) {
        geometry_msgs::msg::PoseStamped point;
        point.header = p.header;
        point.pose.position.x = v.x;
        point.pose.position.y = v.y;
        point.pose.orientation.z = std::sin(v.yaw / 2);
        point.pose.orientation.w = std::cos(v.yaw / 2);
        p.poses.push_back(point);
      }
      prediction->publish(p);
    }
  }
  template <class T>
  T parameter(const std::string & name, T value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return node.declare_parameter<T>(name, value, descriptor);
  }
  void setup()
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
    frequency = parameter<double>("control_frequency", 20);
    timeout = parameter<double>("state_timeout", 0.25);
    if (
      !std::isfinite(frequency) || frequency < 1 || frequency > 100 || !std::isfinite(timeout) ||
      timeout <= 0)
      throw std::invalid_argument("invalid frequency/timeout");
    odom_frame = parameter<std::string>("odom_frame", "odom");
    body_frame = parameter<std::string>("body_frame", "base_footprint");
    GoalTolerance t;
    number("goal_position_tolerance", t.position);
    number("goal_yaw_tolerance", t.yaw);
    number("goal_linear_speed", t.linear_speed);
    number("goal_angular_speed", t.angular_speed);
    number("goal_settle_time", t.settle);
    goal = GoalChecker(t);
    TrackingWeights weights;
    number("path_distance_weight", weights.path_distance);
    number("path_heading_weight", weights.path_heading);
    number("progress_weight", weights.progress);
    number("goal_position_weight", weights.goal_position);
    number("goal_heading_weight", weights.goal_heading);
    number("effort_weight", weights.effort);
    number("smoothness_weight", weights.smoothness);
    number("steering_change_weight", weights.steering_change);
    auto critics = tracking_critics(weights);
    map_enabled = parameter<bool>("use_costmap", false);
    double weight = parameter<double>("costmap_weight", 5);
    bool unknown = parameter<bool>("unknown_is_obstacle", true);
    if (map_enabled) critics.push_back(std::make_shared<CostmapCritic>(weight, unknown));
    controller = std::make_unique<Controller>(config, std::move(critics));
    commands = node.create_publisher<swerve_gazebo_sim::msg::ChassisCommand>(
      parameter<std::string>("command_topic", "chassis_controller/command"), 1);
    reached = node.create_publisher<std_msgs::msg::Bool>(
      "~/goal_reached", rclcpp::QoS(1).transient_local());
    diagnostics = node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 1);
    prediction = node.create_publisher<nav_msgs::msg::Path>("~/prediction", 1);
    tf = std::make_unique<tf2_ros::Buffer>(node.get_clock());
    listener = std::make_shared<tf2_ros::TransformListener>(*tf, &node, false);
    paths = node.create_subscription<nav_msgs::msg::Path>(
      parameter<std::string>("path_topic", "path"), rclcpp::QoS(1),
      [this](const nav_msgs::msg::Path & msg) { task(msg); });
    maps = node.create_subscription<nav_msgs::msg::OccupancyGrid>(
      parameter<std::string>("costmap_topic", "costmap"), rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid & msg) { map_message(msg); });
    odometry = node.create_subscription<nav_msgs::msg::Odometry>(
      parameter<std::string>("odometry_topic", "odom"), 10,
      [this](const nav_msgs::msg::Odometry & msg) {
        if (msg.header.frame_id != odom_frame || msg.child_frame_id != body_frame) return;
        try {
          static_cast<void>(pose(msg.pose.pose));
        } catch (const std::exception &) {
          return;
        }
        odom = msg;
        odom_stamp = seconds(msg.header.stamp);
        odom_wall = Steady::now();
      });
    states = node.create_subscription<swerve_gazebo_sim::msg::ChassisState>(
      parameter<std::string>("state_topic", "chassis_controller/state"), rclcpp::SensorDataQoS(),
      [this](const swerve_gazebo_sim::msg::ChassisState & msg) {
        if (msg.header.frame_id != body_frame || msg.actual_mode > 2 || msg.requested_mode > 2)
          return;
        ChassisFeedback f;
        f.request_id = msg.request_id;
        f.entry = velocity(msg.accepted_entry_velocity);
        f.fault = msg.fault != 0;
        f.state.mode = static_cast<Mode>(msg.actual_mode);
        f.state.requested_mode = static_cast<Mode>(msg.requested_mode);
        f.state.switching = !msg.confirmed;
        f.state.steering = msg.steering_angles;
        for (std::size_t i = 0; i < 4; ++i)
          f.state.wheel_speeds[i] = msg.wheel_speeds[i] * config.wheel_radius;
        feedback = f;
        adapter.observe(f);
        state_stamp = seconds(msg.header.stamp);
        state_wall = Steady::now();
      });
    timer = node.create_wall_timer(std::chrono::duration<double>(1 / frequency), [this]() {
      try {
        tick();
      } catch (const std::exception & e) {
        controller->reset();
        status(e.what());
      }
    });
  }
};
ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: Node("mppi_controller", options), runtime_(std::make_unique<Runtime>(*this))
{
  runtime_->setup();
}
ControllerNode::~ControllerNode() = default;
}  // namespace swerve_mppi_ros
