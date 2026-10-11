#include "swerve_mppi_ros/controller_node.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <fstream>
#include <functional>
#include <limits>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <utility>

#include "swerve_mppi/common/config_profile.hpp"
#include "swerve_mppi/planning/controller.hpp"
#include "swerve_mppi_ros/msg/command_publication.hpp"
#include "swerve_mppi_ros/msg/obstacle_array.hpp"
#include "swerve_mppi_ros/observation_buffer.hpp"
#include "swerve_mppi_ros/srv/report_application.hpp"

namespace swerve_mppi_ros
{
namespace
{
using Ns = swerve_mppi::TimestampNs;
using Steady = std::chrono::steady_clock;
using Parameters = swerve_mppi::current_chassis::Parameters;
const std::vector<std::pair<std::string, double Parameters::*>> kSharedParameters = {
  {"wheelbase", &Parameters::wheelbase_m},
  {"track_width", &Parameters::track_m},
  {"wheel_radius", &Parameters::wheel_radius_m},
  {"update_rate", &Parameters::update_rate_hz},
  {"cmd_timeout", &Parameters::command_timeout_s},
  {"max_wheel_speed", &Parameters::max_wheel_speed_radps},
  {"max_wheel_acceleration", &Parameters::max_wheel_accel_radps2},
  {"max_steering_rate", &Parameters::max_steer_rate_radps},
  {"steering_limit", &Parameters::steering_limit_rad},
  {"max_linear_speed", &Parameters::max_linear_speed_mps},
  {"max_angular_speed", &Parameters::max_angular_speed_radps},
  {"max_linear_acceleration", &Parameters::max_linear_accel_mps2},
  {"max_angular_acceleration", &Parameters::max_angular_accel_radps2},
  {"drive_steering_limit", &Parameters::drive_steering_limit_rad},
  {"steering_alignment_tolerance", &Parameters::steering_tolerance_rad},
  {"steering_alignment_duration", &Parameters::alignment_duration_s},
  {"mode_switch_timeout", &Parameters::transition_timeout_s},
  {"stopped_wheel_speed", &Parameters::stopped_wheel_speed_radps}};

swerve_mppi::Config profile(const std::string & path)
{
  if (path.empty()) return {};
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::invalid_argument("cannot open planner_profile: " + path);
  std::string text(65537, '\0');
  input.read(text.data(), static_cast<std::streamsize>(text.size()));
  text.resize(static_cast<std::size_t>(input.gcount()));
  if (input.bad()) throw std::invalid_argument("cannot read planner_profile");
  return swerve_mppi::parse_config_profile(text);
}
}  // namespace

struct ControllerNode::Runtime : std::enable_shared_from_this<Runtime>
{
  explicit Runtime(ControllerNode & owner) : node(owner) {}
  ControllerNode & node;
  swerve_mppi::Config config;
  std::unique_ptr<swerve_mppi::Controller> controller;
  std::unique_ptr<swerve_mppi::current_chassis::Adapter> adapter;
  ObservationBuffer buffer;
  std::optional<swerve_mppi::VehicleState> vehicle;
  std::optional<swerve_mppi::CommandApplication> application;
  std::optional<msg::CommandPublication> pending;
  std::vector<swerve_mppi::Pose2d> path;
  std::vector<swerve_mppi::CircleObstacle> obstacles;
  std::string body_frame, odom_frame, odom_topic, chassis_namespace;
  std::string status = "waiting for chassis parameters";
  double wall_timeout = 0.25, obstacle_timeout = 0.3;
  Ns last_clock = 0, last_plan = -1, obstacle_stamp = -1;
  Steady::time_point clock_advance = Steady::now(), observation_wall = Steady::now();
  Steady::time_point obstacle_wall = Steady::now(), parameter_retry = Steady::now();
  uint64_t path_id = 0;
  bool active = false, started = false, fault = false, clock_ready = false;
  bool parameter_inflight = false, recovering_chassis = false;
  std::shared_ptr<std::atomic_bool> clock_jump = std::make_shared<std::atomic_bool>(false);
  rclcpp::JumpHandler::SharedPtr jump_handler;
  rclcpp::AsyncParametersClient::SharedPtr parameter_client;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Subscription<swerve_gazebo_sim::msg::ChassisState>::SharedPtr state_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
  rclcpp::Subscription<msg::ObstacleArray>::SharedPtr obstacle_sub;
  rclcpp_lifecycle::LifecyclePublisher<swerve_gazebo_sim::msg::ChassisCommand>::SharedPtr commands;
  rclcpp_lifecycle::LifecyclePublisher<msg::CommandPublication>::SharedPtr publications;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics;
  rclcpp::Service<srv::ReportApplication>::SharedPtr evidence_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_service;

  void latch(const std::string & reason)
  {
    if (!fault)
      RCLCPP_ERROR(node.get_logger(), "%s; lifecycle reconfiguration required", reason.c_str());
    fault = true;
    status = reason;
  }

  template <class Function>
  void guarded(Function function)
  {
    try {
      function();
    } catch (const std::exception & error) {
      latch(error.what());
    }
  }

  bool clock_valid()
  {
    if (clock_jump->load()) latch("simulation clock epoch changed");
    const auto now = node.now().nanoseconds();
    const bool backwards = now < last_clock;
    if (backwards) latch("simulation clock moved backwards");
    if (now > last_clock) {
      clock_ready = last_clock > 0;
      clock_advance = Steady::now();
    }
    last_clock = now;
    const bool valid =
      clock_ready && now > 0 &&
      std::chrono::duration<double>(Steady::now() - clock_advance).count() <= wall_timeout;
    if (active && !valid) latch("simulation clock stopped or is missing");
    return valid && !backwards && !clock_jump->load();
  }

  bool fresh_observation() const
  {
    if (!vehicle) return false;
    const auto now = node.now().nanoseconds();
    return now >= vehicle->stamp_ns &&
           now - vehicle->stamp_ns <=
             *swerve_mppi::duration_nanoseconds(config.max_observation_age_s) &&
           std::chrono::duration<double>(Steady::now() - observation_wall).count() <= wall_timeout;
  }

  bool fresh_context() const
  {
    const auto now = node.now().nanoseconds();
    return !path.empty() && obstacle_stamp >= 0 && now >= obstacle_stamp &&
           now - obstacle_stamp <= *swerve_mppi::duration_nanoseconds(obstacle_timeout) &&
           std::chrono::duration<double>(Steady::now() - obstacle_wall).count() <= obstacle_timeout;
  }

  bool stopped() const
  {
    return vehicle && std::abs(vehicle->velocity.vx) <= config.stopped_linear_mps &&
           std::abs(vehicle->velocity.vy) <= config.stopped_linear_mps &&
           std::abs(vehicle->velocity.wz) <= config.stopped_angular_radps &&
           std::all_of(
             vehicle->wheel_speeds.begin(), vehicle->wheel_speeds.end(),
             [this](double speed) { return std::abs(speed) <= config.stopped_wheel_speed_mps; });
  }

  void request_parameters()
  {
    if (
      adapter || parameter_inflight || Steady::now() < parameter_retry ||
      !parameter_client->service_is_ready())
      return;
    std::vector<std::string> names;
    for (const auto & item : kSharedParameters) names.push_back(item.first);
    for (const auto & name :
         {"frame_prefix", "odom_frame", "odom_child_frame", "publish_odom", "odom_topic"}) {
      names.emplace_back(name);
    }
    parameter_inflight = true;
    const auto weak = weak_from_this();
    parameter_client->get_parameters(
      names, [weak](std::shared_future<std::vector<rclcpp::Parameter>> future) {
        const auto self = weak.lock();
        if (!self) return;
        self->parameter_inflight = false;
        self->parameter_retry = Steady::now() + std::chrono::seconds(1);
        try {
          const auto values = future.get();
          if (values.size() != kSharedParameters.size() + 5)
            throw std::invalid_argument("missing chassis parameters");
          Parameters shared;
          for (size_t i = 0; i < kSharedParameters.size(); ++i) {
            shared.*(kSharedParameters[i].second) = values[i].as_double();
          }
          const auto offset = kSharedParameters.size();
          const auto prefix = values[offset].as_string();
          const auto odom = values[offset + 1].as_string();
          const auto body = values[offset + 2].as_string();
          if (
            prefix + "base_footprint" != self->body_frame ||
            (body.empty() ? prefix + "base_footprint" : body) != self->body_frame ||
            (odom.empty() ? prefix + "odom" : odom) != self->odom_frame ||
            !values[offset + 3].as_bool() ||
            rclcpp::expand_topic_or_service_name(
              values[offset + 4].as_string(), "swerve_controller", self->chassis_namespace) !=
              self->node.get_node_topics_interface()->resolve_topic_name(self->odom_topic)) {
            throw std::invalid_argument("chassis odometry/frame configuration mismatch");
          }
          self->adapter = std::make_unique<swerve_mppi::current_chassis::Adapter>(
            self->config, shared, self->body_frame, self->odom_frame);
          self->status = "parameters verified; waiting for clock, stopped chassis and task context";
        } catch (const std::exception & error) {
          self->status = error.what();
          RCLCPP_WARN(self->node.get_logger(), "%s", error.what());
        }
      });
  }

  void receive_path(const nav_msgs::msg::Path & msg)
  {
    if (
      msg.header.frame_id != odom_frame || msg.poses.empty() ||
      msg.poses.size() > config.max_path_points ||
      source_time(msg.header.stamp) > node.now().nanoseconds()) {
      path.clear();
      throw std::invalid_argument("invalid path frame, size or time");
    }
    std::vector<swerve_mppi::Pose2d> candidate;
    candidate.reserve(msg.poses.size());
    for (const auto & pose : msg.poses) {
      if (
        pose.header.frame_id != odom_frame ||
        source_time(pose.header.stamp) > node.now().nanoseconds()) {
        throw std::invalid_argument("invalid path pose frame or time");
      }
      candidate.push_back(planar_pose(pose.pose));
    }
    if (path_id == std::numeric_limits<uint64_t>::max())
      throw std::overflow_error("path identity exhausted");
    path = std::move(candidate);
    ++path_id;
  }

  void receive_obstacles(const msg::ObstacleArray & msg)
  {
    const auto stamp = source_time(msg.header.stamp);
    if (
      msg.header.frame_id != odom_frame || msg.obstacles.size() > config.max_obstacles ||
      stamp > node.now().nanoseconds() || stamp <= obstacle_stamp) {
      throw std::invalid_argument("invalid obstacle frame, size or source time");
    }
    std::vector<swerve_mppi::CircleObstacle> candidate;
    for (const auto & circle : msg.obstacles) {
      if (
        !std::isfinite(circle.x) || !std::isfinite(circle.y) || !std::isfinite(circle.radius) ||
        circle.radius < 0) {
        throw std::invalid_argument("invalid obstacle geometry");
      }
      candidate.push_back({circle.x, circle.y, circle.radius});
    }
    obstacles = std::move(candidate);
    obstacle_stamp = stamp;
    obstacle_wall = Steady::now();
  }

  void report(
    const srv::ReportApplication::Request & request, srv::ReportApplication::Response & response)
  {
    response.accepted = false;
    response.message = "application evidence rejected";
    try {
      if (
        !active || fault || !pending || !clock_valid() ||
        request.command_id != pending->command_id ||
        request.published_stamp != pending->published_stamp)
        return;
      const auto earliest = source_time(request.earliest_application);
      const auto latest = source_time(request.latest_application);
      if (latest > node.now().nanoseconds()) return;
      const auto result = adapter->record_application({request.command_id, earliest, latest});
      if (!result.application) return;
      application = result.application;
      pending.reset();
      response.accepted = true;
      response.message = "application window accepted; waiting for a subsequent source observation";
    } catch (const std::exception & error) {
      response.message = error.what();
    }
  }

  void recover_chassis(std_srvs::srv::Trigger::Response & response)
  {
    response.success = false;
    response.message =
      "requires active, faulted chassis with fresh observations and advancing clock";
    if (
      !active || !adapter || !vehicle || !clock_valid() || !fresh_observation() ||
      !vehicle->mode_fault)
      return;
    const auto command = adapter->make_recovery_command(node.now().nanoseconds());
    if (!command.message) return;
    latch("explicit chassis recovery; reconfigure after stopped confirmation");
    pending.reset();
    application.reset();
    recovering_chassis = true;
    commands->publish(convert(*command.message));
    response.success = true;
    response.message =
      "zero recovery request sent; immutable retries until confirmation, then reconfigure";
  }

  void diagnostic()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = node.now();
    diagnostic_msgs::msg::DiagnosticStatus item;
    item.name = node.get_fully_qualified_name();
    item.hardware_id = "current_chassis";
    item.level = fault ? item.ERROR : pending ? item.WARN : item.OK;
    item.message = status;
    array.status.push_back(item);
    diagnostics->publish(array);
  }

  void tick()
  {
    const bool clock = clock_valid();
    request_parameters();
    if (adapter && clock) {
      const auto pair =
        buffer.take(*swerve_mppi::duration_nanoseconds(config.observation_pairing_tolerance_s));
      if (pair) {
        const auto result =
          adapter->observe(convert(pair->first), convert(pair->second), node.now().nanoseconds());
        if (!result.state)
          throw std::invalid_argument(
            "chassis observation rejected: " + std::to_string(static_cast<int>(result.error)));
        vehicle = result.state;
        observation_wall = Steady::now();
      }
    }
    if (!active) return;
    if (!clock || !fresh_observation()) {
      if (clock) latch("chassis observation expired");
      recovering_chassis = false;
      diagnostic();
      return;
    }
    if (vehicle->mode_fault) latch("chassis fault");
    if (!fresh_context()) latch("path or obstacle context missing/expired");
    const auto now = node.now().nanoseconds();
    if (pending && now > source_time(pending->valid_until))
      latch("application evidence deadline expired");
    if (now - last_plan < *swerve_mppi::duration_nanoseconds(config.planning_period_s)) return;
    if (recovering_chassis) {
      last_plan = now;
      if (!adapter->recovery_pending()) {
        recovering_chassis = false;
        status = "chassis recovery confirmed; deactivate, cleanup and configure";
      } else {
        const auto command = adapter->make_recovery_command(now);
        if (command.message)
          commands->publish(convert(*command.message));
        else
          recovering_chassis = false;
      }
    }
    if (fault) {
      diagnostic();
      return;
    }
    if (pending) {
      status = "waiting for external application evidence";
      diagnostic();
      return;
    }
    if (
      vehicle->stamp_ns <= last_plan ||
      (application && vehicle->stamp_ns < application->latest_stamp_ns))
      return;
    swerve_mppi::ControllerInput input;
    input.vehicle = *vehicle;
    input.planning_stamp_ns = now;
    input.reference_path = path;
    input.obstacles = obstacles;
    input.path_id = path_id;
    input.previous_command_application = application;
    auto output = controller->compute(input);
    application.reset();
    last_plan = now;
    if (!output.command) {
      latch("core withheld command: " + std::to_string(static_cast<int>(output.failure_reason)));
      diagnostic();
      return;
    }
    // Observe the clock again after compute; a rollback or pause never authorizes a late publication.
    if (!clock_valid() || fault) return;
    const auto command = adapter->make_command(output, node.now().nanoseconds());
    if (!command.message) {
      latch("adapter withheld publication");
      diagnostic();
      return;
    }
    msg::CommandPublication publication;
    publication.command_id = output.command_id;
    publication.observation_stamp = ros_time(output.observation_stamp_ns);
    publication.computed_stamp = ros_time(output.computed_stamp_ns);
    publication.published_stamp = ros_time(output.published_stamp_ns);
    publication.valid_until = ros_time(output.valid_until_ns);
    publication.command = convert(*command.message);
    pending = publication;
    commands->publish(publication.command);
    publications->publish(publication);
    status = "command published; waiting for external application evidence";
    diagnostic();
  }

  void setup()
  {
    config = profile(node.get_parameter("planner_profile").as_string());
    swerve_mppi::validate_live_config(config);
    if (!node.get_parameter("use_sim_time").as_bool())
      throw std::invalid_argument("use_sim_time must be true");
    if (!node.get_node_options().use_clock_thread())
      throw std::invalid_argument("use_clock_thread must be true for live publication timestamps");
    rcl_jump_threshold_t threshold{};
    threshold.on_clock_change = true;
    threshold.min_backward.nanoseconds = -1;
    const auto jump_flag = clock_jump;
    jump_handler = node.get_clock()->create_jump_callback(
      nullptr,
      [jump_flag](const rcl_time_jump_t & jump) {
        if (jump.delta.nanoseconds < 0 || jump.clock_change == RCL_ROS_TIME_DEACTIVATED) {
          jump_flag->store(true);
        }
      },
      threshold);
    body_frame = node.get_parameter("body_frame").as_string();
    odom_frame = node.get_parameter("odom_frame").as_string();
    odom_topic = node.get_parameter("odometry_topic").as_string();
    const auto chassis_name = rclcpp::expand_topic_or_service_name(
      node.get_parameter("chassis_node").as_string(), node.get_name(), node.get_namespace());
    chassis_namespace = chassis_name.substr(0, chassis_name.find_last_of('/'));
    if (chassis_namespace.empty()) chassis_namespace = "/";
    wall_timeout = node.get_parameter("wall_timeout_s").as_double();
    obstacle_timeout = node.get_parameter("obstacle_timeout_s").as_double();
    if (
      !std::isfinite(wall_timeout) || wall_timeout <= 0 || !std::isfinite(obstacle_timeout) ||
      obstacle_timeout <= 0 || !swerve_mppi::duration_nanoseconds(obstacle_timeout)) {
      throw std::invalid_argument("invalid wrapper timeout");
    }
    controller = std::make_unique<swerve_mppi::Controller>(config);
    commands = node.create_publisher<swerve_gazebo_sim::msg::ChassisCommand>(
      node.get_parameter("command_topic").as_string(),
      rclcpp::QoS(1).reliable().durability_volatile());
    publications = node.create_publisher<msg::CommandPublication>(
      "~/command_publication", rclcpp::QoS(1).reliable());
    diagnostics = node.create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 1);
    state_sub = node.create_subscription<swerve_gazebo_sim::msg::ChassisState>(
      node.get_parameter("chassis_state_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const swerve_gazebo_sim::msg::ChassisState & message) {
        guarded([&] { buffer.push(message); });
      });
    odom_sub = node.create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry & message) { guarded([&] { buffer.push(message); }); });
    path_sub = node.create_subscription<nav_msgs::msg::Path>(
      node.get_parameter("path_topic").as_string(), rclcpp::QoS(1).reliable().durability_volatile(),
      [this](const nav_msgs::msg::Path & message) { guarded([&] { receive_path(message); }); });
    obstacle_sub = node.create_subscription<msg::ObstacleArray>(
      node.get_parameter("obstacles_topic").as_string(), 1,
      [this](const msg::ObstacleArray & message) { guarded([&] { receive_obstacles(message); }); });
    evidence_service = node.create_service<srv::ReportApplication>(
      "~/report_application", [this](
                                std::shared_ptr<srv::ReportApplication::Request> request,
                                std::shared_ptr<srv::ReportApplication::Response> response) {
        report(*request, *response);
      });
    recovery_service = node.create_service<std_srvs::srv::Trigger>(
      "~/recover_chassis", [this](
                             std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        recover_chassis(*response);
      });
    parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(
      node.get_node_base_interface(), node.get_node_topics_interface(),
      node.get_node_graph_interface(), node.get_node_services_interface(),
      node.get_parameter("chassis_node").as_string());
    timer = node.create_wall_timer(
      std::chrono::milliseconds(10), [this] { guarded([this] { tick(); }); });
  }
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: LifecycleNode("mppi_controller", options)
{
  rcl_interfaces::msg::ParameterDescriptor startup;
  startup.read_only = true;
  for (const auto & item : std::vector<std::pair<std::string, std::string>>{
         {"planner_profile", ""},
         {"chassis_node", "swerve_controller"},
         {"body_frame", "base_footprint"},
         {"odom_frame", "odom"},
         {"odometry_topic", "odom"},
         {"chassis_state_topic", "chassis_controller/state"},
         {"command_topic", "chassis_controller/command"},
         {"path_topic", "path"},
         {"obstacles_topic", "obstacles"}}) {
    declare_parameter(item.first, item.second, startup);
  }
  declare_parameter("wall_timeout_s", 0.25, startup);
  declare_parameter("obstacle_timeout_s", 0.3, startup);
  parameter_guard_ =
    add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & parameter : parameters) {
        if (runtime_ && parameter.get_name() == "use_sim_time" && !parameter.as_bool()) {
          result.successful = false;
          result.reason = "simulation clock domain cannot change while configured";
        }
      }
      return result;
    });
}
ControllerNode::~ControllerNode() = default;

ControllerNode::CallbackReturn ControllerNode::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    auto runtime = std::make_shared<Runtime>(*this);
    runtime->setup();
    runtime_ = std::move(runtime);
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "configuration failed: %s", error.what());
    return CallbackReturn::FAILURE;
  }
}
ControllerNode::CallbackReturn ControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
  if (
    !runtime_ || runtime_->started || runtime_->fault || !runtime_->adapter ||
    !runtime_->clock_valid() || !runtime_->fresh_observation() || !runtime_->fresh_context() ||
    !runtime_->stopped() || runtime_->vehicle->mode_fault || !runtime_->vehicle->mode_confirmed) {
    RCLCPP_WARN(
      get_logger(), "activation refused: %s", runtime_ ? runtime_->status.c_str() : "unconfigured");
    return CallbackReturn::FAILURE;
  }
  runtime_->commands->on_activate();
  runtime_->publications->on_activate();
  runtime_->diagnostics->on_activate();
  runtime_->active = runtime_->started = true;
  return CallbackReturn::SUCCESS;
}
ControllerNode::CallbackReturn ControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (runtime_) {
    runtime_->active = false;
    runtime_->recovering_chassis = false;
    runtime_->commands->on_deactivate();
    runtime_->publications->on_deactivate();
    runtime_->diagnostics->on_deactivate();
  }
  return CallbackReturn::SUCCESS;
}
ControllerNode::CallbackReturn ControllerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  runtime_.reset();
  return CallbackReturn::SUCCESS;
}
ControllerNode::CallbackReturn ControllerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  runtime_.reset();
  return CallbackReturn::SUCCESS;
}
}  // namespace swerve_mppi_ros
