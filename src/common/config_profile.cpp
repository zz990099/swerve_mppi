#include "swerve_mppi/common/config_profile.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace swerve_mppi
{
namespace
{
struct Parameter
{
  ConfigParameterInfo info;
  ConfigValue (*read)(const Config &);
  void (*write)(Config &, const ConfigValue &);
};
template <auto Member>
Parameter parameter(std::string_view name, std::string_view unit, ConfigScope scope)
{
  using T = std::decay_t<decltype(Config{}.*Member)>;
  constexpr bool real = std::is_same_v<T, double>;
  return {
    {name, unit, real ? ConfigValueType::Real : ConfigValueType::UnsignedInteger, scope},
    [](const Config & config) -> ConfigValue {
      if constexpr (real) {
        return config.*Member;
      } else {
        return static_cast<std::uint64_t>(config.*Member);
      }
    },
    [](Config & config, const ConfigValue & value) {
      if constexpr (real) {
        const auto * number = std::get_if<double>(&value);
        if (!number || !std::isfinite(*number)) {
          throw std::invalid_argument("finite real value required");
        }
        config.*Member = *number;
      } else {
        const auto * number = std::get_if<std::uint64_t>(&value);
        if (!number || *number > std::numeric_limits<T>::max()) {
          throw std::invalid_argument("unsigned integer out of range or wrong numeric type");
        }
        config.*Member = static_cast<T>(*number);
      }
    }};
}
const Parameter parameters[] = {
  parameter<&Config::wheelbase_m>("wheelbase_m", "m", ConfigScope::Execution),
  parameter<&Config::track_m>("track_m", "m", ConfigScope::Execution),
  parameter<&Config::wheel_radius_m>("wheel_radius_m", "m", ConfigScope::Execution),
  parameter<&Config::robot_radius_m>("robot_radius_m", "m", ConfigScope::Execution),
  parameter<&Config::collision_margin_m>("collision_margin_m", "m", ConfigScope::Execution),
  parameter<&Config::max_wheel_speed_mps>("max_wheel_speed_mps", "m/s", ConfigScope::Execution),
  parameter<&Config::max_wheel_accel_mps2>("max_wheel_accel_mps2", "m/s^2", ConfigScope::Execution),
  parameter<&Config::steering_limit_rad>("steering_limit_rad", "rad", ConfigScope::Execution),
  parameter<&Config::max_steer_rate_radps>("max_steer_rate_radps", "rad/s", ConfigScope::Execution),
  parameter<&Config::max_vx_mps>("max_vx_mps", "m/s", ConfigScope::Execution),
  parameter<&Config::max_crab_speed_mps>("max_crab_speed_mps", "m/s", ConfigScope::Execution),
  parameter<&Config::max_spin_radps>("max_spin_radps", "rad/s", ConfigScope::Execution),
  parameter<&Config::max_yaw_rate_radps>("max_yaw_rate_radps", "rad/s", ConfigScope::Execution),
  parameter<&Config::min_turn_radius_m>("min_turn_radius_m", "m", ConfigScope::Execution),
  parameter<&Config::max_linear_accel_mps2>(
    "max_linear_accel_mps2", "m/s^2", ConfigScope::Execution),
  parameter<&Config::max_linear_decel_mps2>(
    "max_linear_decel_mps2", "m/s^2", ConfigScope::Execution),
  parameter<&Config::max_angular_accel_radps2>(
    "max_angular_accel_radps2", "rad/s^2", ConfigScope::Execution),
  parameter<&Config::max_angular_decel_radps2>(
    "max_angular_decel_radps2", "rad/s^2", ConfigScope::Execution),
  parameter<&Config::steering_tolerance_rad>(
    "steering_tolerance_rad", "rad", ConfigScope::Execution),
  parameter<&Config::drive_steering_limit_rad>(
    "drive_steering_limit_rad", "rad", ConfigScope::Execution),
  parameter<&Config::drive_kinematic_tolerance_mps>(
    "drive_kinematic_tolerance_mps", "m/s", ConfigScope::Execution),
  parameter<&Config::stopped_linear_mps>("stopped_linear_mps", "m/s", ConfigScope::Execution),
  parameter<&Config::stopped_angular_radps>(
    "stopped_angular_radps", "rad/s", ConfigScope::Execution),
  parameter<&Config::stopped_wheel_speed_mps>(
    "stopped_wheel_speed_mps", "m/s", ConfigScope::Execution),
  parameter<&Config::feedback_linear_tolerance_mps>(
    "feedback_linear_tolerance_mps", "m/s", ConfigScope::Execution),
  parameter<&Config::feedback_angular_tolerance_radps>(
    "feedback_angular_tolerance_radps", "rad/s", ConfigScope::Execution),
  parameter<&Config::minimum_mode_dwell_s>("minimum_mode_dwell_s", "s", ConfigScope::Execution),
  parameter<&Config::alignment_min_s>("alignment_min_s", "s", ConfigScope::Execution),
  parameter<&Config::confirmation_prediction_s>(
    "confirmation_prediction_s", "s", ConfigScope::Planning),
  parameter<&Config::confirmation_timeout_s>("confirmation_timeout_s", "s", ConfigScope::Execution),
  parameter<&Config::switch_cost>("switch_cost", "dimensionless", ConfigScope::Planning),
  parameter<&Config::switch_hysteresis>(
    "switch_hysteresis", "dimensionless", ConfigScope::Planning),
  parameter<&Config::dt_s>("dt_s", "s", ConfigScope::Execution),
  parameter<&Config::compute_budget_ratio>(
    "compute_budget_ratio", "dimensionless", ConfigScope::Planning),
  parameter<&Config::max_path_points>("max_path_points", "dimensionless", ConfigScope::Execution),
  parameter<&Config::max_obstacles>("max_obstacles", "dimensionless", ConfigScope::Execution),
  parameter<&Config::horizon_steps>("horizon_steps", "dimensionless", ConfigScope::Planning),
  parameter<&Config::stopping_horizon_steps>(
    "stopping_horizon_steps", "dimensionless", ConfigScope::Execution),
  parameter<&Config::safety_reduction_attempts>(
    "safety_reduction_attempts", "dimensionless", ConfigScope::Planning),
  parameter<&Config::samples_per_branch>(
    "samples_per_branch", "dimensionless", ConfigScope::Planning),
  parameter<&Config::iterations>("iterations", "dimensionless", ConfigScope::Planning),
  parameter<&Config::temperature>("temperature", "dimensionless", ConfigScope::Planning),
  parameter<&Config::noise_v_mps>("noise_v_mps", "m/s", ConfigScope::Planning),
  parameter<&Config::noise_w_radps>("noise_w_radps", "rad/s", ConfigScope::Planning),
  parameter<&Config::noise_correlation>(
    "noise_correlation", "dimensionless", ConfigScope::Planning),
  parameter<&Config::control_correction_weight>(
    "control_correction_weight", "dimensionless", ConfigScope::Planning),
  parameter<&Config::random_seed>("random_seed", "dimensionless", ConfigScope::Planning),
  parameter<&Config::path_weight>("path_weight", "dimensionless", ConfigScope::Planning),
  parameter<&Config::goal_weight>("goal_weight", "dimensionless", ConfigScope::Planning),
  parameter<&Config::yaw_weight>("yaw_weight", "dimensionless", ConfigScope::Planning),
  parameter<&Config::effort_weight>("effort_weight", "dimensionless", ConfigScope::Planning),
  parameter<&Config::clearance_weight>("clearance_weight", "dimensionless", ConfigScope::Planning),
  parameter<&Config::path_heading_weight>(
    "path_heading_weight", "dimensionless", ConfigScope::Planning),
  parameter<&Config::smoothness_weight>(
    "smoothness_weight", "dimensionless", ConfigScope::Planning),
  parameter<&Config::goal_speed_weight>(
    "goal_speed_weight", "dimensionless", ConfigScope::Planning),
  parameter<&Config::path_lookahead_m>("path_lookahead_m", "m", ConfigScope::Planning),
  parameter<&Config::path_lookahead_turn_rad>(
    "path_lookahead_turn_rad", "rad", ConfigScope::Planning),
  parameter<&Config::path_search_window_m>("path_search_window_m", "m", ConfigScope::Planning),
  parameter<&Config::path_progress_slack_m>("path_progress_slack_m", "m", ConfigScope::Planning),
  parameter<&Config::goal_position_tolerance_m>(
    "goal_position_tolerance_m", "m", ConfigScope::Planning),
  parameter<&Config::goal_yaw_tolerance_rad>(
    "goal_yaw_tolerance_rad", "rad", ConfigScope::Planning),
  parameter<&Config::goal_settle_time_s>("goal_settle_time_s", "s", ConfigScope::Planning),
  parameter<&Config::goal_slowdown_distance_m>(
    "goal_slowdown_distance_m", "m", ConfigScope::Planning),
  parameter<&Config::goal_docking_distance_m>(
    "goal_docking_distance_m", "m", ConfigScope::Planning),
  parameter<&Config::goal_translation_gain>("goal_translation_gain", "1/s", ConfigScope::Planning),
  parameter<&Config::goal_rotation_gain>("goal_rotation_gain", "1/s", ConfigScope::Planning),
  parameter<&Config::progress_timeout_s>("progress_timeout_s", "s", ConfigScope::Planning),
  parameter<&Config::progress_distance_m>("progress_distance_m", "m", ConfigScope::Planning),
};
const Parameter & find_parameter(std::string_view name)
{
  const auto found = std::find_if(
    std::begin(parameters), std::end(parameters),
    [&](const auto & p) { return p.info.name == name; });
  if (found == std::end(parameters)) {
    throw std::invalid_argument("unknown configuration parameter: " + std::string(name));
  }
  return *found;
}
std::string_view trim(std::string_view text)
{
  const auto first = text.find_first_not_of(" \t\r");
  if (first == std::string_view::npos) {
    return {};
  }
  return text.substr(first, text.find_last_not_of(" \t\r") - first + 1);
}
ConfigValue parse_value(std::string_view text, ConfigValueType type)
{
  if (text.empty()) {
    throw std::invalid_argument("missing numeric value");
  }
  if (type == ConfigValueType::UnsignedInteger) {
    std::uint64_t value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
      throw std::invalid_argument("unsigned decimal integer required");
    }
    return value;
  }
  if (text.front() == '+') {
    text.remove_prefix(1);
    if (text.empty() || text.front() == '-' || text.front() == '+') {
      throw std::invalid_argument("finite decimal real required");
    }
  }
  double value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (
    result.ec != std::errc{} || result.ptr != text.data() + text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("finite decimal real required");
  }
  return value;
}
}  // namespace

const std::vector<ConfigParameterInfo> & config_schema()
{
  static const auto schema = [] {
    std::vector<ConfigParameterInfo> result;
    for (const auto & p : parameters) {
      result.push_back(p.info);
    }
    return result;
  }();
  return schema;
}
std::vector<ConfigParameter> config_parameters(const Config & config)
{
  validate(config);
  std::vector<ConfigParameter> result;
  result.reserve(std::size(parameters));
  for (const auto & p : parameters) {
    result.push_back({std::string(p.info.name), p.read(config)});
  }
  return result;
}
Config with_config_parameters(const Config & base, const std::vector<ConfigParameter> & overrides)
{
  validate(base);
  if (overrides.size() > std::size(parameters)) {
    throw std::invalid_argument("too many configuration parameters");
  }
  Config result = base;
  std::vector<std::string_view> seen;
  seen.reserve(overrides.size());
  for (const auto & item : overrides) {
    const auto & p = find_parameter(item.name);
    if (std::find(seen.begin(), seen.end(), p.info.name) != seen.end()) {
      throw std::invalid_argument("duplicate configuration parameter: " + item.name);
    }
    seen.push_back(p.info.name);
    try {
      p.write(result, item.value);
    } catch (const std::invalid_argument & error) {
      throw std::invalid_argument(item.name + ": " + error.what());
    }
  }
  validate(result);
  return result;
}
Config parse_config_profile(std::string_view text, const Config & base)
{
  if (text.size() > 65536) {
    throw std::invalid_argument("configuration profile exceeds 64 KiB");
  }
  validate(base);
  Config result = base;
  std::vector<std::string_view> seen;
  seen.reserve(std::size(parameters));
  std::size_t line_number = 0;
  while (!text.empty()) {
    const auto end = text.find('\n');
    auto line = text.substr(0, end);
    text = end == std::string_view::npos ? std::string_view{} : text.substr(end + 1);
    ++line_number;
    line = trim(line.substr(0, line.find('#')));
    if (line.empty()) {
      continue;
    }
    try {
      const auto equal = line.find('=');
      if (equal == std::string_view::npos || line.find('=', equal + 1) != std::string_view::npos) {
        throw std::invalid_argument("expected one name=value assignment");
      }
      const auto name = trim(line.substr(0, equal));
      const auto & p = find_parameter(name);
      if (std::find(seen.begin(), seen.end(), p.info.name) != seen.end()) {
        throw std::invalid_argument("duplicate configuration parameter: " + std::string(name));
      }
      seen.push_back(p.info.name);
      try {
        p.write(result, parse_value(trim(line.substr(equal + 1)), p.info.type));
      } catch (const std::invalid_argument & error) {
        throw std::invalid_argument(std::string(name) + ": " + error.what());
      }
    } catch (const std::invalid_argument & error) {
      throw std::invalid_argument(
        "configuration line " + std::to_string(line_number) + ": " + error.what());
    }
  }
  validate(result);
  return result;
}
std::string write_config_profile(const Config & config)
{
  validate(config);
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<double>::max_digits10);
  for (const auto & p : parameters) {
    output << p.info.name << " = ";
    std::visit([&](auto value) { output << value; }, p.read(config));
    output << "  # " << p.info.unit << "; "
           << (p.info.scope == ConfigScope::Execution ? "execution" : "planning") << '\n';
  }
  return output.str();
}
std::vector<std::string_view> execution_config_mismatches(
  const Config & planner, const Config & executor)
{
  validate(planner);
  validate(executor);
  std::vector<std::string_view> result;
  for (const auto & p : parameters) {
    if (p.info.scope == ConfigScope::Execution && p.read(planner) != p.read(executor)) {
      result.push_back(p.info.name);
    }
  }
  return result;
}
void require_execution_compatible(const Config & planner, const Config & executor)
{
  const auto mismatches = execution_config_mismatches(planner, executor);
  if (!mismatches.empty()) {
    throw std::invalid_argument(
      "planning/execution configuration differs: " + std::string(mismatches.front()));
  }
}
void validate_live_config(const Config & config, double maximum_budget_ratio)
{
  validate(config);
  if (
    !std::isfinite(maximum_budget_ratio) || maximum_budget_ratio <= 0 || maximum_budget_ratio > 1 ||
    config.compute_budget_ratio <= 0 || config.compute_budget_ratio > maximum_budget_ratio ||
    !std::isfinite(config.dt_s * config.compute_budget_ratio) ||
    config.dt_s * config.compute_budget_ratio <= 0) {
    throw std::invalid_argument(
      "live configuration requires a positive representable compute budget within its maximum "
      "ratio");
  }
}
}  // namespace swerve_mppi
