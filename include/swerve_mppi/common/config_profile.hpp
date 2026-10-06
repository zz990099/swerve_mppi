#pragma once

#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "swerve_mppi/common/config.hpp"

namespace swerve_mppi
{
// Execution includes model, protocol, admission and stopping-certificate
// settings shared by planning and execution. Planning settings may differ.
enum class ConfigScope
{
  Execution,
  Planning
};
enum class ConfigValueType
{
  Real,
  UnsignedInteger
};
using ConfigValue = std::variant<double, std::uint64_t>;
struct ConfigParameterInfo
{
  std::string_view name;
  std::string_view unit;
  ConfigValueType type;
  ConfigScope scope;
};
struct ConfigParameter
{
  std::string name;
  ConfigValue value;
};

// Startup-only APIs. They allocate and must not run in actuator callbacks.
const std::vector<ConfigParameterInfo> & config_schema();
std::vector<ConfigParameter> config_parameters(const Config & config);
// Transactional: rejects unknown/duplicate names, wrong numeric types,
// narrowing and invalid final configurations. The base is never modified.
Config with_config_parameters(const Config & base, const std::vector<ConfigParameter> & overrides);
// Portable, locale-independent name=value profile. Comments begin with '#'.
// Integer values are unsigned decimal; real values allow decimal exponents.
// Input is bounded to 64 KiB. Empty profiles preserve the validated base.
Config parse_config_profile(std::string_view text, const Config & base = Config{});
// Peer exchange requires every registered parameter exactly once. Unlike an
// override profile, omitted fields never inherit local defaults.
Config parse_resolved_config_profile(std::string_view text);
std::string write_config_profile(const Config & config);
// Exact shared-setting comparison, before arming. Neither tolerant numeric
// matching nor a hash can establish agreement. Returns the differing names.
std::vector<std::string_view> execution_config_mismatches(
  const Config & planner, const Config & executor);
void require_execution_compatible(const Config & planner, const Config & executor);
// Offline zero-budget profiles remain supported; live consumers must opt into
// this check and may impose a tighter maximum ratio for transport headroom.
void validate_live_config(const Config & config, double maximum_budget_ratio = 1.0);
}  // namespace swerve_mppi
