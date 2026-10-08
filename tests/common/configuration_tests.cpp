#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <locale>
#include <stdexcept>

#include "swerve_mppi/common/config_profile.hpp"
#include "swerve_mppi/model/model.hpp"

using namespace swerve_mppi;
namespace
{
void check(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}
template <class F>
void rejects(F operation, std::string_view expected)
{
  try {
    operation();
  } catch (const std::invalid_argument & error) {
    if (std::string(error.what()).find(expected) == std::string::npos) {
      throw std::runtime_error("expected '" + std::string(expected) + "' in: " + error.what());
    }
    return;
  }
  throw std::runtime_error("invalid configuration was accepted");
}
void same(const Config & a, const Config & b)
{
  const auto left = config_parameters(a), right = config_parameters(b);
  check(left.size() == right.size(), "configuration size changed");
  for (std::size_t i = 0; i < left.size(); ++i) {
    check(
      left[i].name == right[i].name && left[i].value == right[i].value,
      "configuration round trip lost a parameter");
  }
}
struct DecimalComma : std::numpunct<char>
{
  char do_decimal_point() const override { return ','; }
};
void test_complete_profile_and_locale()
{
  Config original;
  original.random_seed = std::numeric_limits<std::uint32_t>::max();
  original.max_linear_accel_mps2 = std::nextafter(.9, 1.0);
  original.noise_v_mps = 0;
  original.compute_budget_ratio = 0;
  const auto text = write_config_profile(original);
  same(original, parse_config_profile(text));
  const auto previous = std::locale();
  std::locale::global(std::locale(previous, new DecimalComma));
  try {
    check(write_config_profile(original) == text, "profile depends on global locale");
    same(original, parse_config_profile(text));
  } catch (...) {
    std::locale::global(previous);
    throw;
  }
  std::locale::global(previous);

  std::ifstream file(SWERVE_MPPI_DEFAULT_PROFILE);
  check(bool(file), "installed example configuration is missing");
  const std::string defaults{
    std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
  same(Config{}, parse_config_profile(defaults));
  const auto & schema = config_schema();
  check(schema.size() == 68, "update the complete parameter schema when Config changes");
  const auto values = config_parameters(original);
  for (std::size_t i = 0; i < schema.size(); ++i) {
    check(
      schema[i].name == values[i].name && !schema[i].unit.empty(), "schema metadata incomplete");
    check(
      (schema[i].type == ConfigValueType::Real) == std::holds_alternative<double>(values[i].value),
      "schema numeric type mismatch");
    for (std::size_t j = 0; j < i; ++j) {
      check(schema[i].name != schema[j].name, "schema names must be unique");
    }
  }
}
void test_transactional_loading_and_model()
{
  Config base;
  base.random_seed = 9;
  const auto configured = parse_config_profile(
    "# startup overrides\r\nmax_linear_accel_mps2 = 2e-1\r\n"
    "max_wheel_accel_mps2 = 2.0\nrandom_seed = 4294967295 # exact integer\n",
    base);
  check(
    configured.max_linear_accel_mps2 == .2 && configured.max_wheel_accel_mps2 == 2 &&
      configured.random_seed == std::numeric_limits<std::uint32_t>::max(),
    "overrides not applied");
  check(
    base.max_linear_accel_mps2 == .9 && base.random_seed == 9,
    "loading modified the base configuration");
  const auto step = DriveModel(configured).step({}, {.5, 0, 0}, configured.dt_s);
  check(
    step.valid && step.state.velocity.vx > 0 && step.state.velocity.vx <= .02 + 1e-9,
    "loaded acceleration does not reach the actual model");
  same(base, parse_config_profile("\n# comment only\n", base));
  rejects([&] { parse_config_profile("temperature = .5\nmax_vx_mps = -1", base); }, "positive");
  check(base.temperature == .35, "failed loading modified base state");
  // Coupled overrides are validated after the complete transaction, not in
  // source order (the tolerance alone would exceed the old steering step).
  const auto coupled = with_config_parameters(
    base, {{"steering_tolerance_rad", .3}, {"drive_steering_limit_rad", .4}});
  check(
    coupled.steering_tolerance_rad == .3 && coupled.drive_steering_limit_rad == .4,
    "coupled valid update failed");
}
void test_rejected_profiles()
{
  for (const auto text :
       {"horizon_steps = -2", "horizon_steps = 2.0", "horizon_steps = 2e1", "horizon_steps = +2",
        "random_seed = 4294967296", "random_seed = 18446744073709551616"}) {
    rejects([&] { parse_config_profile(text); }, "configuration line 1");
  }
  for (const auto text :
       {"dt_s = nan", "dt_s = inf", "dt_s = 1e309", "dt_s = 1e-400", "dt_s = +-1", "dt_s = .1junk",
        "dt_s = 0x1p0", "dt_s =", "dt_s: .1", "dt_s = .1 = .2"}) {
    rejects([&] { parse_config_profile(text); }, "configuration line 1");
  }
  rejects([] { parse_config_profile("# first\nunknown_limit = 1"); }, "configuration line 2");
  rejects([] { parse_config_profile("dt_s=.1\ndt_s=.2"); }, "duplicate");
  rejects([] { parse_config_profile("goal_position_tolerance_m = .5"); }, "invalid");
  rejects([] { parse_config_profile(std::string(65537, '#')); }, "64 KiB");
  const char nul[] = "dt_s=.1\0junk";
  rejects([&] { parse_config_profile(std::string_view(nul, sizeof(nul) - 1)); }, "line 1");
  rejects([] { with_config_parameters({}, {{"random_seed", 42.0}}); }, "random_seed");
  rejects([] { with_config_parameters({}, {{"dt_s", std::uint64_t{1}}}); }, "dt_s");
  rejects(
    [] { with_config_parameters({}, {{"max_vx_mps", std::numeric_limits<double>::infinity()}}); },
    "max_vx_mps");
  rejects(
    [] { with_config_parameters({}, std::vector<ConfigParameter>(69, {"dt_s", .1})); }, "too many");
}

void test_live_budget_admission()
{
  validate_live_config(Config{});
  Config c;
  c.compute_budget_ratio = 0;
  validate(c);
  rejects([&] { validate_live_config(c); }, "live configuration");
  c.compute_budget_ratio = .6;
  validate_live_config(c, .7);
  rejects([&] { validate_live_config(c, .5); }, "live configuration");
  for (double maximum : {0.0, -1.0, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
    rejects([&] { validate_live_config(c, maximum); }, "live configuration");
  }
  c.dt_s = std::numeric_limits<double>::denorm_min();
  c.compute_budget_ratio = .1;
  rejects([&] { validate_live_config(c); }, "representable");
}
}  // namespace
int main()
{
  try {
    test_complete_profile_and_locale();
    test_transactional_loading_and_model();
    test_rejected_profiles();

    test_live_budget_admission();
    std::cout << "Configuration regressions passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
