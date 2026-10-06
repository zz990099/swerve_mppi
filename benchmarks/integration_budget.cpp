#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <optional>
#include <string>

#include "behavior_fixture.hpp"
#include "detail/budget_report.hpp"
#include "swerve_mppi/common/config_profile.hpp"
#include "swerve_mppi/execution/profile_runner.hpp"
#include "swerve_mppi/planning/controller.hpp"
using namespace swerve_mppi;
namespace
{
bool same_path(const std::string & a, const std::string & b)
{
  if (a.empty() || b.empty()) {
    return false;
  }
  return std::filesystem::weakly_canonical(a) == std::filesystem::weakly_canonical(b) ||
         (std::filesystem::exists(a) && std::filesystem::exists(b) &&
          std::filesystem::equivalent(a, b));
}
}  // namespace
int main(int argc, char ** argv)
{
  try {
    std::cout.imbue(std::locale::classic());
    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
    if (argc == 2 && std::string(argv[1]) == "--build-info") {
      std::cout << "version=" << SWERVE_MPPI_TOOL_VERSION
                << "\nbuild_type=" << SWERVE_MPPI_TOOL_BUILD_TYPE
                << "\ncompiler=" << SWERVE_MPPI_TOOL_COMPILER << "\ncplusplus=" << __cplusplus
                << '\n';
      return std::cout ? 0 : 1;
    }
    int repetitions = 50;
    std::optional<double> budget_ratio;
    std::optional<int> obstacle_count;
    std::string config_path, resolved_path, trace_path;
    bool strict = false, matrix = false;
    int path_points = 0;
    bool have_repetitions = false;
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--strict") {
        strict = true;
      } else if (argument == "--matrix") {
        matrix = true;
      } else if (argument == "--trace" && i + 1 < argc) {
        if (!trace_path.empty()) {
          throw std::invalid_argument("duplicate --trace");
        }
        trace_path = argv[++i];
        if (trace_path.empty()) {
          throw std::invalid_argument("empty --trace path");
        }
      } else if (argument == "--obstacles" && i + 1 < argc) {
        const std::string value = argv[++i];
        std::size_t used = 0;
        if (obstacle_count) {
          throw std::invalid_argument("duplicate --obstacles");
        }
        obstacle_count = std::stoi(value, &used);
        if (used != value.size() || *obstacle_count < 0 || *obstacle_count > 128) {
          throw std::invalid_argument("obstacles must be 0..128");
        }
      } else if (argument == "--config" && i + 1 < argc) {
        if (!config_path.empty()) {
          throw std::invalid_argument("duplicate --config");
        }
        config_path = argv[++i];
        if (config_path.empty()) {
          throw std::invalid_argument("empty --config path");
        }
      } else if (argument == "--write-config" && i + 1 < argc) {
        if (!resolved_path.empty()) {
          throw std::invalid_argument("duplicate --write-config");
        }
        resolved_path = argv[++i];
        if (resolved_path.empty()) {
          throw std::invalid_argument("empty --write-config path");
        }
      } else if (argument == "--path-points" && i + 1 < argc) {
        const std::string value = argv[++i];
        std::size_t used = 0;
        path_points = std::stoi(value, &used);
        if (used != value.size() || path_points < 2 || path_points > 4096) {
          throw std::invalid_argument("path points must be 2..4096");
        }
      } else if (argument == "--budget-ratio" && i + 1 < argc) {
        const std::string value = argv[++i];
        std::size_t used = 0;
        budget_ratio = std::stod(value, &used);
        if (used != value.size()) {
          throw std::invalid_argument("invalid budget ratio");
        }
      } else if (!have_repetitions && argument.rfind("--", 0) != 0) {
        std::size_t used = 0;
        repetitions = std::stoi(argument, &used);
        if (used != argument.size()) {
          throw std::invalid_argument("invalid repetition count");
        }
        have_repetitions = true;
      } else {
        throw std::invalid_argument(
          "usage: integration_budget [repetitions] [--budget-ratio ratio] [--path-points count] "
          "[--obstacles count] [--matrix] [--config profile] [--write-config resolved-profile] "
          "[--trace calls.csv] [--strict]; --build-info reports build metadata");
      }
    }
    if (repetitions < 2 || repetitions > 10000) {
      throw std::invalid_argument("repetitions must be 2..10000");
    }
    if (matrix && (path_points || obstacle_count)) {
      throw std::invalid_argument("--matrix cannot combine with --path-points or --obstacles");
    }
    if (same_path(trace_path, resolved_path) || same_path(trace_path, config_path)) {
      throw std::invalid_argument(
        "trace path must differ from input and resolved configuration paths");
    }
    Config config;
    if (!config_path.empty()) {
      std::ifstream file(config_path, std::ios::binary);
      if (!file) {
        throw std::invalid_argument("cannot read configuration profile: " + config_path);
      }
      std::string text(65537, '\0');
      file.read(text.data(), static_cast<std::streamsize>(text.size()));
      if (file.bad()) {
        throw std::invalid_argument("failed to read configuration profile: " + config_path);
      }
      text.resize(static_cast<std::size_t>(file.gcount()));
      config = parse_config_profile(text);
    }
    // Precedence is independent of command-line order: explicit ratio wins
    // over the file, which otherwise overrides standalone defaults.
    if (budget_ratio) {
      config.compute_budget_ratio = *budget_ratio;
    }
    validate_live_config(config);
    const std::vector<std::size_t> point_counts =
      matrix ? std::vector<std::size_t>{41, 401, 4096}
             : std::vector<std::size_t>{
                 path_points ? static_cast<std::size_t>(path_points)
                             : test::scenario_input("curve").reference_path.size()};
    const std::vector<int> obstacle_counts =
      obstacle_count ? std::vector<int>{*obstacle_count} : std::vector<int>{0, 40, 128};
    for (const auto points : point_counts) {
      if (
        points > config.max_path_points ||
        static_cast<std::size_t>(obstacle_counts.back()) > config.max_obstacles) {
        throw std::invalid_argument("benchmark workload exceeds configured path/obstacle limits");
      }
    }
    if (!resolved_path.empty()) {
      std::ofstream file(resolved_path);
      file << write_config_profile(config);
      file.close();
      if (!file) {
        throw std::invalid_argument(
          "cannot write resolved configuration profile: " + resolved_path);
      }
    }
    std::ofstream trace;
    if (!trace_path.empty()) {
      trace.open(trace_path);
      if (!trace) {
        throw std::invalid_argument("cannot write call trace: " + trace_path);
      }
      trace.imbue(std::locale::classic());
      trace << std::setprecision(std::numeric_limits<double>::max_digits10);
      benchmark::trace_header(trace);
    }
    benchmark::summary_header(std::cout);
    bool failed = false;
    for (const auto points : point_counts) {
      for (int count : obstacle_counts) {
        const Config c = config;  // Live budget cannot be disabled for this probe.
        auto input = test::scenario_input("curve");
        if (path_points || matrix) {
          // Change density, not the span of the original curve fixture.
          const double span = input.reference_path.back().yaw;
          input.reference_path.clear();
          for (std::size_t i = 0; i < points; ++i) {
            const double a = span * i / (points - 1);
            input.reference_path.push_back({2 * std::sin(a), 2 * (1 - std::cos(a)), a});
          }
        }
        input.obstacles.clear();
        for (int i = 0; i < count; ++i) {
          const double angle = 6.283185307179586 * i / std::max(1, count);
          input.obstacles.push_back({2 * std::cos(angle), 2 * std::sin(angle), .05});
        }
        std::vector<benchmark::Call> calls;
        calls.reserve(static_cast<std::size_t>(repetitions));
        const std::uint64_t session = 1;
        const double wall = 10;
        for (int iteration = 0; iteration < repetitions; ++iteration) {
          // Fresh instances give every repetition the same cold-start workload;
          // pending-mode or terminal fast paths cannot dilute the percentiles.
          std::array<benchmark::Clock::time_point, 7> boundaries;
          boundaries[0] = benchmark::Clock::now();
          Controller controller(c);
          TimedExecutor executor(c, session);
          ProfileRunner runner(c);
          const double now = input.vehicle.stamp_s;
          const auto cpu_begin = std::clock();
          boundaries[1] = benchmark::Clock::now();
          const auto output = controller.compute(input);
          boundaries[2] = benchmark::Clock::now();
          CommandEnvelope packet{session,    static_cast<std::uint64_t>(iteration + 1),
                                 now,        output,
                                 now,        now,
                                 now + .025, CommandTask::capture(input)};
          boundaries[3] = benchmark::Clock::now();
          const auto admitted = executor.update(packet, input, now);
          boundaries[4] = benchmark::Clock::now();
          const bool installed = runner.install(admitted, now, wall);
          boundaries[5] = benchmark::Clock::now();
          const bool sampled = installed && runner.sample(now, wall).has_value();
          boundaries[6] = benchmark::Clock::now();
          const auto cpu_end = std::clock();
          benchmark::Call call;
          call.time = benchmark::durations(boundaries);
          call.cpu_ms = benchmark::cpu_milliseconds(cpu_begin, cpu_end);
          call.work = output.planning_stats;
          call.safety_reductions = output.safety_reductions;
          call.failure = output.failure_reason;
          call.policy = output.control_policy;
          call.timing_error = admitted.timing_error;
          call.safety_error = admitted.safety_error;
          call.rejected_status = admitted.rejected_status;
          call.command = output.command.has_value();
          call.actuation = admitted.actuation.has_value();
          call.execution_fault = admitted.execution.feedback.fault;
          call.installed = installed;
          call.sampled = sampled;
          // Fixed stationary snapshots isolate workload; no claim of plant
          // motion.
          if (installed) {
            const auto start = benchmark::Clock::now();
            call.endpoint_sampled = runner.sample(now + c.dt_s, wall + c.dt_s).has_value();
            call.time.stages[6] = benchmark::milliseconds(benchmark::Clock::now() - start);
          }
          calls.push_back(call);
        }
        const double budget_ms = c.dt_s * c.compute_budget_ratio * 1000;
        const double span = input.reference_path.back().yaw - input.reference_path.front().yaw;
        const auto report =
          benchmark::summary_row(std::cout, points, count, budget_ms, c, calls, span);
        failed = failed || report.failed(strict);
        if (trace.is_open()) {
          for (std::size_t i = 0; i < calls.size(); ++i) {
            benchmark::trace_row(trace, points, count, i + 1, budget_ms, calls[i], span);
          }
        }
      }
    }
    if (trace.is_open()) {
      trace.close();
      if (!trace) {
        throw std::invalid_argument("failed to write call trace: " + trace_path);
      }
    }
    // CI records timing; an operator can opt into a target-host acceptance gate.
    return failed || !std::cout ? 1 : 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
