#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>

#include "behavior_fixture.hpp"
#include "swerve_mppi/common/config_profile.hpp"
#include "swerve_mppi/execution/profile_runner.hpp"
#include "swerve_mppi/planning/controller.hpp"
using namespace swerve_mppi;
int main(int argc, char ** argv)
{
  try {
    int repetitions = 50;
    std::optional<double> budget_ratio;
    std::string config_path, resolved_path;
    bool strict = false;
    int path_points = 0;
    bool have_repetitions = false;
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--strict") {
        strict = true;
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
          "[--config profile] [--write-config resolved-profile] [--strict]");
      }
    }
    if (repetitions < 2 || repetitions > 10000) {
      throw std::invalid_argument("repetitions must be 2..10000");
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
    const auto workload_points = path_points ? static_cast<std::size_t>(path_points)
                                             : test::scenario_input("curve").reference_path.size();
    if (workload_points > config.max_path_points || config.max_obstacles < 128) {
      throw std::invalid_argument("benchmark workload exceeds configured path/obstacle limits");
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
    std::cout << "path_points,obstacles,repetitions,budget_ms,p50_ms,p95_ms,p99_ms,max_ms,"
                 "min_headroom_ms,"
                 "compute_timeouts,total_overruns,other_failures\n";
    int failures = 0;
    for (int count : {0, 40, 128}) {
      const Config c = config;  // Live budget cannot be disabled for this probe.
      auto input = test::scenario_input("curve");
      if (path_points) {
        input.reference_path.clear();
        for (int i = 0; i < path_points; ++i) {
          const double a = .5 * i / (path_points - 1);
          input.reference_path.push_back({2 * std::sin(a), 2 * (1 - std::cos(a)), a});
        }
      }
      input.obstacles.clear();
      for (int i = 0; i < count; ++i) {
        const double angle = 6.283185307179586 * i / std::max(1, count);
        input.obstacles.push_back({2 * std::cos(angle), 2 * std::sin(angle), .05});
      }
      std::vector<double> times;
      int timeouts = 0, overruns = 0, other = 0;
      const std::uint64_t session = 1;
      const double wall = 10;
      for (int iteration = 0; iteration < repetitions; ++iteration) {
        // Fresh instances give every repetition the same cold-start workload;
        // pending-mode or terminal fast paths cannot dilute the percentiles.
        Controller controller(c);
        TimedExecutor executor(c, session);
        ProfileRunner runner(c);
        const double now = input.vehicle.stamp_s;
        const auto start = std::chrono::steady_clock::now();
        const auto output = controller.compute(input);
        if (output.failure_reason == FailureReason::ComputeTimeout) {
          ++timeouts;
        } else if (!output.command) {
          ++other;
        }
        CommandEnvelope packet{session,    static_cast<std::uint64_t>(iteration + 1),
                               now,        output,
                               now,        now,
                               now + .025, CommandTask::capture(input)};
        const auto admitted = executor.update(packet, input, now);
        const bool installed = runner.install(admitted, now, wall);
        if (installed) {
          if (!runner.sample(now, wall)) {
            ++other;
          }
        } else if (output.command.has_value()) {
          ++other;
        }
        const double ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
            .count();
        times.push_back(ms);
        overruns += ms > c.dt_s * c.compute_budget_ratio * 1000;
        // Fixed stationary snapshots isolate workload; no claim of plant
        // motion.
        if (installed) {
          runner.sample(now + c.dt_s, wall + c.dt_s);
        }
      }
      std::sort(times.begin(), times.end());
      const auto percentile = [&](double p) {
        return times[static_cast<std::size_t>(std::ceil(p * times.size())) - 1];
      };
      const double budget_ms = c.dt_s * c.compute_budget_ratio * 1000;
      std::cout << input.reference_path.size() << ',' << count << ',' << repetitions << ','
                << budget_ms << ',' << percentile(.5) << ',' << percentile(.95) << ','
                << percentile(.99) << ',' << times.back() << ',' << budget_ms - times.back() << ','
                << timeouts << ',' << overruns << ',' << other << '\n';
      failures += other + (strict ? timeouts + overruns : 0);
    }
    // CI records timing; an operator can opt into a target-host acceptance gate.
    return failures ? 1 : 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
