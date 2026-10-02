#include "behavior_fixture.hpp"
#include "swerve_mppi/controller.hpp"
#include "swerve_mppi/profile_runner.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
using namespace swerve_mppi;
int main(int argc, char **argv) {
  try {
    const int repetitions = argc == 2 ? std::stoi(argv[1]) : 50;
    if (repetitions < 2 || repetitions > 10000)
      throw std::invalid_argument("repetitions must be 2..10000");
    std::cout << "obstacles,repetitions,budget_ms,p50_ms,p95_ms,p99_ms,max_ms,min_headroom_ms,"
                 "compute_timeouts,total_overruns,other_failures\n";
    int failures = 0;
    for (int count : {0, 40, 128}) {
      Config c; // Production budget enabled. Never disable it for this probe.
      auto input = test::scenario_input("curve");
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
        if (output.failure_reason == FailureReason::ComputeTimeout)
          ++timeouts;
        else if (output.action == Action::SafeStop)
          ++other;
        CommandEnvelope packet{session,    static_cast<std::uint64_t>(iteration + 1),
                               now,        output,
                               now,        now,
                               now + .025, CommandTask::capture(input)};
        const auto admitted = executor.update(packet, input, now);
        const bool installed = runner.install(admitted, now, wall);
        if (installed) {
          if (!runner.sample(now, wall))
            ++other;
        } else if (output.action != Action::SafeStop)
          ++other;
        const double ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count();
        times.push_back(ms);
        overruns += ms > c.dt_s * c.compute_budget_ratio * 1000;
        // Fixed stationary snapshots isolate workload; no claim of plant motion.
        if (installed)
          runner.sample(now + c.dt_s, wall + c.dt_s);
      }
      std::sort(times.begin(), times.end());
      const auto percentile = [&](double p) {
        return times[static_cast<std::size_t>(std::ceil(p * times.size())) - 1];
      };
      const double budget_ms = c.dt_s * c.compute_budget_ratio * 1000;
      std::cout << count << ',' << repetitions << ',' << budget_ms << ',' << percentile(.5) << ','
                << percentile(.95) << ',' << percentile(.99) << ',' << times.back() << ','
                << budget_ms - times.back() << ',' << timeouts << ',' << overruns << ',' << other
                << '\n';
      failures += other;
    }
    return failures ? 1 : 0; // Timing measurements are not a host-specific CI threshold.
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
