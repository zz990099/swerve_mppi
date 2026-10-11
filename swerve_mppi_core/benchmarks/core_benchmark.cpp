#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <new>
#include <numeric>

#include "behavior_fixture.hpp"
#include "swerve_mppi/planning/controller.hpp"

// Counts ordinary C++ allocations during compute, excluding fixture/reporting.
// Single-threaded benchmark instrumentation, never linked into the library.
namespace
{
bool counting = false;
std::size_t allocations = 0;
}  // namespace
void * operator new(std::size_t size)
{
  if (void * p = std::malloc(size ? size : 1)) {
    if (counting) {
      ++allocations;
    }
    return p;
  }
  throw std::bad_alloc();
}
void * operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void * p) noexcept { std::free(p); }
void operator delete[](void * p) noexcept { std::free(p); }
void operator delete(void * p, std::size_t) noexcept { std::free(p); }
void operator delete[](void * p, std::size_t) noexcept { std::free(p); }
namespace
{
using namespace swerve_mppi;
using namespace swerve_mppi::test;
double percentile(std::vector<double> samples, double q)
{
  std::sort(samples.begin(), samples.end());
  return samples.at(static_cast<std::size_t>(std::ceil(q * samples.size())) - 1);
}
void run(
  const std::string & scenario, unsigned seed, double lookahead, double path_weight,
  bool check_allocations)
{
  Config c;
  c.compute_budget_ratio = 0;  // Measure full work; deadline admission is tested separately.
  c.random_seed = seed;
  c.path_lookahead_m = lookahead;
  c.path_weight = path_weight;
  Controller controller(c);
  NominalChassis plant(c);
  auto input =
    scenario_input(scenario == "dense_curve" || scenario == "obstacles" ? "curve" : scenario);
  if (scenario == "dense_curve") {
    input.reference_path.clear();
    for (int i = 0; i <= 400; ++i) {
      const double a = .7 * i / 400;
      input.reference_path.push_back({2 * std::sin(a), 2 * (1 - std::cos(a)), a});
    }
  }
  if (scenario == "obstacles") {
    for (int i = 0; i < 40; ++i) {
      input.obstacles.push_back({-2.0 - .1 * i, 2, .05});
    }
  }
  std::vector<double> times, planning_times;
  std::size_t planning_allocations = 0, rollouts = 0, feasible = 0, fallbacks = 0;
  std::size_t total_allocations = 0, peak_allocations = 0, overruns = 0;
  double max_error = 0, squared_error = 0;
  double minimum_clearance =
    measured_clearance(input.vehicle.pose, input.vehicle.pose, input.obstacles, c);
  check(minimum_clearance > 0, "benchmark must begin outside inflated obstacles");
  std::size_t waiting_calls = 0, safety_reductions = 0;
  int switches = 0, completion = -1;
  for (int tick = 0; tick < 400; ++tick) {
    allocations = 0;
    counting = true;
    const auto begin = std::chrono::steady_clock::now();
    const auto output = controller.compute(input);
    const auto end = std::chrono::steady_clock::now();
    counting = false;
    times.push_back(std::chrono::duration<double, std::milli>(end - begin).count());
    overruns += times.back() > 1000 * c.model_period_s;
    total_allocations += allocations;
    peak_allocations = std::max(peak_allocations, allocations);
    if (output.planning_stats.branches > 0) {
      planning_times.push_back(times.back());
      planning_allocations += allocations;
    }
    rollouts += output.planning_stats.evaluated_rollouts;
    feasible += output.planning_stats.feasible_rollouts;
    fallbacks += output.planning_stats.fallback_updates;
    safety_reductions += output.safety_reductions;
    waiting_calls += output.navigation_status == NavigationStatus::Waiting;
    max_error = std::max(max_error, output.cross_track_error_m);
    squared_error += output.cross_track_error_m * output.cross_track_error_m;
    check(output.command.has_value(), "benchmark controller fault");
    if (output.goal_reached) {
      completion = tick;
      break;
    }
    auto result = plant.update(output, input.vehicle);
    check(!result.feedback.fault, "benchmark execution fault");
    auto previous = input.vehicle.actual_mode;
    const auto from = input.vehicle.pose;
    actuate(input.vehicle, result, c);
    advance_input(input, &output);
    minimum_clearance =
      std::min(minimum_clearance, measured_clearance(from, input.vehicle.pose, input.obstacles, c));
    check(minimum_clearance > 0, "benchmark measured motion entered an inflated obstacle");
    switches += input.vehicle.actual_mode != previous;
  }
  if (completion < 0) {
    throw std::runtime_error(
      "benchmark did not complete: " + scenario + " seed=" + std::to_string(seed));
  }
  if (check_allocations) {
    check(peak_allocations <= 200, "fixed default workload exceeded allocation regression bound");
  }
  if (scenario == "near_obstacles") {
    check(minimum_clearance < .2, "near obstacles must exercise the clearance region");
  }
  std::cout << scenario << ',' << seed << ',' << times.size() << ','
            << completion * c.model_period_s << ',' << max_error << ','
            << std::sqrt(squared_error / times.size()) << ',' << switches << ','
            << percentile(times, .5) << ',' << percentile(times, .95) << ','
            << percentile(times, .99) << ',' << *std::max_element(times.begin(), times.end()) << ','
            << overruns << ',' << static_cast<double>(total_allocations) / times.size() << ','
            << peak_allocations << ',' << planning_times.size() << ','
            << (planning_times.empty() ? 0 : percentile(planning_times, .95)) << ','
            << (planning_times.empty()
                  ? 0
                  : static_cast<double>(planning_allocations) / planning_times.size())
            << ',' << rollouts << ',' << feasible << ',' << fallbacks << ',' << waiting_calls << ','
            << safety_reductions << ',';
  if (std::isfinite(minimum_clearance)) {
    std::cout << minimum_clearance;
  }
  std::cout << '\n';
}
}  // namespace
int main(int argc, char ** argv)
{
  try {
    check(
      argc <= 5,
      "usage: core_benchmark [all|scenario] [seed] [lookahead_m] "
      "[path_weight]");
    const bool smoke = argc == 2 && std::string(argv[1]) == "--smoke";
    const std::string scenario = smoke ? "straight" : argc > 1 ? argv[1] : "all";
    const double lookahead = argc > 3 ? std::stod(argv[3]) : Config{}.path_lookahead_m;
    const double weight = argc > 4 ? std::stod(argv[4]) : Config{}.path_weight;
    std::cout << std::setprecision(9)
              << "scenario,seed,calls,completion_s,max_path_error_m,rms_path_"
                 "error_m,mode_changes,"
                 "p50_ms,p95_ms,p99_ms,max_ms,period_overruns,mean_allocations,"
                 "peak_allocations,"
                 "planning_calls,planning_p95_ms,planning_mean_allocations,"
                 "evaluated_rollouts,"
                 "feasible_rollouts,fallback_updates,waiting_calls,safety_"
                 "reductions,min_measured_"
                 "clearance_m\n";
    for (const auto & name :
         {"straight", "lateral", "curve", "spin", "reverse", "final_yaw", "scurve",
          "scurve_duplicates", "cusp", "loop", "short_cusp", "short_loop", "short_corner",
          "near_obstacles", "dense_curve", "obstacles"}) {
      if (
        scenario == "all" &&
        (std::string(name) == "dense_curve" || std::string(name) == "obstacles")) {
        continue;
      }
      if (scenario != "all" && scenario != name) {
        continue;
      }
      if (smoke || argc > 2) {
        run(name, smoke ? 42u : std::stoul(argv[2]), lookahead, weight, smoke);
      } else {
        for (unsigned seed : {1u, 7u, 42u, 73u, 101u}) {
          run(name, seed, lookahead, weight, false);
        }
      }
    }
    check(
      scenario == "all" || scenario == "straight" || scenario == "lateral" || scenario == "curve" ||
        scenario == "spin" || scenario == "reverse" || scenario == "final_yaw" ||
        scenario == "scurve" || scenario == "scurve_duplicates" || scenario == "cusp" ||
        scenario == "loop" || scenario == "short_cusp" || scenario == "short_loop" ||
        scenario == "short_corner" || scenario == "near_obstacles" || scenario == "dense_curve" ||
        scenario == "obstacles",
      "unknown scenario");
  } catch (const std::exception & e) {
    counting = false;
    std::cerr << e.what() << '\n';
    return 1;
  }
}
