#include <iostream>
#include <sstream>
#include <stdexcept>

#include "detail/budget_report.hpp"

using namespace swerve_mppi;
namespace b = swerve_mppi::benchmark;
namespace
{
void check(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}
b::Call healthy()
{
  b::Call call;
  call.command = call.actuation = call.installed = call.sampled = call.endpoint_sampled = true;
  call.policy = ControlPolicy::Tracking;
  return call;
}
void test_boundaries_and_cpu_clock()
{
  std::array<b::Clock::time_point, 7> points;
  const int absolute_ms[] = {0, 80, 95, 98, 105, 107, 110};
  for (std::size_t i = 0; i < points.size(); ++i) {
    points[i] = b::Clock::time_point{} + std::chrono::milliseconds(absolute_ms[i]);
  }
  const auto times = b::durations(points);
  check(
    times.pipeline_ms == 30 && times.stages == std::array<double, 7>{80, 15, 3, 7, 2, 3, 0},
    "setup must be excluded and pipeline stages must partition the measured interval");
  points[3] = points[1];
  try {
    b::durations(points);
    throw std::runtime_error("backwards clock accepted");
  } catch (const std::invalid_argument &) {
  }
  check(
    !b::cpu_milliseconds(static_cast<std::clock_t>(-1), 0) && !b::cpu_milliseconds(2, 1),
    "unavailable and wrapped CPU clocks must not invent data");
  check(
    b::cpu_milliseconds(0, CLOCKS_PER_SEC).value() == 1000,
    "CPU tick conversion must preserve its independent units");
}
void test_root_outcomes_and_safety()
{
  auto call = healthy();
  check(b::outcome(call) == b::Outcome::Healthy, "healthy pipeline misclassified");
  call.failure = FailureReason::NoFeasiblePlan;
  check(
    b::outcome(call) == b::Outcome::ControllerFailure,
    "a safe fallback must not disguise failure of the requested planning workload");
  call = healthy();
  call.command = false;
  check(b::outcome(call) == b::Outcome::ControllerFailure, "missing authorization not recorded");
  for (int fault = 0; fault < 5; ++fault) {
    call = healthy();
    if (fault == 0) {
      call.actuation = false;
    }
    if (fault == 1) {
      call.safety_error = ExecutionSafetyError::CommandRejected;
    }
    if (fault == 2) {
      call.timing_error = TimingError::ExecutionExpired;
    }
    if (fault == 3) {
      call.execution_fault = true;
    }
    if (fault == 4) {
      call.rejected_status = TrajectoryStatus::Collision;
    }
    check(
      b::outcome(call) == b::Outcome::AdmissionFailure,
      "admission rejection must be visible even if a fallback installs");
  }
  call = healthy();
  call.installed = false;
  check(b::outcome(call) == b::Outcome::InstallationFailure, "installation failure not recorded");
  for (int endpoint = 0; endpoint < 2; ++endpoint) {
    call = healthy();
    if (endpoint) {
      call.endpoint_sampled = false;
    } else {
      call.sampled = false;
    }
    check(b::outcome(call) == b::Outcome::SamplingFailure, "sampling failure not recorded");
  }
  call = {};
  call.failure = FailureReason::ComputeTimeout;
  check(b::outcome(call) == b::Outcome::ComputeTimeout, "checked timeout misclassified");
  for (int authorization = 0; authorization < 5; ++authorization) {
    auto unsafe = call;
    if (authorization == 0) {
      unsafe.command = true;
    }
    if (authorization == 1) {
      unsafe.actuation = true;
    }
    if (authorization == 2) {
      unsafe.installed = true;
    }
    if (authorization == 3) {
      unsafe.sampled = true;
    }
    if (authorization == 4) {
      unsafe.endpoint_sampled = true;
    }
    check(b::counts({unsafe}, 60).failed(false), "timeout concealed unsafe authorization");
  }
}
void test_percentiles_and_accounting()
{
  const auto percentiles = b::distribution({4, 1, 3, 2});
  check(
    percentiles.p50 == 2 && percentiles.p95 == 4 && percentiles.p99 == 4 &&
      percentiles.maximum == 4,
    "nearest-rank percentiles changed");
  auto on_boundary = healthy();
  on_boundary.time.pipeline_ms = 60;
  on_boundary.time.stages[1] = 60;
  auto execution_overrun = healthy();
  execution_overrun.time.pipeline_ms = 65;
  execution_overrun.time.stages[1] = 55;
  auto planning_overrun = healthy();
  planning_overrun.time.pipeline_ms = 66;
  planning_overrun.time.stages[1] = 61;
  b::Call timeout;
  timeout.failure = FailureReason::ComputeTimeout;
  timeout.work = {2, 7, 4, 1, true};
  timeout.safety_reductions = 3;
  const auto report = b::counts({on_boundary, execution_overrun, planning_overrun, timeout}, 60);
  check(
    report.compute_timeouts == 1 && report.overruns == 2 && report.controller_overruns == 1 &&
      report.post_controller_overruns == 1,
    "controller and post-controller overruns must account for the total without double counting");
  check(
    report.outcomes[0] == 3 && report.outcomes[1] == 1 && report.other_failures() == 0 &&
      !report.failed(false) && report.failed(true),
    "strict and recording acceptance were conflated");
  check(
    report.branches == 2 && report.evaluated == 7 && report.feasible == 4 &&
      report.fallbacks == 1 && report.reductions == 3,
    "partial timeout work was lost or inferred from configured maxima");
  auto failed = healthy();
  failed.sampled = false;
  check(b::counts({failed}, 60).failed(false), "functional failure must fail recording mode too");
  std::ostringstream trace, summary;
  b::trace_header(trace);
  b::trace_row(trace, 41, 0, 1, 60, timeout, .7);
  b::summary_header(summary);
  b::summary_row(summary, 41, 0, 60, Config{}, {timeout}, .7);
  check(
    trace.str().find(",compute_timeout,") != std::string::npos &&
      trace.str().find("ComputeTimeout") != std::string::npos,
    "trace lost readable root and controller diagnostics");
  check(
    summary.str().find(",0,,,") != std::string::npos,
    "unavailable CPU samples must remain absent rather than appear as zero");
}
}  // namespace
int main()
{
  try {
    test_boundaries_and_cpu_clock();
    test_root_outcomes_and_safety();
    test_percentiles_and_accounting();
    std::cout << "Offline budget report regressions passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
