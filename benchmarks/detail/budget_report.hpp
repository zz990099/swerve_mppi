#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include "swerve_mppi/execution/timing.hpp"

namespace swerve_mppi::benchmark
{
// Measurement/reporting lives in the offline tool, not in library callbacks.
using Clock = std::chrono::steady_clock;
inline constexpr std::array<const char *, 7> stage_names{
  "setup", "controller", "envelope", "admission", "install", "sample", "endpoint_sample"};
struct Durations
{
  std::array<double, 7> stages{};
  double pipeline_ms = 0;
};
inline double milliseconds(Clock::duration duration)
{
  return std::chrono::duration<double, std::milli>(duration).count();
}
// Consecutive boundaries partition the pipeline exactly. Setup and the later
// endpoint sample are measured independently and excluded from the live budget.
inline Durations durations(const std::array<Clock::time_point, 7> & boundaries)
{
  Durations result;
  for (std::size_t i = 0; i + 1 < boundaries.size(); ++i) {
    if (boundaries[i + 1] < boundaries[i]) {
      throw std::invalid_argument("nonmonotonic measurement clock");
    }
    result.stages[i] = milliseconds(boundaries[i + 1] - boundaries[i]);
  }
  result.pipeline_ms = milliseconds(boundaries.back() - boundaries[1]);
  return result;
}
inline std::optional<double> cpu_milliseconds(std::clock_t begin, std::clock_t end)
{
  if (
    static_cast<long double>(begin) < 0 || static_cast<long double>(end) < 0 || end < begin ||
    begin == static_cast<std::clock_t>(-1) || end == static_cast<std::clock_t>(-1)) {
    return std::nullopt;
  }
  return static_cast<double>(
    (static_cast<long double>(end) - static_cast<long double>(begin)) * 1000 / CLOCKS_PER_SEC);
}
enum class Outcome : std::size_t
{
  Healthy,
  ComputeTimeout,
  ControllerFailure,
  AdmissionFailure,
  InstallationFailure,
  SamplingFailure
};
inline constexpr std::array<const char *, 6> outcome_names{
  "ok",
  "compute_timeout",
  "controller_failure",
  "admission_failure",
  "installation_failure",
  "sampling_failure"};
struct Call
{
  Durations time;
  std::optional<double> cpu_ms;
  PlanningStats work;
  std::size_t safety_reductions = 0;
  FailureReason failure = FailureReason::None;
  ControlPolicy policy = ControlPolicy::Fault;
  TimingError timing_error = TimingError::None;
  ExecutionSafetyError safety_error = ExecutionSafetyError::None;
  TrajectoryStatus rejected_status = TrajectoryStatus::Valid;
  bool command = false;
  bool actuation = false;
  bool execution_fault = false;
  bool installed = false;
  bool sampled = false;
  bool endpoint_sampled = false;
};
inline Outcome outcome(const Call & call)
{
  if (call.failure == FailureReason::ComputeTimeout) {
    // A timeout must withhold authorization. Do not let the timeout category
    // conceal an executable command/profile if that invariant ever regresses.
    return call.command || call.actuation || call.installed || call.sampled || call.endpoint_sampled
             ? Outcome::ControllerFailure
             : Outcome::ComputeTimeout;
  }
  if (call.failure != FailureReason::None || !call.command) {
    return Outcome::ControllerFailure;
  }
  if (
    !call.actuation || call.execution_fault || call.timing_error != TimingError::None ||
    call.safety_error != ExecutionSafetyError::None ||
    call.rejected_status != TrajectoryStatus::Valid) {
    return Outcome::AdmissionFailure;
  }
  if (!call.installed) {
    return Outcome::InstallationFailure;
  }
  return call.sampled && call.endpoint_sampled ? Outcome::Healthy : Outcome::SamplingFailure;
}
struct Distribution
{
  double p50, p95, p99, maximum;
};
inline Distribution distribution(std::vector<double> values)
{
  if (values.empty() || std::any_of(values.begin(), values.end(), [](double v) {
        return !std::isfinite(v);
      })) {
    throw std::invalid_argument("finite nonempty measurement distribution required");
  }
  std::sort(values.begin(), values.end());
  const auto percentile = [&](double p) {
    return values[static_cast<std::size_t>(std::ceil(p * values.size())) - 1];
  };
  return {percentile(.5), percentile(.95), percentile(.99), values.back()};
}
struct Counts
{
  std::array<std::size_t, 6> outcomes{};
  std::size_t compute_timeouts = 0, overruns = 0, controller_overruns = 0,
              post_controller_overruns = 0;
  std::size_t branches = 0, evaluated = 0, feasible = 0, fallbacks = 0, reductions = 0;
  std::size_t other_failures() const
  {
    return outcomes[2] + outcomes[3] + outcomes[4] + outcomes[5];
  }
  bool failed(bool strict) const
  {
    return other_failures() != 0 || (strict && (compute_timeouts != 0 || overruns != 0));
  }
};
inline Counts counts(const std::vector<Call> & calls, double budget_ms)
{
  if (!std::isfinite(budget_ms) || budget_ms <= 0) {
    throw std::invalid_argument("positive measurement budget required");
  }
  Counts result;
  for (const auto & call : calls) {
    ++result.outcomes[static_cast<std::size_t>(outcome(call))];
    result.compute_timeouts += call.failure == FailureReason::ComputeTimeout;
    result.overruns += call.time.pipeline_ms > budget_ms;
    result.controller_overruns += call.time.stages[1] > budget_ms;
    result.post_controller_overruns +=
      call.time.pipeline_ms > budget_ms && call.time.stages[1] <= budget_ms;
    result.branches += call.work.branches;
    result.evaluated += call.work.evaluated_rollouts;
    result.feasible += call.work.feasible_rollouts;
    result.fallbacks += call.work.fallback_updates;
    result.reductions += call.safety_reductions;
  }
  return result;
}
inline std::string_view name(FailureReason value)
{
  switch (value) {
    case FailureReason::None:
      return "None";
    case FailureReason::InvalidInput:
      return "InvalidInput";
    case FailureReason::NonmonotonicTime:
      return "NonmonotonicTime";
    case FailureReason::InvalidPath:
      return "InvalidPath";
    case FailureReason::FeedbackFault:
      return "FeedbackFault";
    case FailureReason::NoFeasiblePlan:
      return "NoFeasiblePlan";
    case FailureReason::ModelFailure:
      return "ModelFailure";
    case FailureReason::TransitionFault:
      return "TransitionFault";
    case FailureReason::UnsafeStoppingTrajectory:
      return "UnsafeStoppingTrajectory";
    case FailureReason::InconsistentFeedback:
      return "InconsistentFeedback";
    case FailureReason::WorkloadExceeded:
      return "WorkloadExceeded";
    case FailureReason::ComputeTimeout:
      return "ComputeTimeout";
  }
  return "Unknown";
}
inline std::string_view name(ControlPolicy value)
{
  switch (value) {
    case ControlPolicy::Stopped:
      return "Stopped";
    case ControlPolicy::Tracking:
      return "Tracking";
    case ControlPolicy::Alignment:
      return "Alignment";
    case ControlPolicy::Capture:
      return "Capture";
    case ControlPolicy::ModeTransition:
      return "ModeTransition";
    case ControlPolicy::Fault:
      return "Fault";
    case ControlPolicy::Blocked:
      return "Blocked";
  }
  return "Unknown";
}
inline std::string_view name(TimingError value)
{
  switch (value) {
    case TimingError::None:
      return "None";
    case TimingError::InvalidTime:
      return "InvalidTime";
    case TimingError::ClockDiscontinuity:
      return "ClockDiscontinuity";
    case TimingError::OffPeriod:
      return "OffPeriod";
    case TimingError::FeedbackTimeout:
      return "FeedbackTimeout";
    case TimingError::NonmonotonicFeedback:
      return "NonmonotonicFeedback";
    case TimingError::CommandTimeout:
      return "CommandTimeout";
    case TimingError::CommandReplay:
      return "CommandReplay";
    case TimingError::SessionMismatch:
      return "SessionMismatch";
    case TimingError::MissingCommand:
      return "MissingCommand";
    case TimingError::InvalidPlanTime:
      return "InvalidPlanTime";
    case TimingError::SourceTimeout:
      return "SourceTimeout";
    case TimingError::NotYetExecutable:
      return "NotYetExecutable";
    case TimingError::ExecutionExpired:
      return "ExecutionExpired";
  }
  return "Unknown";
}
inline std::string_view name(ExecutionSafetyError value)
{
  switch (value) {
    case ExecutionSafetyError::None:
      return "None";
    case ExecutionSafetyError::InvalidContext:
      return "InvalidContext";
    case ExecutionSafetyError::StateNotCurrent:
      return "StateNotCurrent";
    case ExecutionSafetyError::InvalidActuation:
      return "InvalidActuation";
    case ExecutionSafetyError::CommandRejected:
      return "CommandRejected";
    case ExecutionSafetyError::UnsafeStoppingTrajectory:
      return "UnsafeStoppingTrajectory";
    case ExecutionSafetyError::TaskMismatch:
      return "TaskMismatch";
    case ExecutionSafetyError::InconsistentFeedback:
      return "InconsistentFeedback";
  }
  return "Unknown";
}
inline std::string_view name(TrajectoryStatus value)
{
  switch (value) {
    case TrajectoryStatus::Valid:
      return "Valid";
    case TrajectoryStatus::Invalid:
      return "Invalid";
    case TrajectoryStatus::Collision:
      return "Collision";
    case TrajectoryStatus::Rejected:
      return "Rejected";
  }
  return "Unknown";
}
inline void trace_header(std::ostream & out)
{
  out << "path_points,obstacles,repetition,budget_ms,pipeline_ms";
  for (const auto * stage : stage_names) {
    out << ',' << stage << "_ms";
  }
  out << ",pipeline_cpu_ms,wall_minus_cpu_ms,outcome,compute_timeout,pipeline_overrun,controller_"
         "overrun,"
         "failure_reason,control_policy,timing_error,safety_error,rejected_status,"
         "command,actuation,execution_fault,installed,sampled,endpoint_sampled,"
         "branches,evaluated_rollouts,feasible_rollouts,fallback_updates,safety_reductions,budget_"
         "exhausted,path_span_rad\n";
}
inline void trace_row(
  std::ostream & out, std::size_t points, int obstacles, std::size_t repetition, double budget_ms,
  const Call & call, double span_rad)
{
  out << points << ',' << obstacles << ',' << repetition << ',' << budget_ms << ','
      << call.time.pipeline_ms;
  for (const auto stage : call.time.stages) {
    out << ',' << stage;
  }
  out << ',';
  if (call.cpu_ms) {
    out << *call.cpu_ms;
  }
  out << ',';
  if (call.cpu_ms) {
    out << call.time.pipeline_ms - *call.cpu_ms;
  }
  out << ',' << outcome_names[static_cast<std::size_t>(outcome(call))] << ','
      << (call.failure == FailureReason::ComputeTimeout) << ','
      << (call.time.pipeline_ms > budget_ms) << ',' << (call.time.stages[1] > budget_ms) << ','
      << name(call.failure) << ',' << name(call.policy) << ',' << name(call.timing_error) << ','
      << name(call.safety_error) << ',' << name(call.rejected_status) << ',' << call.command << ','
      << call.actuation << ',' << call.execution_fault << ',' << call.installed << ','
      << call.sampled << ',' << call.endpoint_sampled << ',' << call.work.branches << ','
      << call.work.evaluated_rollouts << ',' << call.work.feasible_rollouts << ','
      << call.work.fallback_updates << ',' << call.safety_reductions << ','
      << call.work.budget_exhausted << ',' << span_rad << '\n';
}
inline void summary_header(std::ostream & out)
{
  // Keep the legacy leading columns; append the richer diagnostics.
  out << "path_points,obstacles,repetitions,budget_ms,p50_ms,p95_ms,p99_ms,max_ms,min_headroom_ms,"
         "compute_timeouts,total_overruns,other_failures,controller_p50_ms,controller_p95_ms,"
         "controller_p99_ms,controller_max_ms";
  for (std::size_t i = 0; i < stage_names.size(); ++i) {
    if (i != 1) {
      out << ',' << stage_names[i] << "_p95_ms";
    }
  }
  out << ",cpu_samples,pipeline_cpu_p95_ms,wall_minus_cpu_p95_ms,controller_overruns,post_"
         "controller_overruns,"
         "successful_calls,timeout_only_calls,controller_failures,admission_failures,installation_"
         "failures,sampling_failures,"
         "branches_total,evaluated_rollouts_total,feasible_rollouts_total,fallback_updates_total,"
         "safety_reductions_total,"
         "horizon_steps,samples_per_branch,iterations,random_seed,path_span_rad\n";
}
inline Counts summary_row(
  std::ostream & out, std::size_t points, int obstacles, double budget_ms, const Config & config,
  const std::vector<Call> & calls, double span_rad)
{
  std::vector<double> pipeline, controller, cpu, gap;
  for (const auto & call : calls) {
    pipeline.push_back(call.time.pipeline_ms);
    controller.push_back(call.time.stages[1]);
    if (call.cpu_ms) {
      cpu.push_back(*call.cpu_ms);
      gap.push_back(call.time.pipeline_ms - *call.cpu_ms);
    }
  }
  const auto whole = distribution(std::move(pipeline)),
             planning = distribution(std::move(controller));
  const auto work = counts(calls, budget_ms);
  out << points << ',' << obstacles << ',' << calls.size() << ',' << budget_ms << ',' << whole.p50
      << ',' << whole.p95 << ',' << whole.p99 << ',' << whole.maximum << ','
      << budget_ms - whole.maximum << ',' << work.compute_timeouts << ',' << work.overruns << ','
      << work.other_failures() << ',' << planning.p50 << ',' << planning.p95 << ',' << planning.p99
      << ',' << planning.maximum;
  for (std::size_t i = 0; i < stage_names.size(); ++i) {
    if (i == 1) {
      continue;
    }
    std::vector<double> values;
    for (const auto & call : calls) {
      values.push_back(call.time.stages[i]);
    }
    out << ',' << distribution(std::move(values)).p95;
  }
  out << ',' << cpu.size() << ',';
  if (!cpu.empty()) {
    out << distribution(std::move(cpu)).p95;
  }
  out << ',';
  if (!gap.empty()) {
    out << distribution(std::move(gap)).p95;
  }
  out << ',' << work.controller_overruns << ',' << work.post_controller_overruns;
  for (const auto number : work.outcomes) {
    out << ',' << number;
  }
  out << ',' << work.branches << ',' << work.evaluated << ',' << work.feasible << ','
      << work.fallbacks << ',' << work.reductions << ',' << config.horizon_steps << ','
      << config.samples_per_branch << ',' << config.iterations << ',' << config.random_seed << ','
      << span_rad << '\n';
  return work;
}
}  // namespace swerve_mppi::benchmark
