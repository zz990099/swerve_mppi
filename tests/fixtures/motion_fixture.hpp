#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "swerve_mppi/feedback/motion_observer.hpp"
#include "swerve_mppi/model/model.hpp"

namespace swerve_mppi::test
{
// Independent numerical plant. Model endpoints are actuator targets only;
// predicted pose/twist and core FK/integration never advance this plant.
struct MotionPerturbation
{
  const char * name;
  double wheel_lag_s = 0;
  double body_lag_s = 0;
  double slip_fraction = 0;
  double noise_mps = 0;
  double noise_radps = 0;
  bool delayed = false;
};
inline std::vector<MotionPerturbation> motion_perturbations()
{
  return {
    {"nominal"},
    {"wheel_lag", .15},
    {"body_lag", 0, .20},
    {"slip", 0, 0, .25},
    {"noise", 0, 0, 0, .005, .01},
    {"delayed", 0, 0, 0, 0, 0, true}};
}
class MotionPlant
{
public:
  MotionPlant(const Config & config, DriveMode mode, const MotionPerturbation & perturbation)
  : config_(config), perturbation_(perturbation)
  {
    state.actual_mode = mode;
    if (mode == DriveMode::Crab) {
      state.steering_angles.fill(std::acos(-1.0) / 2);
    } else if (mode == DriveMode::Spin) {
      const double a = std::atan2(config.wheelbase_m, config.track_m);
      state.steering_angles = {-a, a, a, -a};
    }
    state.time_in_mode_s = 2;
    synchronize();
  }
  Twist2d encoder_twist() const
  {
    Twist2d out;
    double moment = 0;
    for (std::size_t i = 0; i < 4; ++i) {
      const double x = (i < 2 ? 1 : -1) * config_.wheelbase_m / 2;
      const double y = (i % 2 == 0 ? 1 : -1) * config_.track_m / 2;
      const double vx = state.wheel_speeds[i] * std::cos(state.steering_angles[i]);
      const double vy = state.wheel_speeds[i] * std::sin(state.steering_angles[i]);
      out.vx += vx / 4;
      out.vy += vy / 4;
      out.wz += x * vy - y * vx;
      moment += x * x + y * y;
    }
    out.wz /= moment;
    return out;
  }
  // Generic independent affine target ramp, or explicit held-target interval.
  // Neither mode implements the chassis command generator.
  void advance(
    const std::array<double, 4> & angles, const std::array<double, 4> & speeds, int substeps = 512,
    bool held = false)
  {
    const auto before = state;
    const double h = config_.dt_s / substeps;
    for (int step = 0; step < substeps; ++step) {
      const double t = (step + .5) * h;
      const double angle_fraction = held ? 0 : t / config_.dt_s;
      const auto prior_wheels = state.wheel_speeds;
      for (std::size_t i = 0; i < 4; ++i) {
        state.steering_angles[i] =
          before.steering_angles[i] + (angles[i] - before.steering_angles[i]) * angle_fraction;
        const double target =
          held ? held_speeds_[i]
               : before.wheel_speeds[i] + (speeds[i] - before.wheel_speeds[i]) * t / config_.dt_s;
        if (perturbation_.wheel_lag_s > 0) {
          state.wheel_speeds[i] +=
            (target - state.wheel_speeds[i]) * (1 - std::exp(-h / perturbation_.wheel_lag_s));
        } else {
          state.wheel_speeds[i] = target;
        }
      }
      const auto end_wheels = state.wheel_speeds;
      if (perturbation_.wheel_lag_s > 0) {
        for (std::size_t i = 0; i < 4; ++i) {
          state.wheel_speeds[i] = (prior_wheels[i] + end_wheels[i]) / 2;
        }
      }
      auto physical = encoder_twist();
      state.wheel_speeds = end_wheels;
      physical.vx *= 1 - perturbation_.slip_fraction;
      physical.vy *= 1 - perturbation_.slip_fraction;
      physical.wz *= 1 - perturbation_.slip_fraction;
      const auto prior = body_velocity;
      if (perturbation_.body_lag_s > 0) {
        const double f = 1 - std::exp(-h / perturbation_.body_lag_s);
        body_velocity.vx += f * (physical.vx - body_velocity.vx);
        body_velocity.vy += f * (physical.vy - body_velocity.vy);
        body_velocity.wz += f * (physical.wz - body_velocity.wz);
        physical = {
          (prior.vx + body_velocity.vx) / 2, (prior.vy + body_velocity.vy) / 2,
          (prior.wz + body_velocity.wz) / 2};
      } else {
        body_velocity = physical;
      }
      const double yaw = state.pose.yaw + physical.wz * h / 2;
      state.pose.x += h * (std::cos(yaw) * physical.vx - std::sin(yaw) * physical.vy);
      state.pose.y += h * (std::sin(yaw) * physical.vx + std::cos(yaw) * physical.vy);
      state.pose.yaw = std::atan2(
        std::sin(state.pose.yaw + h * physical.wz), std::cos(state.pose.yaw + h * physical.wz));
    }
    held_speeds_ = speeds;
    state.steering_angles = angles;
    if (perturbation_.wheel_lag_s == 0) {
      state.wheel_speeds = speeds;
    }
    if (perturbation_.body_lag_s == 0) {
      body_velocity = encoder_twist();
      body_velocity.vx *= 1 - perturbation_.slip_fraction;
      body_velocity.vy *= 1 - perturbation_.slip_fraction;
      body_velocity.wz *= 1 - perturbation_.slip_fraction;
    }
    stamp_ns += static_cast<std::int64_t>(std::llround(config_.dt_s * 1e9));
    synchronize();
  }
  VehicleState state;
  Twist2d body_velocity;
  std::int64_t stamp_ns = 1000000000;

private:
  void synchronize()
  {
    state.velocity = encoder_twist();
    state.stamp_s = std::chrono::duration<double>(std::chrono::nanoseconds(stamp_ns)).count();
  }
  std::array<double, 4> held_speeds_{};
  Config config_;
  MotionPerturbation perturbation_;
};
struct MotionProbeRow
{
  DriveMode mode;
  std::string perturbation;
  int tick;
  bool braking;
  std::int64_t encoder_stamp_ns;
  VehicleState encoded;
  Twist2d physical_velocity;
  MotionObservation observed;
  MotionAssessment assessment;
  double prediction_position_error_m;
  double prediction_yaw_error_rad;
  bool encoded_model_valid;
  bool observed_model_valid;
};
inline std::vector<MotionProbeRow> run_motion_probe(const Config & c)
{
  validate(c);
  if (c.dt_s > 1 || c.dt_s < .001) {
    throw std::invalid_argument("Probe requires dt_s in [0.001,1]");
  }
  if (std::abs(c.dt_s / c.chassis_period_s - std::round(c.dt_s / c.chassis_period_s)) > 1e-9) {
    throw std::invalid_argument(
      "Probe requires an integer number of chassis periods per model tick");
  }
  const auto tick_ns = static_cast<std::int64_t>(std::llround(c.dt_s * 1e9));
  if (std::chrono::duration<double>(std::chrono::nanoseconds(tick_ns)).count() != c.dt_s) {
    throw std::invalid_argument("Probe dt_s must represent an exact integer nanosecond period");
  }
  std::vector<MotionProbeRow> rows;
  MotionObserver observer(c);
  DriveModel model(c);
  for (DriveMode mode : {DriveMode::DualAckermann, DriveMode::Crab, DriveMode::Spin}) {
    const Control drive = mode == DriveMode::DualAckermann ? Control{.4, 0, 0}
                          : mode == DriveMode::Crab        ? Control{0, .4, 0}
                                                           : Control{0, 0, .5};
    for (const auto & p : motion_perturbations()) {
      Config micro = c;
      micro.dt_s = c.chassis_period_s;
      MotionPlant plant(micro, mode, p);
      auto memory = model.seed(plant.state);
      MotionObservation previous{
        plant.body_velocity, plant.stamp_ns, MotionSource::IndependentBody};
      for (int tick = 0; tick < 40; ++tick) {
        const bool braking = tick >= 20;
        const auto predicted = model.step(plant.state, braking ? Control{} : drive, c.dt_s, memory);
        if (!predicted.valid) {
          throw std::runtime_error("Probe intent is incompatible with resolved configuration");
        }
        // Independently apply each command endpoint to a sample-and-hold plant.
        const auto ticks = static_cast<std::size_t>(std::llround(c.dt_s / c.chassis_period_s));
        for (std::size_t substep = 0; substep < ticks; ++substep) {
          const auto target =
            model.step(plant.state, braking ? Control{} : drive, c.chassis_period_s, memory);
          if (!target.valid) throw std::runtime_error("invalid probe command prediction");
          memory = target.prediction;
          plant.advance(target.steering_targets, target.wheel_speed_targets, 64, true);
        }
        MotionObservation current{
          plant.body_velocity, plant.stamp_ns, MotionSource::IndependentBody};
        // Deterministic bounded alternating measurement disturbance, not MPPI proposal noise.
        const double sign = tick % 2 ? -1 : 1;
        current.velocity.vx += sign * p.noise_mps;
        current.velocity.wz += sign * p.noise_radps;
        current.linear_error_bound_mps = p.noise_mps;
        current.angular_error_bound_radps = p.noise_radps;
        const auto observed = p.delayed ? previous : current;
        const auto assessment = observer.assess(plant.state, plant.stamp_ns, observed);
        auto independent_state = plant.state;
        independent_state.velocity = observed.velocity;
        rows.push_back(
          {mode, p.name, tick, braking, plant.stamp_ns, plant.state, plant.body_velocity, observed,
           assessment,
           std::hypot(
             predicted.state.pose.x - plant.state.pose.x,
             predicted.state.pose.y - plant.state.pose.y),
           std::abs(angle_distance(predicted.state.pose.yaw, plant.state.pose.yaw)),
           check_model_feedback(plant.state, c).status == FeedbackStatus::Valid,
           !p.delayed &&
             check_model_feedback(independent_state, c).status == FeedbackStatus::Valid});
        previous = current;
      }
    }
  }
  return rows;
}
}  // namespace swerve_mppi::test
