#pragma once
#include <ostream>

#include "swerve_mppi/types.hpp"
namespace swerve_mppi_ros
{
inline void decision_header(std::ostream & out)
{
  out << "t,iteration,candidate,selected,valid,rejection,reference_speed,reference_limit,noise_"
         "scale,reorientation,alignment_error,terminal_hold,"
         "seed_vx,seed_vy,seed_wz,seed_conditioned_vx,seed_conditioned_vy,seed_conditioned_wz,seed_"
         "peak,seed_conditioned_peak,"
         "raw_vx,raw_vy,raw_wz,conditioned_vx,conditioned_vy,conditioned_wz,raw_peak,conditioned_"
         "peak,predicted_progress,xy_motion,yaw_motion,"
         "cost,path,heading,velocity,alignment,effort,progress_cost,goal,"
         "x,y,yaw,measured_vx,measured_vy,measured_wz,request_vx,request_vy,request_wz,"
         "steer0,steer1,steer2,steer3,target0,target1,target2,target3\n";
}
inline void decision_rows(
  std::ostream & out, double time, const swerve_mppi::Input & input,
  const swerve_mppi::Result & result)
{
  const auto velocity = [&](swerve_mppi::Velocity v) {
    out << ',' << v.vx << ',' << v.vy << ',' << v.wz;
  };
  for (const auto & d : result.decisions) {
    out << time << ',' << d.iteration << ',' << d.candidate << ',' << d.selected << ',' << d.valid
        << ',' << static_cast<int>(d.rejection) << ',' << result.reference_speed << ','
        << result.reference_limit << ',' << result.noise_scale << ',' << result.reorientation_active
        << ',' << result.alignment_error << ',' << result.terminal_hold;
    velocity(result.seeds[0].desired_first);
    velocity(result.seeds[0].conditioned_first);
    out << ',' << result.seeds[0].desired_peak << ',' << result.seeds[0].conditioned_peak;
    velocity(d.raw_first);
    velocity(d.conditioned_first);
    out << ',' << d.raw_peak << ',' << d.conditioned_peak << ',' << d.predicted_progress << ','
        << d.xy_motion << ',' << d.yaw_motion << ',' << d.cost << ',' << d.tracking.path << ','
        << d.tracking.heading << ',' << d.tracking.velocity << ',' << d.tracking.alignment << ','
        << d.tracking.effort << ',' << d.tracking.progress << ',' << d.tracking.goal << ','
        << input.state.pose.x << ',' << input.state.pose.y << ',' << input.state.pose.yaw;
    velocity(input.state.velocity);
    velocity(input.state.requested);
    for (double a : input.state.steering) out << ',' << a;
    for (double a : input.state.generator.command.steering) out << ',' << a;
    out << '\n';
  }
}
}  // namespace swerve_mppi_ros
