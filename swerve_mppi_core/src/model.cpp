#include "swerve_mppi/model.hpp"
#include <algorithm>
#include <cmath>
namespace swerve_mppi
{
namespace chassis = swerve_chassis_core;
Model::Model(Config config) : config_(config), generator_(config.geometry, config.limits)
{
  validate(config_);
}
Velocity Model::project(Velocity v) const
{
  if (!chassis::finite(chassis::Twist{v.vx,v.vy,v.wz})) return {};
  double ratio = std::min({1.0, config_.max_speed / std::max(1e-12, std::hypot(v.vx,v.vy)),
    config_.max_yaw_rate / std::max(1e-12,std::abs(v.wz))});
  // A zero translation cap still permits an in-place rotation.
  if (std::hypot(v.vx,v.vy) < 1e-12) ratio = std::min(1.0,config_.max_yaw_rate/std::max(1e-12,std::abs(v.wz)));
  auto t = chassis::scale({v.vx,v.vy,v.wz},ratio);
  auto wheels = chassis::inverse(config_.geometry,t);
  ratio = 1;
  for(double speed:wheels.speed) ratio=std::min(ratio,config_.limits.wheel_speed/std::max(1e-12,std::abs(speed)));
  return {t.vx*ratio,t.vy*ratio,t.wz*ratio};
}
bool Model::valid(const State & s) const
{
  if (!s.generator.valid || !std::isfinite(s.pose.x) || !std::isfinite(s.pose.y) || !std::isfinite(s.pose.yaw) ||
      !chassis::finite(chassis::Twist{s.velocity.vx,s.velocity.vy,s.velocity.wz}) ||
      !chassis::finite(chassis::Joints{s.steering,s.wheel_speeds}) || !chassis::finite(s.generator.command)) return false;
  for(std::size_t i=0;i<4;++i) {
    if(std::abs(s.steering[i])>config_.limits.steering+0.03 ||
       std::abs(s.generator.command.steering[i])>config_.limits.steering+1e-9 ||
       std::abs(s.generator.command.speed[i])>config_.limits.wheel_speed+1e-9) return false;
  }
  return chassis::residual(config_.geometry,s.generator.command)<1e-7 &&
    chassis::residual(config_.geometry,{s.steering,s.wheel_speeds})<=config_.max_residual;
}
Trajectory Model::rollout(const State & initial,const std::vector<Velocity> & controls) const
{
  Trajectory out; out.controls=controls; out.states.reserve(controls.size()+1);
  out.states.push_back(initial);
  if(!valid(initial)) {out.valid=false;return out;}
  State s=initial;
  int ticks=static_cast<int>(std::ceil(config_.model_dt/config_.integration_dt));
  const double dt=config_.model_dt/ticks;
  try {
    for(auto & u:out.controls) {
      if(!chassis::finite(chassis::Twist{u.vx,u.vy,u.wz})) {out.valid=false;return out;}
      u=project(u);
      for(int tick=0;tick<ticks;++tick) {
        const chassis::Joints measured{s.steering,s.wheel_speeds};
        auto result=generator_.step({u.vx,u.vy,u.wz},measured,s.generator,dt);
        auto next=chassis::response(measured,result.state.command,config_.limits,dt);
        const double residual=chassis::residual(config_.geometry,next);
        if(!std::isfinite(residual) || residual>config_.max_residual) {out.valid=false;return out;}
        // Integrate measured nominal velocity before advancing actuator response.
        const auto body=chassis::forward(config_.geometry,measured);
        auto p=chassis::integrate({s.pose.x,s.pose.y,s.pose.yaw},body,dt);
        s.pose={p.x,p.y,p.yaw};
        for(std::size_t i=0;i<4;++i) {
          out.actuator_cost+=dt*(std::pow((next.steering[i]-s.steering[i])/(dt*config_.limits.steering_rate),2)+
            std::pow((next.speed[i]-s.wheel_speeds[i])/(dt*config_.limits.wheel_acceleration),2))/4;
        }
        out.residual_cost+=dt*std::pow(residual/config_.max_residual,2);
        s.steering=next.steering;s.wheel_speeds=next.speed;s.generator=result.state;
        auto v=chassis::forward(config_.geometry,next);s.velocity={v.vx,v.vy,v.wz};
      }
      out.states.push_back(s);
    }
  } catch(const std::exception &) {out.valid=false;}
  return out;
}
}
