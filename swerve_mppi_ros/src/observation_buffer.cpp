#include "swerve_mppi_ros/observation_buffer.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

namespace swerve_mppi_ros
{
namespace
{
template <class Message>
void insert(
  std::map<swerve_mppi::TimestampNs, Message> & queue, const Message & msg,
  const Message * consumed)
{
  const auto stamp = source_time(msg.header.stamp);
  if (consumed) {
    const auto last = source_time(consumed->header.stamp);
    if (stamp == last && msg != *consumed) {
      throw std::invalid_argument("conflicting duplicate source observation");
    }
    if (stamp <= last) return;
  }
  const auto [iterator, added] = queue.emplace(stamp, msg);
  if (!added && iterator->second != msg) {
    throw std::invalid_argument("conflicting duplicate source observation");
  }
  if (queue.size() > 32) queue.erase(queue.begin());
}
}  // namespace

void ObservationBuffer::push(const swerve_gazebo_sim::msg::ChassisState & msg)
{
  insert(states_, msg, consumed_ ? &consumed_->first : nullptr);
}
void ObservationBuffer::push(const nav_msgs::msg::Odometry & msg)
{
  insert(odometry_, msg, consumed_ ? &consumed_->second : nullptr);
}
std::optional<ObservationBuffer::Pair> ObservationBuffer::take(
  swerve_mppi::TimestampNs tolerance_ns)
{
  for (auto state = states_.rbegin(); state != states_.rend(); ++state) {
    for (auto odom = odometry_.rbegin(); odom != odometry_.rend(); ++odom) {
      if (std::abs(state->first - odom->first) > tolerance_ns) continue;
      consumed_ = Pair{state->second, odom->second};
      states_.erase(states_.begin(), states_.upper_bound(state->first));
      odometry_.erase(odometry_.begin(), odometry_.upper_bound(odom->first));
      return consumed_;
    }
  }
  return std::nullopt;
}
}  // namespace swerve_mppi_ros
