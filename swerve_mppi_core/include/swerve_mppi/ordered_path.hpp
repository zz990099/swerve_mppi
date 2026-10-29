#pragma once
#include "swerve_mppi/path.hpp"

namespace swerve_mppi
{
// Ordered sections retain zero-distance heading changes as executable events.
// The same progress rules are used by portable callers and the Nav2 adapter.
class OrderedPath
{
public:
  void set(std::vector<Pose> path)
  {
    if (path.empty() || path.size() > 4096) throw std::invalid_argument("empty or oversized path");
    for (auto p : path)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw))
        throw std::invalid_argument("invalid path");
    sections_.clear();
    std::vector<Pose> translation{path.front()};
    for (std::size_t i = 1; i < path.size(); ++i) {
      if (distance(path[i - 1], path[i]) < 1e-8) {
        if (std::abs(wrap_angle(path[i].yaw - path[i - 1].yaw)) < 1e-8) continue;
        if (translation.size() > 1) sections_.push_back({translation, false});
        sections_.push_back({{path[i - 1], path[i]}, true});
        translation = {path[i]};
      } else {
        translation.push_back(path[i]);
      }
    }
    if (translation.size() > 1 || sections_.empty()) sections_.push_back({translation, false});
    section_ = progress_ = 0;
    progress_arc_ = 0;
  }
  bool empty() const { return sections_.empty(); }
  bool goal_active(double terminal_distance = .5) const
  {
    return !empty() && section_ + 1 == sections_.size() && remaining() <= terminal_distance;
  }
  bool tracking_goal_active(double terminal_distance = .5) const
  {
    return !empty() &&
           PathView(sections_[section_].points).length() - progress_arc_ <= terminal_distance;
  }
  double remaining() const
  {
    if (empty()) return 0;
    double length = -progress_arc_;
    for (std::size_t i = section_; i < sections_.size(); ++i)
      length += PathView(sections_[i].points).length();
    return length;
  }
  Pose goal() const { return sections_.back().points.back(); }
  Pose tracking_goal() const { return sections_[section_].points.back(); }
  bool rotating() const { return !empty() && sections_[section_].rotation; }
  std::size_t section() const { return section_; }
  std::vector<Pose> local(const Pose & pose, double length = 3.0, Velocity velocity = {})
  {
    // Advance only after measured arrival and stopping. A rotation must attain
    // its yaw before a later translation or rotation becomes visible to MPPI.
    while (section_ + 1 < sections_.size()) {
      const auto & current = sections_[section_];
      const auto end = current.points.back();
      const bool stopped =
        std::hypot(velocity.vx, velocity.vy) < .035 && std::abs(velocity.wz) < .035;
      const bool aligned = !current.rotation || std::abs(wrap_angle(pose.yaw - end.yaw)) < .03;
      if (!stopped || !aligned || distance(pose, end) >= .015) break;
      ++section_;
      progress_ = 0;
      progress_arc_ = 0;
    }
    const auto & points = sections_[section_].points;
    if (rotating()) return points;
    PathView view(points);
    while (progress_ + 1 < points.size()) {
      const auto & a = points[progress_];
      const auto & b = points[progress_ + 1];
      double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
      double dot = (pose.x - a.x) * dx + (pose.y - a.y) * dy;
      double cross = (pose.x - a.x) * dy - (pose.y - a.y) * dx;
      bool passed = l2 < 1e-12 || (dot >= l2 && cross * cross <= .03 * .03 * l2);
      bool corner = view.turn(progress_ + 1) >= PathView::stop_angle && distance(pose, b) < .015;
      if (!passed && !corner) break;
      progress_arc_ = view.vertex_arc(progress_ + 1);
      ++progress_;
    }
    double before = view.vertex_arc(progress_);
    double next = progress_ + 1 < points.size() ? view.vertex_arc(progress_ + 1) : before;
    auto projected = view.match(pose, before, next);
    progress_arc_ = std::max(progress_arc_, projected.arc);
    double end = std::min(view.length(), progress_arc_ + length);
    std::vector<Pose> out{view.at(progress_arc_)};
    for (std::size_t i = 1; i < points.size(); ++i) {
      double arc = view.vertex_arc(i);
      if (arc > progress_arc_ + 1e-8 && arc < end - 1e-8) out.push_back(points[i]);
    }
    if (end > progress_arc_ + 1e-8) out.push_back(view.at(end));
    return out;
  }

private:
  struct Section
  {
    std::vector<Pose> points;
    bool rotation = false;
  };
  std::vector<Section> sections_;
  std::size_t section_ = 0, progress_ = 0;
  double progress_arc_ = 0;
};
}  // namespace swerve_mppi
