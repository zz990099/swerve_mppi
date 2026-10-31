#pragma once
#include <optional>

#include "swerve_mppi/heading_reference.hpp"
#include "swerve_mppi/reference.hpp"

namespace swerve_mppi
{
// Ordered sections retain zero-distance heading changes as executable events.
// The same progress rules are used by portable callers and the Nav2 adapter.
class OrderedPath
{
public:
  void set(
    std::vector<Pose> path, double lookahead = .3, ReferenceKind kind = ReferenceKind::ExternalPath,
    double heading_preview = 2.0, double heading_turn = .50)
  {
    if (path.empty() || path.size() > 4096) throw std::invalid_argument("empty or oversized path");
    for (auto p : path)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw))
        throw std::invalid_argument("invalid path");
    final_goal_ = path.back();
    if (!std::isfinite(lookahead) || lookahead <= 0)
      throw std::invalid_argument("invalid lookahead");
    if (kind == ReferenceKind::ExternalPath) path = geometric_path(path, lookahead);
    sections_.clear();
    std::vector<Pose> translation{path.front()};
    for (std::size_t i = 1; i < path.size(); ++i) {
      if (distance(path[i - 1], path[i]) < 1e-8) {
        if (std::abs(wrap_angle(path[i].yaw - path[i - 1].yaw)) < 1e-8) continue;
        if (translation.size() > 1) sections_.push_back({translation, false, {}});
        sections_.push_back({{path[i - 1], path[i]}, true, {}});
        translation = {path[i]};
      } else {
        translation.push_back(path[i]);
      }
    }
    if (translation.size() > 1 || sections_.empty()) sections_.push_back({translation, false, {}});
    Config context;
    context.heading_preference_distance = heading_preview;
    context.heading_preference_turn = heading_turn;
    // Build on each complete ordered section, before local cropping. Runtime
    // queries use this fixed spatial context rather than the moving crop end.
    for (auto & section : sections_) {
      ReferenceProfile profile(section.points, context);
      PathView view(section.points);
      for (std::size_t i = 0; i < section.points.size(); ++i)
        section.preference.push_back(profile.heading_preference(view.vertex_arc(i)));
    }
    local_preference_.clear();
    section_ = 0;
    progress_arc_ = 0;
    travel_budget_ = .03;
    previous_pose_.reset();
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
  Pose goal() const { return final_goal_; }
  Pose tracking_goal() const
  {
    return section_ + 1 == sections_.size() ? final_goal_ : sections_[section_].points.back();
  }
  bool rotating() const { return !empty() && sections_[section_].rotation; }
  std::size_t section() const { return section_; }
  const std::vector<double> & heading_preferences() const { return local_preference_; }
  // A completed bounded steering maneuver may follow a short chord instead
  // of every sampled tangent. Rejoin only this section's nearby ordered arc;
  // never cross an unreached mandatory corner or a rotation section.
  void recover(const Pose & pose)
  {
    if (empty() || rotating()) return;
    const auto & points = sections_[section_].points;
    PathView view(points);
    double hi = std::min(view.length(), progress_arc_ + .35);
    const double corner = view.next_corner(progress_arc_);
    if (corner < view.length() - 1e-5 && distance(pose, view.at(corner)) > .015)
      hi = std::min(hi, std::max(progress_arc_, corner - 1e-4));
    const auto match = view.match(pose, progress_arc_, hi);
    if (match.error < .04) {
      travel_budget_ = std::max(0., travel_budget_ - (match.arc - progress_arc_));
      progress_arc_ = std::max(progress_arc_, match.arc);
    }
  }
  std::vector<Pose> local(const Pose & pose, double length = 3.0, Velocity velocity = {})
  {
    if (previous_pose_) {
      const double travel = distance(pose, *previous_pose_);
      travel_budget_ = std::min(std::max(.35, 1.5 * travel), travel_budget_ + 1.5 * travel);
    }
    previous_pose_ = pose;
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
      progress_arc_ = 0;
      travel_budget_ = .03;
    }
    const auto & points = sections_[section_].points;
    if (rotating()) {
      local_preference_ = sections_[section_].preference;
      return points;
    }
    PathView view(points);
    // A missed sampled tangent must not freeze progress on a smooth curve.
    // The forward search is paid for by measured displacement and cannot pass
    // an unreached mandatory corner; nearby future loops remain outside it.
    double corner = view.next_corner(progress_arc_);
    if (
      corner < view.length() - 1e-5 && distance(pose, view.at(corner)) < .015 &&
      std::hypot(velocity.vx, velocity.vy) < .04) {
      travel_budget_ = std::max(0., travel_budget_ - (corner - progress_arc_));
      progress_arc_ = corner;
    }
    double hi = std::min(view.length(), progress_arc_ + travel_budget_);
    corner = view.next_corner(progress_arc_);
    if (corner < view.length() - 1e-5 && distance(pose, view.at(corner)) > .015)
      hi = std::min(hi, std::max(progress_arc_, corner - 1e-4));
    const auto projected = view.match(pose, progress_arc_, hi);
    if (projected.error < .04) {
      travel_budget_ = std::max(0., travel_budget_ - (projected.arc - progress_arc_));
      progress_arc_ = std::max(progress_arc_, projected.arc);
    }
    double end = std::min(view.length(), progress_arc_ + length);
    std::vector<Pose> out{view.at(progress_arc_)};
    local_preference_ = {preference_at(view, progress_arc_)};
    for (std::size_t i = 1; i < points.size(); ++i) {
      double arc = view.vertex_arc(i);
      if (arc > progress_arc_ + 1e-8 && arc < end - 1e-8) {
        out.push_back(points[i]);
        local_preference_.push_back(sections_[section_].preference[i]);
      }
    }
    if (end > progress_arc_ + 1e-8) {
      out.push_back(view.at(end));
      local_preference_.push_back(preference_at(view, end));
    }
    return out;
  }

private:
  struct Section
  {
    std::vector<Pose> points;
    bool rotation = false;
    std::vector<double> preference;
  };
  double preference_at(const PathView & view, double arc) const
  {
    const auto & weights = sections_[section_].preference;
    for (std::size_t i = 1; i < weights.size(); ++i)
      if (view.vertex_arc(i) >= arc) {
        const double a = view.vertex_arc(i - 1), b = view.vertex_arc(i);
        const double t = b > a ? std::clamp((arc - a) / (b - a), 0., 1.) : 0;
        return weights[i - 1] + t * (weights[i] - weights[i - 1]);
      }
    return weights.back();
  }
  std::vector<double> local_preference_;
  Pose final_goal_;
  std::vector<Section> sections_;
  std::size_t section_ = 0;
  double progress_arc_ = 0, travel_budget_ = .03;
  std::optional<Pose> previous_pose_;
};
}  // namespace swerve_mppi
