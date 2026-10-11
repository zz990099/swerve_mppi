#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "common/detail/geometry.hpp"

namespace swerve_mppi::detail
{
struct Bounds
{
  double xmin, ymin, xmax, ymax;
  static Bounds segment(const Pose2d & a, const Pose2d & b, double margin = 0)
  {
    const double inf = std::numeric_limits<double>::infinity();
    return {
      finite_geometry(std::nextafter(std::min(a.x, b.x) - margin, -inf)),
      finite_geometry(std::nextafter(std::min(a.y, b.y) - margin, -inf)),
      finite_geometry(std::nextafter(std::max(a.x, b.x) + margin, inf)),
      finite_geometry(std::nextafter(std::max(a.y, b.y) + margin, inf))};
  }
  bool overlaps(const Bounds & b) const
  {
    return xmin <= b.xmax && xmax >= b.xmin && ymin <= b.ymax && ymax >= b.ymin;
  }
  double distance(const Pose2d & p) const
  {
    return finite_geometry(
      std::hypot(std::max({xmin - p.x, 0.0, p.x - xmax}), std::max({ymin - p.y, 0.0, p.y - ymax})));
  }
};
// Immutable after construction. Entries always retain original indices; no
// resampling, truncation or approximate nearest-neighbour lookup is performed.
class SpatialIndex
{
public:
  explicit SpatialIndex(std::vector<Bounds> bounds) : bounds_(std::move(bounds))
  {
    order_.resize(bounds_.size());
    std::iota(order_.begin(), order_.end(), 0);
    nodes_.reserve(bounds_.size() * 2);
    if (!order_.empty()) {
      build(0, order_.size());
    }
  }
  template <class Visitor>
  bool visit(const Bounds & query, Visitor visitor) const
  {
    return nodes_.empty() || visit_node(0, query, visitor);
  }
  template <class Visitor>
  void nearest(const Pose2d & point, double & best, Visitor visitor) const
  {
    if (!nodes_.empty()) {
      nearest_node(0, point, best, visitor);
    }
  }

private:
  struct Node
  {
    Bounds bounds;
    std::size_t begin, end, left = 0, right = 0;
  };
  std::size_t build(std::size_t begin, std::size_t end)
  {
    Bounds b = bounds_[order_[begin]];
    for (std::size_t i = begin + 1; i < end; ++i) {
      const auto & a = bounds_[order_[i]];
      b = {
        std::min(b.xmin, a.xmin), std::min(b.ymin, a.ymin), std::max(b.xmax, a.xmax),
        std::max(b.ymax, a.ymax)};
    }
    const auto index = nodes_.size();
    nodes_.push_back({b, begin, end});
    if (end - begin > 4) {
      const bool x = b.xmax - b.xmin >= b.ymax - b.ymin;
      const auto middle = begin + (end - begin) / 2;
      std::nth_element(
        order_.begin() + begin, order_.begin() + middle, order_.begin() + end,
        [&](std::size_t a, std::size_t c) {
          const auto & u = bounds_[a];
          const auto & v = bounds_[c];
          return x ? u.xmin / 2 + u.xmax / 2 < v.xmin / 2 + v.xmax / 2
                   : u.ymin / 2 + u.ymax / 2 < v.ymin / 2 + v.ymax / 2;
        });
      const auto left = build(begin, middle);
      const auto right = build(middle, end);
      nodes_[index].left = left;
      nodes_[index].right = right;
    }
    return index;
  }
  template <class Visitor>
  bool visit_node(std::size_t index, const Bounds & query, Visitor & visitor) const
  {
    const auto & n = nodes_[index];
    if (!n.bounds.overlaps(query)) {
      return true;
    }
    if (n.left) {
      return visit_node(n.left, query, visitor) && visit_node(n.right, query, visitor);
    }
    for (std::size_t i = n.begin; i < n.end; ++i) {
      if (bounds_[order_[i]].overlaps(query) && !visitor(order_[i])) {
        return false;
      }
    }
    return true;
  }
  template <class Visitor>
  void nearest_node(std::size_t index, const Pose2d & p, double & best, Visitor & visitor) const
  {
    const auto & n = nodes_[index];
    // Retain a roundoff allowance, including exact-distance ties. Earliest
    // ordered segments retain their original yaw interpolation semantics.
    if (n.bounds.distance(p) > best + 1e-12 * (1 + best)) {
      return;
    }
    if (n.left) {
      const bool left_first =
        nodes_[n.left].bounds.distance(p) <= nodes_[n.right].bounds.distance(p);
      nearest_node(left_first ? n.left : n.right, p, best, visitor);
      nearest_node(left_first ? n.right : n.left, p, best, visitor);
    } else {
      for (std::size_t i = n.begin; i < n.end; ++i) {
        visitor(order_[i], best);
      }
    }
  }
  std::vector<Bounds> bounds_;
  std::vector<std::size_t> order_;
  std::vector<Node> nodes_;
};
inline SpatialIndex obstacle_index(const std::vector<CircleObstacle> & obstacles)
{
  std::vector<Bounds> bounds;
  bounds.reserve(obstacles.size());
  for (const auto & o : obstacles) {
    bounds.push_back(Bounds::segment({o.x, o.y, 0}, {o.x, o.y, 0}, o.radius));
  }
  return SpatialIndex(std::move(bounds));
}
}  // namespace swerve_mppi::detail
