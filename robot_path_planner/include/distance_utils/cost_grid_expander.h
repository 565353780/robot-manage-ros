#ifndef VIRTUAL_SCAN_GEODESIC_DISTANCE_EXPAND_H_
#define VIRTUAL_SCAN_GEODESIC_DISTANCE_EXPAND_H_

#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <queue>
#include <unordered_map>

#include "grid_map/grid_map_2d.h"

namespace virtual_scan 
{
class CostGridExpander
{
  friend class GeodesicDistance;
public:
  CostGridExpander() : step_cost_(50.0), step_cost_diagonal_(50.0 * std::sqrt(2)), unknown_cost_(40.0)
  {
  }

  virtual ~CostGridExpander() = default;

  virtual void expand(const GridMap2D& grid_map, const Eigen::Vector2d& start_point,
                      const std::vector<Eigen::Vector2d>& end_points) = 0;
  [[nodiscard]] const std::vector<double>& getCosts() const
  {
    return cost_grid_;
  }
  [[nodiscard]] const std::unordered_map<std::size_t, std::size_t>& getExpandedTrace() const
  {
    return expanded_trace_;
  }

  void clear()
  {
    std::fill(cost_grid_.begin(), cost_grid_.end(), std::numeric_limits<double>::max());
    expanded_trace_.clear();
    node_queue_ = std::priority_queue<ExpandedNode, std::vector<ExpandedNode>, decltype(nodePriorityComparator)*>{
      nodePriorityComparator
    };
  }

protected:
  using ExpandedNode = std::pair<std::size_t, double>;
  const double step_cost_;
  const double step_cost_diagonal_;
  const double unknown_cost_;
  std::vector<double> cost_grid_;
  std::unordered_map<std::size_t, std::size_t> expanded_trace_;
  static bool nodePriorityComparator(const ExpandedNode& lhs, const ExpandedNode& rhs)
  {
    return lhs.second > rhs.second;
  }
  std::priority_queue<ExpandedNode, std::vector<ExpandedNode>, decltype(nodePriorityComparator)*> node_queue_{
    nodePriorityComparator
  };

  bool traverse_unknown_ = true;
};
}  // namespace co_scan
#endif
