#include <limits>
#include <memory>
#include <unordered_map>
#include <algorithm>

#include "distance_utils/dijkstra_expander.h"
#include "distance_utils/geodesic_distance.h"
namespace virtual_scan
{
double GeodesicDistance::compute(const GridMap2D& grid_map, const Eigen::Vector2d& start_point,
                                 const Eigen::Vector2d& end_point, bool use_cache)
{
  if (use_cache)
  {
    dijkstra_expander_.expand(grid_map, start_point, std::vector<Eigen::Vector2d>{ end_point });
    expanded_trace_ = &dijkstra_expander_.getExpandedTrace();
    return dijkstra_expander_.getCosts()[grid_map.getIndex(end_point)] / dijkstra_expander_.step_cost_;
  }
  else
  {
    astar_expander_.expand(grid_map, start_point, std::vector<Eigen::Vector2d>{ end_point });
    expanded_trace_ = &astar_expander_.getExpandedTrace();
    return astar_expander_.getCosts()[grid_map.getIndex(end_point)] / dijkstra_expander_.step_cost_;
  }
}

std::vector<double> GeodesicDistance::compute(const GridMap2D& grid_map, const Eigen::Vector2d& start_point,
                                              const std::vector<Eigen::Vector2d>& end_points, bool use_cache)
{
  if (!use_cache)
  {
    dijkstra_expander_.clear();
  }
  dijkstra_expander_.expand(grid_map, start_point, end_points);

  std::vector<double> results;
  results.resize(end_points.size());
  std::transform(end_points.cbegin(), end_points.cend(), results.begin(), [&](auto end_point) {
    return dijkstra_expander_.getCosts()[grid_map.getIndex(end_point)] / dijkstra_expander_.step_cost_;
  });
  expanded_trace_ = &dijkstra_expander_.getExpandedTrace();
  return results;
}

}  // namespace virtual_scan
