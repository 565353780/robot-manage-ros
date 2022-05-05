#ifndef VIRTUAL_SCAN_GEODESIC_DISTANCE_GEODESIC_DISTANCE_CALCULATOR_H_
#define VIRTUAL_SCAN_GEODESIC_DISTANCE_GEODESIC_DISTANCE_CALCULATOR_H_

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <memory>
#include <unordered_map>

#include "grid_map/grid_map_2d.h"
#include "distance_utils/astar_expander.h"
#include "distance_utils/dijkstra_expander.h"

namespace virtual_scan
{
class GeodesicDistance
{
public:
  // GeodesicDistance()
  double compute(const GridMap2D& grid_map, const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point,
                 bool use_cache = false);
  // 1 to N distances

  std::vector<double> compute(const GridMap2D& grid_map, const Eigen::Vector2d& start_point,
                              const std::vector<Eigen::Vector2d>& end_points, bool use_cache = false);

  const std::unordered_map<std::size_t, std::size_t>& getExpandedTrace() const
  {
    return *expanded_trace_;
  }

  double extractPath(const GridMap2D& grid_map, const std::unordered_map<std::size_t, std::size_t>& expanded_trace,
                     const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point);

private:
  DijkstraExpander dijkstra_expander_;
  AStarExpander astar_expander_;
  const std::unordered_map<std::size_t, std::size_t>* expanded_trace_;
};

}  // namespace virtual_scan

#endif
