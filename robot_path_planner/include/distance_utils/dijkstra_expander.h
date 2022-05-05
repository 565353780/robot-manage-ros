#ifndef VIRTUAL_SCAN_GEODESIC_DISTANCE_DIJKSTRA_EXPANSION_H_
#define VIRTUAL_SCAN_GEODESIC_DISTANCE_DIJKSTRA_EXPANSION_H_

#include <vector>
#include <memory>
#include <unordered_set>
#include "distance_utils/cost_grid_expander.h"

namespace virtual_scan
{
class DijkstraExpander : public CostGridExpander
{
public:
  void expand(const GridMap2D& grid_map, const Eigen::Vector2d& start_point,
              const std::vector<Eigen::Vector2d>& end_points) override;

private:
  bool needRestart(const GridMap2D& grid_map, std::size_t start_index);
  void addNode(const GridMap2D& grid_map, std::size_t prev_index, std::size_t next_index, bool is_vertical);
};
}  // namespace virtual_scan
#endif
