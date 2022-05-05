#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

#include "distance_utils/astar_expander.h"

namespace virtual_scan
{
void AStarExpander::expand(const GridMap2D& grid_map, const Eigen::Vector2d& start_point,
                           const std::vector<Eigen::Vector2d>& end_points)
{
  std::size_t grid_size = grid_map.getSize();
  if (cost_grid_.size() != grid_size)
  {
    cost_grid_.resize(grid_size);
  }
  clear();

  std::size_t start_index = grid_map.getIndex(start_point);
  auto end_point = end_points[0];  // actually, end_points is 1-size vector when using astar
  std::size_t goal_index = grid_map.getIndex(end_point);
  node_queue_.emplace(start_index, 0.0);
  cost_grid_[start_index] = 0;

  std::size_t iteration_threshold = grid_size * 2;
  std::size_t iteration_counter = 0;
  while (!node_queue_.empty() && iteration_counter < iteration_threshold)
  {
    ExpandedNode expanding_node = node_queue_.top();
    node_queue_.pop();

    std::size_t expanding_index = expanding_node.first;

    if (expanding_index == goal_index)
    {
      return;
    }
    if (!grid_map.checkWithinBound(expanding_index))
      continue;

    for (int dx = -1; dx < 2; ++dx)
    {
      for (int dy = -1; dy < 2; ++dy)
      {
        if (0 == dx && 0 == dy)
          continue;
        bool is_vertical = (abs(dx) + abs(dy) != 2);
        auto next_expanding_index = grid_map.getNeighborIndex(expanding_index, dy, dx);
        if (!next_expanding_index)
          continue;

        addNode(grid_map, expanding_index, *next_expanding_index, end_point, is_vertical);
      }
    }
    ++iteration_counter;
  }
}

void AStarExpander::addNode(const GridMap2D& grid_map, std::size_t prev_index, std::size_t next_index,
                            const Eigen::Vector2d& end_point, bool is_vertical)
{
  Eigen::Vector2d current_point = grid_map.getCoordinate(next_index);
  if (grid_map.isObstacle(next_index) || (grid_map.isUnknown(next_index) && !traverse_unknown_))
  {
    return;
  }

  double step_cost = (is_vertical ? step_cost_ : step_cost_diagonal_);
  double new_cost =
      cost_grid_[prev_index] + step_cost + static_cast<int>(grid_map.isUnknown(next_index)) * unknown_cost_;
  if (cost_grid_[next_index] <= new_cost)
  {
    return;
  }
  double manhattan_distance = (end_point - current_point).lpNorm<1>();
  cost_grid_[next_index] = new_cost;
  node_queue_.emplace(next_index, cost_grid_[next_index] + manhattan_distance * step_cost_);
  expanded_trace_[next_index] = prev_index;
}

}  // namespace virtual_scan
