#include "distance_utils/dijkstra_expander.h"

namespace virtual_scan
{
void DijkstraExpander::expand(const GridMap2D& grid_map, const Eigen::Vector2d& start_point,
                              const std::vector<Eigen::Vector2d>& end_points)
{
  std::size_t start_index = grid_map.getIndex(start_point);
  if (needRestart(grid_map, start_index))
  {
    clear();
    node_queue_.emplace(start_index, 0.0);
    cost_grid_[start_index] = 0.0;
  }

  std::unordered_set<std::size_t> goal_indexes;
  for (const auto& end_point : end_points)
  {
    std::size_t goal_index = grid_map.getIndex(end_point);
    if (cost_grid_[goal_index] >= std::numeric_limits<double>::max())
    {
      goal_indexes.insert(grid_map.getIndex(end_point));
    }
  }
  std::size_t remained_goals_num = end_points.size();

  std::size_t iteration_threshold = grid_map.getSize() * 2;
  std::size_t iteration_counter = 0;
  while (!node_queue_.empty() && iteration_counter < iteration_threshold)
  {
    if (remained_goals_num == 0)
    {
      return;
    }
    ExpandedNode expanding_node = node_queue_.top();
    node_queue_.pop();

    std::size_t expanding_index = expanding_node.first;

    if (goal_indexes.find(expanding_index) != goal_indexes.end())  //! can use '.contains()' if using C++20
    {
      --remained_goals_num;
    }

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

        addNode(grid_map, expanding_index, *next_expanding_index, is_vertical);
      }
    }
    ++iteration_counter;
  }
}

bool DijkstraExpander::needRestart(const GridMap2D& grid_map, std::size_t start_index)
{
  if (cost_grid_.size() != grid_map.getSize())
  {
    cost_grid_.resize(grid_map.getSize());
    return true;
  }

  if (cost_grid_[start_index])
  {
    return true;
  }

  return false;
}

void DijkstraExpander::addNode(const GridMap2D& grid_map, std::size_t prev_index, std::size_t next_index,
                               bool is_vertical)
{
  if (!grid_map.checkWithinBound(next_index) || cost_grid_[next_index] < std::numeric_limits<double>::max())
  {
    return;
  }

  if (grid_map.isObstacle(next_index) || (grid_map.isUnknown(next_index) && !traverse_unknown_))
  {
    return;
  }

  auto step_cost = (is_vertical ? step_cost_ : step_cost_diagonal_);
  auto new_cost = cost_grid_[prev_index] + step_cost + static_cast<int>(grid_map.isUnknown(next_index)) * unknown_cost_;

  cost_grid_[next_index] = new_cost;
  node_queue_.emplace(next_index, cost_grid_[next_index]);
  expanded_trace_[next_index] = prev_index;
}

}  // namespace virtual_scan
