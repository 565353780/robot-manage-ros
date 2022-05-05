#include "distance_utils/distance_metric.h"

namespace virtual_scan
{
// 1 to 1 distances (use_cache can be used to reuse intermediate results)
double DistanceMetric::compute(const GridMap2D& grid_map, const Eigen::Vector2d& start_point,
                               const Eigen::Vector2d& end_point, bool use_cache)
{
  if (type_ == DistanceType::Euclidean)
  {
    return euclidean_distance_.compute(start_point, end_point);
  }
  else
  {
    return geodesic_distance_.compute(grid_map, start_point, end_point, use_cache);
  }
}

// 1 to N distances
std::vector<double> DistanceMetric::compute(const GridMap2D& grid_map, const Eigen::Vector2d& start_point,
                                            const std::vector<Eigen::Vector2d>& end_points, bool use_cache)
{
  if (type_ == DistanceType::Euclidean)
  {
    return euclidean_distance_.compute(start_point, end_points);
  }
  else
  {
    return geodesic_distance_.compute(grid_map, start_point, end_points, use_cache);
  }
}

// distance with path
std::optional<DistanceMetric::PathType> DistanceMetric::findPath(const GridMap2D& grid_map,
                                                                 const Eigen::Vector2d& start_point,
                                                                 const Eigen::Vector2d& end_point, bool use_cache)
{
  PathType result;
  try
  {
    geodesic_distance_.compute(grid_map, start_point, end_point);
  }
  catch (const char* msg)
  {
    std::cerr << "no safety path found !" << std::endl;
    return std::nullopt;
  }

  std::size_t current_index = grid_map.getIndex(end_point);
  std::size_t start_index = grid_map.getIndex(start_point);
  auto expanded_trace = geodesic_distance_.getExpandedTrace();
  if (expanded_trace.find(current_index) == expanded_trace.cend())
  {
    return std::nullopt;
  }

  auto iteration_threshold = grid_map.getSize() * 2;
  std::size_t iteration_counter = 0;
  while (current_index != start_index)
  {
    std::size_t prev_index = current_index;
    result.push_back(grid_map.getCoordinate(prev_index));
    current_index = expanded_trace.find(prev_index)->second;
    if (iteration_counter++ > iteration_threshold)
    {
      return std::nullopt;
    }
  }
  result.push_back(start_point);

  return result;
}

std::optional<std::vector<DistanceMetric::PathType>>
DistanceMetric::findPath(const GridMap2D& grid_map, const Eigen::Vector2d& start_point,
                         const std::vector<Eigen::Vector2d>& end_points, bool use_cache)
{
  std::vector<PathType> results;
  for (const auto& end_point : end_points)
  {
    auto path = findPath(grid_map, start_point, end_point, use_cache);
    if (!path)
    {
      return std::nullopt;
    }
    results.push_back(*path);
  }
  return results;
}
}  // namespace virtual_scan
