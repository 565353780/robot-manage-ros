#include "path_generator.h"
namespace virtual_scan
{
const std::optional<std::vector<Eigen::Vector2d>> PathGenerator::generatePath(const GridMap2D& grid_map,
                                                                              const Eigen::Vector2d& start,
                                                                              const Eigen::Vector2d& end)
{
  auto erode_map = erodeMap(grid_map);
  return distance_metric_.findPath(erode_map, start, end, false);
}

const std::optional<std::vector<Eigen::Vector2d>> PathGenerator::generatePath(
    const nav_msgs::OccupancyGrid& occupancy_grid,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end)
{
  GridMap2D grid_map(occupancy_grid.info.height, occupancy_grid.info.width);
  grid_map = occupancy_grid.data;

  virtual_scan::GridMap2D erode_map = erodeMap(grid_map);
  return distance_metric_.findPath(erode_map, start, end, false);
}

GridMap2D PathGenerator::erodeMap(const GridMap2D& grid_map)
{
  cv::Mat free_grid = grid_map.convertToMat();
  cv::threshold(free_grid, free_grid, grid_map.getFreeColorThreshold(), safety_increment_, cv::THRESH_BINARY);

  free_grid.copyTo(safety_grid_);
  for (int i = 0; i < iter_num_; ++i)
  {
    cv::erode(free_grid, free_grid, erode_kernel_);
    safety_grid_ += free_grid;
  }
  safety_grid_.setTo(std::numeric_limits<double>::lowest(), safety_grid_ == 0);
  GridMap2D erode_map(safety_grid_);
  return erode_map;
}

}  // namespace virtual_scan
