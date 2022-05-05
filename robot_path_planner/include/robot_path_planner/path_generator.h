#ifndef VIRTUAL_SCAN_PATH_GENERATOR_H_
#define VIRTUAL_SCAN_PATH_GENERATOR_H_

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "distance_utils/distance_metric.h"
#include "distance_utils/geodesic_distance.h"
#include "grid_map/grid_map_2d.h"
#include "nav_msgs/OccupancyGrid.h"
namespace virtual_scan
{
class PathGenerator
{
public:
  const std::optional<std::vector<Eigen::Vector2d>> generatePath(const GridMap2D& grid_map,
                                                                 const Eigen::Vector2d& start,
                                                                 const Eigen::Vector2d& end);

  const std::optional<std::vector<Eigen::Vector2d>> generatePath(const nav_msgs::OccupancyGrid& occupancy_grid,
                                                                 const Eigen::Vector2d& start,
                                                                 const Eigen::Vector2d& end);

  GridMap2D erodeMap(const GridMap2D& grid_map);

private:
  DistanceMetric distance_metric_{ DistanceMetric::DistanceType::Geodesic };

  const std::uint8_t safety_increment_ = 10;
  const unsigned iter_num_ = 10;
  const int kernel_size_ = 10;
  const cv::Mat erode_kernel_{ cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size_, kernel_size_)) };
  cv::Mat safety_grid_;
};
}  // namespace virtual_scan

#endif
