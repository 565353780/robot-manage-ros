#include <algorithm>
#include "distance_utils/euclidean_distance.h"

namespace virtual_scan
{
double EuclideanDistance::compute(const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point)
{
  return (start_point - end_point).norm();
}

std::vector<double> EuclideanDistance::compute(const Eigen::Vector2d& start_point,
                                               const std::vector<Eigen::Vector2d>& end_points)
{
  std::vector<double> result(end_points.size());
  std::transform(end_points.cbegin(), end_points.cend(), result.begin(),
                 [&, this](const Eigen::Vector2d& end_point) { return compute(start_point, end_point); });

  return result;
}
}  // namespace virtual_scan
