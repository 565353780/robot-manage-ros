#ifndef VIRTUAL_SCAN_EUCLIDEAN_DISTANCE_H_
#define VIRTUAL_SCAN_EUCLIDEAN_DISTANCE_H_

#include <vector>
#include <Eigen/Dense>

namespace virtual_scan
{
class EuclideanDistance
{
public:
  double compute(const Eigen::Vector2d& start_point, const Eigen::Vector2d& end_point);

  std::vector<double> compute(const Eigen::Vector2d& start_point, const std::vector<Eigen::Vector2d>& end_points);
};
}  // namespace virtual_scan

#endif  // VIRTUAL_SCAN_EUCLIDEAN_DISTANCE_H_
