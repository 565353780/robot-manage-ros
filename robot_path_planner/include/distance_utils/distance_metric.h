#ifndef VIRTUAL_SCAN_DISTANCE_METRIC_H_
#define VIRTUAL_SCAN_DISTANCE_METRIC_H_

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <utility>
#include <optional>

#include "grid_map/grid_map_2d.h"
#include "distance_utils/euclidean_distance.h"
#include "distance_utils/geodesic_distance.h"

namespace virtual_scan
{
class DistanceMetric
{
public:
  enum class DistanceType
  {
    Euclidean,
    Geodesic
  };

  DistanceMetric(DistanceType type = DistanceType::Geodesic) : type_(type)
  {
  }

  // 1 to 1 distances (use_cache can be used to reuse intermediate results)
  double compute(const GridMap2D&, const Eigen::Vector2d&, const Eigen::Vector2d&, bool use_cache = false);
  // 1 to N distances
  // case 'use_cache = true' : end points shall be fixed
  std::vector<double> compute(const GridMap2D&, const Eigen::Vector2d&, const std::vector<Eigen::Vector2d>&,
                              bool use_cache = false);

  using PathType = std::vector<Eigen::Vector2d>;

  // distance with path
  std::optional<PathType> findPath(const GridMap2D&, const Eigen::Vector2d&, const Eigen::Vector2d&,
                                   bool use_cache = false);

  std::optional<std::vector<PathType>> findPath(const GridMap2D&, const Eigen::Vector2d&,
                                                const std::vector<Eigen::Vector2d>&, bool use_cache = false);

  DistanceType getDistanceType() const
  {
    return type_;
  }
  void setDistanceType(DistanceType type)
  {
    type_ = type;
  }

private:
  DistanceType type_;
  EuclideanDistance euclidean_distance_;
  GeodesicDistance geodesic_distance_;
};
}  // namespace virtual_scan

#endif  // CO_SCAN_DISTANCE_METRIC_H_
