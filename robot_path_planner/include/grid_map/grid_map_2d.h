#ifndef VIRTUAL_SCAN_GRID_MAP_2D_H_
#define VIRTUAL_SCAN_GRID_MAP_2D_H_

#include <iostream>
#include <bits/stdint-intn.h>
#include <cstdint>
#include <vector>
#include <string>
#include <utility>
#include <optional>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace virtual_scan
{
class GridMap2D
{
public:
  // Representation for ROS OccupancyGrid topic
  // http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
  enum CellType : std::int8_t
  {
    UnknownCell = -1,
    FreeCell = 0,
    ObstacleCell = 100
  };

  GridMap2D() = default;

  explicit GridMap2D(const std::string& image_path);
  GridMap2D(const cv::Mat& map_image);
  GridMap2D(std::vector<CellType> occupancy_grid, std::size_t height, std::size_t width)
    : height_(height), width_(width), map_(std::move(occupancy_grid))
  {
  }

  GridMap2D(std::size_t height, std::size_t width, CellType cell_default = UnknownCell)
    : height_(height), width_(width), map_(height_ * width_, cell_default)
  {
  }

  GridMap2D& operator=(const cv::Mat& map_image);
  GridMap2D& operator=(const std::vector<std::int8_t>& occupancy_grid);

  CellType& operator()(std::size_t index);
  const CellType& operator()(std::size_t index) const;
  CellType& operator()(std::size_t row, std::size_t col);
  const CellType& operator()(std::size_t row, std::size_t col) const;
  CellType& operator()(const Eigen::Vector2d& query_point);
  const CellType& operator()(const Eigen::Vector2d& query_point) const;

  [[nodiscard]] bool isUnknown(std::size_t index) const;
  [[nodiscard]] bool isUnknown(std::size_t row, std::size_t col) const;
  [[nodiscard]] bool isUnknown(const Eigen::Vector2d& query_point) const;
  [[nodiscard]] bool isObstacle(std::size_t index) const;
  [[nodiscard]] bool isObstacle(std::size_t row, std::size_t col) const;
  [[nodiscard]] bool isObstacle(const Eigen::Vector2d& query_point) const;
  [[nodiscard]] bool isFree(std::size_t index) const;
  [[nodiscard]] bool isFree(std::size_t row, std::size_t col) const;
  [[nodiscard]] bool isFree(const Eigen::Vector2d& query_point) const;

  [[nodiscard]] std::size_t getHeight() const;
  [[nodiscard]] std::size_t getWidth() const;
  [[nodiscard]] std::size_t getSize() const;
  [[nodiscard]] std::size_t getIndex(const Eigen::Vector2d& query_point) const;
  [[nodiscard]] std::optional<std::size_t> getNeighborIndex(std::size_t index, int row_diff, int col_diff) const;
  [[nodiscard]] Eigen::Vector2d getCoordinate(std::size_t index) const;
  [[nodiscard]] static std::uint8_t getFreeColorThreshold();
  [[nodiscard]] static std::uint8_t getObstacleColorThreshold();

  [[nodiscard]] bool checkWithinBound(std::size_t index) const;
  [[nodiscard]] bool checkWithinBound(std::size_t row, std::size_t col) const;
  [[nodiscard]] bool checkWithinBound(const Eigen::Vector2d& query_point) const;

  void loadImage(const std::string& image_path);

  cv::Mat convertToMat() const;
  void convertFromMat(const cv::Mat& map_image);

private:
  std::size_t height_ = 0;
  std::size_t width_ = 0;
  std::vector<CellType> map_;

  static constexpr std::uint8_t free_color_ = 255;
  static constexpr std::uint8_t free_color_threshold_ = 240;
  static constexpr std::uint8_t obstacle_color_ = 0;
  static constexpr std::uint8_t obstacle_color_threshold_ = 15;
  static constexpr std::uint8_t unknown_color_ = 205;
};

inline GridMap2D::CellType& GridMap2D::operator()(std::size_t row, std::size_t col)
{
  return map_[row * width_ + col];
}

inline const GridMap2D::CellType& GridMap2D::operator()(std::size_t row, std::size_t col) const
{
  return map_[row * width_ + col];
}

inline GridMap2D::CellType& GridMap2D::operator()(std::size_t index)
{
  return map_[index];
}

inline const GridMap2D::CellType& GridMap2D::operator()(std::size_t index) const
{
  return map_[index];
}

inline bool GridMap2D::isUnknown(std::size_t index) const
{
  return operator()(index) == UnknownCell;
}

inline bool GridMap2D::isUnknown(std::size_t row, std::size_t col) const
{
  return operator()(row, col) == UnknownCell;
}

inline bool GridMap2D::isUnknown(const Eigen::Vector2d& query_point) const
{
  return operator()(query_point) == UnknownCell;
}

inline bool GridMap2D::isObstacle(std::size_t index) const
{
  return operator()(index) == ObstacleCell;
}

inline bool GridMap2D::isObstacle(std::size_t row, std::size_t col) const
{
  return operator()(row, col) == ObstacleCell;
}

inline bool GridMap2D::isObstacle(const Eigen::Vector2d& query_point) const
{
  return operator()(query_point) == ObstacleCell;
}

inline bool GridMap2D::isFree(std::size_t index) const
{
  return operator()(index) == FreeCell;
}

inline bool GridMap2D::isFree(std::size_t row, std::size_t col) const
{
  return operator()(row, col) == FreeCell;
}

inline bool GridMap2D::isFree(const Eigen::Vector2d& query_point) const
{
  return operator()(query_point) == FreeCell;
}

inline std::size_t GridMap2D::getHeight() const
{
  return height_;
}

inline std::size_t GridMap2D::getWidth() const
{
  return width_;
}

inline std::size_t GridMap2D::getSize() const
{
  return map_.size();
}

inline std::size_t GridMap2D::getIndex(const Eigen::Vector2d& query_point) const
{
  int col = static_cast<int>(std::nearbyint(query_point.x()));
  int row = static_cast<int>(std::nearbyint(query_point.y()));

  // // FIXME : somewhere will cause out of range!
  // col = std::max(col, 0);
  // col = std::min(col, int(width_ - 1));
  // row = std::max(row, 0);
  // row = std::min(row, int(height_ - 1));

  assert(checkWithinBound(row, col));
  return col + row * width_;
}

inline Eigen::Vector2d GridMap2D::getCoordinate(std::size_t index) const
{
  assert(checkWithinBound(index));
  return Eigen::Vector2d(index % width_, index / width_);
}

inline std::uint8_t GridMap2D::getFreeColorThreshold()
{
  return free_color_threshold_;
}

inline std::uint8_t GridMap2D::getObstacleColorThreshold()
{
  return obstacle_color_threshold_;
}

inline bool GridMap2D::checkWithinBound(std::size_t index) const
{
  return index < getSize();
}

inline bool GridMap2D::checkWithinBound(std::size_t row, std::size_t col) const
{
  return !(col >= width_ || row >= height_);
}

inline bool GridMap2D::checkWithinBound(const Eigen::Vector2d& query_point) const
{
  int col = static_cast<int>(std::nearbyint(query_point.x()));
  int row = static_cast<int>(std::nearbyint(query_point.y()));
  return !(col < 0 || row < 0 || col >= width_ || row >= height_);
}
}  // namespace virtual_scan

#endif  // CO_SCAN_GRID_MAP_2D_H_
