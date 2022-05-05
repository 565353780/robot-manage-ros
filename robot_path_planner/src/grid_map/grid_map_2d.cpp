#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <utility>
#include <cstring>
#include "grid_map/grid_map_2d.h"

namespace virtual_scan 
{
GridMap2D::CellType& GridMap2D::operator()(const Eigen::Vector2d& query_point)
{
  return const_cast<CellType&>(const_cast<const GridMap2D&>(*this)(query_point));
}

const GridMap2D::CellType& GridMap2D::operator()(const Eigen::Vector2d& query_point) const
{
  auto row = static_cast<std::size_t>(std::nearbyint(query_point.y()));
  auto col = static_cast<std::size_t>(std::nearbyint(query_point.x()));
  return operator()(row, col);
}

GridMap2D::GridMap2D(const std::string& image_path)
{
  loadImage(image_path);
}

GridMap2D::GridMap2D(const cv::Mat& map_image)
{
  convertFromMat(map_image);
}

GridMap2D& GridMap2D::operator=(const cv::Mat& map_image)
{
  convertFromMat(map_image);
  return *this;
}

GridMap2D& GridMap2D::operator=(const std::vector<std::int8_t>& occupancy_grid)
{
  assert(occupancy_grid.size() == getSize());
  std::memcpy(map_.data(), occupancy_grid.data(), sizeof(std::int8_t) * occupancy_grid.size());
  return *this;
}

std::optional<std::size_t> GridMap2D::getNeighborIndex(std::size_t index, int row_diff, int col_diff) const
{
  assert(checkWithinBound(index));
  std::size_t new_row = static_cast<int>(index / width_) + row_diff;
  std::size_t new_col = static_cast<int>(index % width_) + col_diff;
  if (new_row >= height_ || new_col >= width_)
  {
    return std::nullopt;
  }
  return new_col + new_row * width_;
}

void GridMap2D::loadImage(const std::string& image_path)
{
  cv::Mat map_image = cv::imread(image_path);
  if (map_image.empty())
  {
    throw std::runtime_error("Cannot load image!");
  }

  convertFromMat(map_image);
}

void GridMap2D::convertFromMat(const cv::Mat& input_image)
{
  width_ = input_image.cols;
  height_ = input_image.rows;
  map_.resize(width_ * height_);

  cv::Mat map_image = input_image;
  if (input_image.channels() == 3)
  {
    cv::cvtColor(input_image, map_image, cv::COLOR_RGB2GRAY);
  }

  for (std::size_t i = 0; i < height_; ++i)
  {
    for (std::size_t j = 0; j < width_; ++j)
    {
      if (map_image.at<std::uint8_t>(i, j) > free_color_threshold_)
      {
        operator()(i, j) = FreeCell;
      }
      else if (map_image.at<std::uint8_t>(i, j) < obstacle_color_threshold_)
      {
        operator()(i, j) = ObstacleCell;
      }
      else
      {
        operator()(i, j) = UnknownCell;
      }
    }
  }
}

cv::Mat GridMap2D::convertToMat() const
{
  cv::Mat result(height_, width_, CV_8UC1);
  for (std::size_t i = 0; i < height_; ++i)
  {
    for (std::size_t j = 0; j < width_; ++j)
    {
      switch (operator()(i, j))
      {
        case FreeCell:
          result.at<std::uint8_t>(i, j) = free_color_;
          break;
        case ObstacleCell:
          result.at<std::uint8_t>(i, j) = obstacle_color_;
          break;
        case UnknownCell:
          result.at<std::uint8_t>(i, j) = unknown_color_;
      }
    }
  }
  return result;
}
}  // namespace co_scan
