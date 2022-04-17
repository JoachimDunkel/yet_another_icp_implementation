#include <cmath>
#include <iostream>
#include "PointCloud.hpp"

PointCloud::PointCloud()
= default;

void PointCloud::add(const Point &point)
{
  points_.conservativeResize(Eigen::NoChange, points_.cols() + 1);
  points_.col(points_.cols() - 1) = point;
}

PointCloud PointCloud::copy() {
  PointCloud copy;
  copy.points_ = points_;

  float o_value = points_(0, 0);

  copy.points_(0, 0) = -1;

  assert(points_(0, 0) == o_value);
  return copy;
}

void PointCloud::transform(const float &x, const float &y, const float &theta)
{
  Eigen::Transform<float, 2, Eigen::Affine> transform = Eigen::Transform<float, 2, Eigen::Affine>::Identity();

  transform(0,0) = std::cos(theta);
  transform(1,0) = std::sin(theta);
  transform(0,1) = -transform(1,0);
  transform(1,1) = transform(0,0);
  transform(0,2) = x;
  transform(1,2) = y;

  points_ = transform * points_.colwise().homogeneous();
}

const PContainer &PointCloud::getPoints()
{
  return points_;
}
