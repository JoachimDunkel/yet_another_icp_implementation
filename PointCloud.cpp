#include <cmath>
#include <iostream>
#include "PointCloud.hpp"


namespace fast_icp {
    PointCloud::PointCloud()
    = default;

    void PointCloud::add(const Point &point) {
        points_.conservativeResize(Eigen::NoChange, points_.cols() + 1);
        points_.col(points_.cols() - 1) = point;
    }

    PointCloud PointCloud::copy() const{
        PointCloud copy;
        copy.points_ = points_;

        float o_value = points_(0, 0);

        copy.points_(0, 0) = -1;

        assert(points_(0, 0) == o_value);
        return copy;
    }

    void PointCloud::transform(const float &x, const float &y, const float &theta)
    {
        Transform2D trans = Transform2D::Identity();

        trans(0, 0) = std::cos(theta);
        trans(1, 0) = std::sin(theta);
        trans(0, 1) = -trans(1, 0);
        trans(1, 1) = trans(0, 0);
        trans(0, 2) = x;
        trans(1, 2) = y;

        transform(trans);
    }

    const PContainer &PointCloud::getPoints() const {
        return points_;
    }

    void PointCloud::transform(const Transform2D &transformation)
    {
        points_ = transformation * points_.colwise().homogeneous();
    }

}

