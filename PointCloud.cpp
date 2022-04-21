#include <cmath>
#include <iostream>
#include "PointCloud.hpp"
#include "util.h"


namespace fast_icp
{
    PointCloud::PointCloud()
    = default;

    void PointCloud::add(const Point &point)
    {
        points_.conservativeResize(Eigen::NoChange, points_.cols() + 1);
        points_.col(points_.cols() - 1) = point;
    }

    PointCloud PointCloud::copy() const
    {
        PointCloud copy;
        copy.points_ = points_;

        float o_value = points_(0, 0);

        copy.points_(0, 0) = -1;

        assert(points_(0, 0) == o_value);
        return copy;
    }

    void PointCloud::transform(const float &x, const float &y, const float &theta)
    {
        auto trans = fast_icp::util::transformationFrom(x, y, theta);
        transform(trans);
    }

    const PContainer &PointCloud::getPoints() const
    {
        return points_;
    }

    void PointCloud::transform(const Transform2D &transformation)
    {
        points_ = transformation * points_.colwise().homogeneous();
    }

    std::string PointCloud::getShape() const
    {
        std::ostringstream stream;
        stream  << "(" << points_.rows() << ", " << points_.cols() << ")";
        return stream.str();
    }

    bool PointCloud::operator==(const PointCloud &rhs) const
    {
        return getPoints().isApprox(rhs.getPoints());
    }

    bool PointCloud::operator!=(const PointCloud &rhs) const
    {
        return !(*this == rhs);
    }


}

