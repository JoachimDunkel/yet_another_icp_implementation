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

    size_t PointCloud::size() const
    {
        return getPoints().cols();
    }

    PointCloud PointCloud::getSample(const std::vector<size_t> & samples) const
    {
        PointCloud sampled_cloud;

        for (const size_t & sample_id : samples) {
            assert(sample_id < (size_t)points_.cols());
            Point source_point = points_.col((long)sample_id);
            sampled_cloud.add(source_point);
        }
        return sampled_cloud;
    }


}

