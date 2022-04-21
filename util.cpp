#include <Eigen/Core>
#include <cmath>
#include "util.h"

namespace fast_icp
{

    float util::euclidDistance(const Point &a, const Point &b)
    {
        float x = b.x() - a.x();
        float y = b.y() - a.y();

        return std::sqrt(x*x + y*y);
    }

    Transform2D util::transformationFrom(const float &x, const float &y, const float &theta)
    {
        Transform2D trans = Transform2D::Identity();

        trans(0, 0) = std::cos(theta);
        trans(1, 0) = std::sin(theta);
        trans(0, 1) = -trans(1, 0);
        trans(1, 1) = trans(0, 0);
        trans(0, 2) = x;
        trans(1, 2) = y;
        return trans;
    }

}

