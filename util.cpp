#include <Eigen/Core>
#include <cmath>
#include "util.h"

namespace fast_icp
{
    float util::euclidDistance(const fast_icp::Point &a, const fast_icp::Point &b)
    {
        float x = b.x() - a.x();
        float y = b.y() - a.y();

        return std::sqrt(x*x + y*y);
    }
}

