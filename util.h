#pragma once

#include "resources.h"
#include <cmath>

namespace fast_icp
{
class util
{
public:
    static float euclidDistance(const fast_icp::Point &a, const fast_icp::Point &b);
};

}

