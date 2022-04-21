#pragma once

#include "resources.h"
#include <cmath>

namespace fast_icp
{
class util
{
public:
    static float euclidDistance(const Point &a, const Point &b);
    static Transform2D transformationFrom(const float &x, const float &y, const float &theta);
};

}

