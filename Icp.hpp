#pragma once

#include "PointCloud.hpp"

namespace fast_icp
{

class ICP
{
public:
    explicit ICP();

    Transform2D Align(const fast_icp::PointCloud & source_cloud, const fast_icp::PointCloud & target_cloud, fast_icp::PointCloud & transformed_cloud);

    double converged_threshold_;
    double max_iterations_;

private:
    //Determine correspondences..
    //compute transformation.

};

}
