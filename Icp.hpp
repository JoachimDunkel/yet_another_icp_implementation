#pragma once

#include "PointCloud.hpp"

namespace fast_icp
{

class ICP
{
public:
    explicit ICP(const fast_icp::PointCloud & source_cloud, const fast_icp::PointCloud & target_cloud);

    Transform2D Align(fast_icp::PointCloud & transformed_cloud);

    double converged_threshold_;
    double max_iterations_;

    const fast_icp::PointCloud & source_cloud_;
    const fast_icp::PointCloud & target_cloud_;

private:
    //Determine correspondences..
    //compute transformation.
    std::vector<int> DetermineCorrespondences();

    Transform2D ComputeTransformation(const std::vector<int>& correspondences);

};

}
