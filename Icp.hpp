#pragma once

#include "resources.h"
#include "PointCloud.hpp"

namespace fast_icp
{

class ICP
{
public:
    explicit ICP(const fast_icp::PointCloud & source_cloud, const fast_icp::PointCloud & target_cloud);

    Transform2D Align(fast_icp::PointCloud & transformed_cloud);

    double converged_threshold_;
    size_t max_iterations_;

    bool isConverged() const;

private:

    void DetermineCorrespondences();
    static Transform2D ComputeTransformationSVD(const PointCloud & source_cloud, const PointCloud & target_cloud);
    double ComputeError(const PointCloud &transformed_cloud);
    PointCloud GetTargetCloudCorrespondences();

    const fast_icp::PointCloud & source_cloud_;
    const fast_icp::PointCloud & target_cloud_;

    std::vector<int> target_correspondences_;
    bool converged_;
};

}
