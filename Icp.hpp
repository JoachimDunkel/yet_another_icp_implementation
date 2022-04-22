#pragma once

#include "resources.h"
#include "PointCloud.hpp"
#include "CorrespondencePicker.h"

namespace fast_icp
{

class ICP
{
public:
    explicit ICP(const fast_icp::PointCloud & source_cloud, const fast_icp::PointCloud & target_cloud);

    Transform2D Align(fast_icp::PointCloud & transformed_cloud);

    double converged_threshold_;
    size_t max_iterations_;
    SAMPLING_STRATEGY sampling_strategy_;
    CORRESPONDENCE_STRATEGY correspondence_strategy_;

    bool isConverged() const;
    size_t neededIterations() const;
    float getAlignmentError() const;

private:

    static Transform2D ComputeTransformationSVD(const PointCloud & source_cloud, const PointCloud & target_cloud);
    void ComputeError(const PointCloud &transformed_cloud);

    size_t needed_iterations_;
    const fast_icp::PointCloud & source_cloud_;
    const fast_icp::PointCloud & target_cloud_;

    bool converged_;
    float error_;
};

}
