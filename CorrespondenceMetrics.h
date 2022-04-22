#pragma once

#include <vector>
#include "PointCloud.hpp"

namespace fast_icp
{

enum CORR_REJECTION{
    ALL_SOURCE_POINTS = 0,
    RANDOM_SUB_SAMPLE = 1
};

enum CORR_METRIC{
    CLOSEST_IN_TARGET = 0
};

class CorrespondenceMetrics
{
public:
    explicit CorrespondenceMetrics(const PointCloud &source_cloud, const PointCloud &target_cloud);

    const PointCloud & source_cloud_;
    const PointCloud & target_cloud_;
    float sample_ration_;
    CORR_REJECTION corr_rejection_;
    CORR_METRIC corr_metric_;

    void GetAlignment(PointCloud & sampled_source, PointCloud & sampled_target);

private:

    std::vector<size_t> drawSample();
    std::vector<size_t> getSourceIds();





};







}


