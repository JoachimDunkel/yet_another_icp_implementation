#pragma once

#include <memory>
#include <vector>
#include "PointCloud.hpp"
#include "nanoflann.hpp"

namespace fast_icp
{

enum SAMPLING_STRATEGY{
    ALL_SOURCE_POINTS = 0,
    RANDOM_SUB_SAMPLE = 1
};

enum CORRESPONDENCE_STRATEGY{
    PICK_CLOSEST_TARGET_POINT = 0
};

class CorrespondencePicker
{
    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<float, Eigen::Dynamic, 2>> KDTree;


public:
    explicit CorrespondencePicker(const PointCloud &source_cloud, const PointCloud &target_cloud);

    const PointCloud & source_cloud_;
    const PointCloud & target_cloud_;
    float sample_ration_;
    SAMPLING_STRATEGY sampling_strategy_;
    CORRESPONDENCE_STRATEGY correspondence_strategy_;

    void GetAlignment(PointCloud & sampled_source, PointCloud & sampled_target);

private:

    std::vector<size_t> drawSample();
    std::vector<size_t> getSourceIds();

    const Eigen::Matrix<float, Eigen::Dynamic, 2> kd_tree_points_;
    std::shared_ptr<KDTree> kd_tree_;

};


}


