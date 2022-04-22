#include "CorrespondencePicker.h"
#include "util.h"
#include <random>
#include "nanoflann.hpp"

namespace fast_icp
{

    CorrespondencePicker::CorrespondencePicker(const PointCloud &source_cloud, const PointCloud &target_cloud)
    : source_cloud_(source_cloud),
      target_cloud_(target_cloud),
      sample_ration_(0.2),
      sampling_strategy_(SAMPLING_STRATEGY::ALL_SOURCE_POINTS),
      correspondence_strategy_(CORRESPONDENCE_STRATEGY::PICK_CLOSEST_TARGET_POINT) {}

    std::vector<size_t> CorrespondencePicker::drawSample()
    {
        std::random_device random_device;
        std::mt19937 random_generator = std::mt19937(random_device());

        auto index_range = getSourceIds();
        long sample_size = std::lrint((float)source_cloud_.size() * sample_ration_);
        if(sample_size < 3){
            sample_size = 3;
        }
        std::shuffle(index_range.begin(), index_range.end(), random_generator);
        std::vector<size_t> source_ids = std::vector<size_t>(index_range.begin(), index_range.begin() + sample_size);
        return source_ids;
    }

    std::vector<size_t> CorrespondencePicker::getSourceIds()
    {
        std::vector<size_t> source_ids(source_cloud_.size());
        std::iota(source_ids.begin(), source_ids.end(), 0);
        return source_ids;
    }


    void CorrespondencePicker::GetAlignment(PointCloud & sampled_source, PointCloud & sampled_target)
    {
        std::vector<size_t> source_ids, target_ids;
        if(sampling_strategy_ == ALL_SOURCE_POINTS)
        {
            source_ids = getSourceIds();
        }
        else if(sampling_strategy_ == RANDOM_SUB_SAMPLE)
        {
            source_ids = drawSample();
        }
        sampled_source = source_cloud_.getSample(source_ids);

        if(correspondence_strategy_ == PICK_CLOSEST_TARGET_POINT)
        {
            using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<PContainer, 2, nanoflann::metric_L2, true>;
            KDTree kd_tree((int32_t)target_cloud_.size(), std::cref(target_cloud_.getPoints()), 10);
            kd_tree.index->buildIndex();

            // do a knn search
            const size_t K = 1;
            for (size_t i = 0; i < sampled_source.size(); ++i) {
                Point source_point = sampled_source.getPoints().col(i);
                float query_pt[2] = {source_point.x(),source_point.y()};
                size_t ret_index;
                float  out_dists_sqr;

                nanoflann::KNNResultSet<float> resultSet(K);
                resultSet.init(&ret_index, &out_dists_sqr);

                kd_tree.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams());

            }

//            //Todo replace with kd-tree implementation
//            for (int i = 0; i < sampled_source.getPoints().cols(); ++i) {
//                Point source_point = sampled_source.getPoints().col(i);
//                int closest_target_index = -1;
//                auto closest_distance = std::numeric_limits<float>::max();
//
//                for (int j = 0; j < target_cloud_.getPoints().cols(); ++j) {
//                    Point target_point = target_cloud_.getPoints().col(j);
//                    auto distance = util::euclidDistance(source_point, target_point);
//                    if(distance < closest_distance)
//                    {
//                        closest_distance = distance;
//                        closest_target_index = j;
//                    }
//                }
//
//                assert(closest_target_index != -1);
//                target_ids.push_back(closest_target_index);
//            }

        }
        assert(source_ids.size() == target_ids.size());
        sampled_target = target_cloud_.getSample(target_ids);
    }



}

