#include "CorrespondenceMetrics.h"
#include "util.h"
#include <random>

namespace fast_icp
{

    CorrespondenceMetrics::CorrespondenceMetrics(const PointCloud &source_cloud, const PointCloud &target_cloud)
    : source_cloud_(source_cloud),
      target_cloud_(target_cloud),
      sample_ration_(0.2),
      corr_rejection_(CORR_REJECTION::ALL_SOURCE_POINTS),
      corr_metric_(CORR_METRIC::CLOSEST_IN_TARGET) {}

    std::vector<size_t> CorrespondenceMetrics::drawSample()
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

    std::vector<size_t> CorrespondenceMetrics::getSourceIds()
    {
        std::vector<size_t> source_ids(source_cloud_.size());
        std::iota(source_ids.begin(), source_ids.end(), 0);
        return source_ids;
    }


    void CorrespondenceMetrics::GetAlignment(PointCloud & sampled_source, PointCloud & sampled_target)
    {
        std::vector<size_t> source_ids, target_ids;
        if(corr_rejection_ == ALL_SOURCE_POINTS)
        {
            source_ids = getSourceIds();
        }
        else if(corr_rejection_ == RANDOM_SUB_SAMPLE)
        {
            source_ids = drawSample();
        }
        sampled_source = source_cloud_.getSample(source_ids);

        if(corr_metric_ == CLOSEST_IN_TARGET)
        {
            //Todo replace with kd-tree implementation
            for (int i = 0; i < sampled_source.getPoints().cols(); ++i) {
                Point source_point = sampled_source.getPoints().col(i);
                int closest_target_index = -1;
                auto closest_distance = std::numeric_limits<float>::max();

                for (int j = 0; j < target_cloud_.getPoints().cols(); ++j) {
                    Point target_point = target_cloud_.getPoints().col(j);
                    auto distance = util::euclidDistance(source_point, target_point);
                    if(distance < closest_distance)
                    {
                        closest_distance = distance;
                        closest_target_index = j;
                    }
                }

                assert(closest_target_index != -1);
                target_ids.push_back(closest_target_index);
            }
        }
        assert(source_ids.size() == target_ids.size());
        sampled_target = target_cloud_.getSample(target_ids);
    }



}

