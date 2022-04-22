#include "Icp.hpp"
#include "util.h"

namespace fast_icp
{

    ICP::ICP(const PointCloud &source_cloud, const PointCloud &target_cloud)
            : converged_threshold_(2e-2),
              max_iterations_(50),
              corr_rejection_(CORR_REJECTION::ALL_SOURCE_POINTS),
              corr_metric_(CORR_METRIC::CLOSEST_IN_TARGET),
              needed_iterations_(max_iterations_),
              source_cloud_(source_cloud),
              target_cloud_(target_cloud),
              converged_(false),
              error_(std::numeric_limits<float>::max())
                {}

    Transform2D ICP::Align(PointCloud &transformed_cloud)
    {
        Transform2D full_transformation;
        full_transformation.setIdentity();

        transformed_cloud = source_cloud_.copy();

        needed_iterations_ = 0;

        CorrespondenceMetrics corr_metrics_(transformed_cloud, target_cloud_);
        corr_metrics_.corr_rejection_ = corr_rejection_;
        corr_metrics_.corr_metric_ = corr_metric_;

        for (size_t i = 0; i < max_iterations_; ++i)
        {
            needed_iterations_ ++;

            PointCloud sampled_source, sampled_target;
            corr_metrics_.GetAlignment(sampled_source, sampled_target);

            Transform2D transformation = ComputeTransformationSVD(sampled_source, sampled_target);
            transformed_cloud.transform(transformation);

            full_transformation = full_transformation * transformation;

            ComputeError(transformed_cloud);

            if(error_ < converged_threshold_)
            {
                converged_ = true;
                break;
            }
        }

        return full_transformation;
    }


    Transform2D ICP::ComputeTransformationSVD(const PointCloud &source_cloud, const PointCloud &target_cloud)
    {
        Point source_centroid = source_cloud.getPoints().rowwise().mean();
        Point target_centroid = target_cloud.getPoints().rowwise().mean();

        PContainer centered_source_coordinates = source_cloud.getPoints().colwise() - source_centroid;
        PContainer centered_target_coordinates = target_cloud.getPoints().colwise() - target_centroid;

        auto W = centered_source_coordinates * centered_target_coordinates.transpose();

        Eigen::JacobiSVD<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>
                    svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Rotation2D rotation = svd.matrixV() * svd.matrixU().transpose();

        Translation2D translation = target_centroid - rotation * source_centroid;

        Transform2D transformation;
        transformation.setIdentity();
        transformation(0,0) = rotation(0,0);
        transformation(0,1) = rotation(0,1);
        transformation(1,0) = rotation(1,0);
        transformation(1,1) = rotation(1,1);

        transformation(0,2) = translation(0,0);
        transformation(1,2) = translation(1,0);

        return transformation;
    }

    void ICP::ComputeError(const PointCloud &transformed_cloud)
    {
        PContainer differences = target_cloud_.getPoints() - transformed_cloud.getPoints();
        error_ = differences.squaredNorm();
    }

    bool ICP::isConverged() const
    {
        return converged_;
    }

    size_t ICP::neededIterations() const
    {
        return needed_iterations_;
    }

    float ICP::getAlignmentError() const
    {
        return error_;
    }

}

