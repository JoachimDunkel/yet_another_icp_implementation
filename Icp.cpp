#include "Icp.hpp"
#include "util.h"

namespace fast_icp
{


    ICP::ICP(const PointCloud &source_cloud, const PointCloud &target_cloud)
            : converged_threshold_(1e-8),
            max_iterations_(100),
            source_cloud_(source_cloud),
            target_cloud_(target_cloud),
            converged_(false){}

    Transform2D ICP::Align(PointCloud &transformed_cloud)
    {
        Transform2D full_transformation;
        full_transformation.setIdentity();

        transformed_cloud = source_cloud_.copy();

        double curr_error = std::numeric_limits<double>::max();
        double last_error = curr_error;

        for (size_t i = 0; i < max_iterations_; ++i)
        {
            DetermineCorrespondences();
            PointCloud corr_target_cloud = GetTargetCloudCorrespondences();
            Transform2D transformation = ComputeTransformationSVD(transformed_cloud, corr_target_cloud);
            transformed_cloud.transform(transformation);
            //Does this line work? -> Tests
            full_transformation = full_transformation * transformation;

            curr_error = ComputeError(transformed_cloud);

            if(curr_error > last_error){
                converged_ = false;
                break;
            }

            if(curr_error < converged_threshold_)
            {
                converged_ = true;
                break;
            }

            last_error = curr_error;
        }

        //transform transformed_cloud.

        return full_transformation;
    }

    void ICP::DetermineCorrespondences()
    {
        target_correspondences_.clear();

        //Todo replace with kd-tree implementation
        for (int i = 0; i < source_cloud_.getPoints().cols(); ++i) {
            Point source_point = source_cloud_.getPoints().col(i);
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

            target_correspondences_.push_back(closest_target_index);
        }
    }

    Transform2D ICP::ComputeTransformationSVD(const PointCloud &source_cloud, const PointCloud &target_cloud)
    {
        Point source_centroid = source_cloud.getPoints().rowwise().mean();
        Point target_centroid = target_cloud.getPoints().rowwise().mean();

        //compute transformation with svd
        PContainer centered_source_coordinates = source_cloud.getPoints().colwise() - source_centroid;
        PContainer centered_target_coordinates = target_cloud.getPoints().colwise() - target_centroid;

        auto W = centered_source_coordinates * centered_target_coordinates.transpose();

        Eigen::JacobiSVD<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>>
                    svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);

        //Should this be U*V^T ??
        Rotation2D rotation = svd.matrixV() * svd.matrixU().transpose();

        //why do i need this?
        if(rotation.determinant() < 0){
            rotation.col(1) *= -1.;
        }

        Translation2D translation = target_centroid - rotation * source_centroid;

        //stimmt das alles hier?
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

    double ICP::ComputeError(const PointCloud &transformed_cloud)
    {
        assert((size_t)transformed_cloud.getPoints().cols() == target_correspondences_.size());

        //root of squared_sum;
        PContainer differences = target_cloud_.getPoints() - transformed_cloud.getPoints();

        double error = differences.squaredNorm();
        //or
        //double error = differences.norm();
        return error;
    }

    bool ICP::isConverged() const
    {
        return converged_;
    }

    PointCloud ICP::GetTargetCloudCorrespondences()
    {
        PointCloud corresponding_target_cloud;
        assert(target_correspondences_.size() == (size_t)target_cloud_.getPoints().cols());

        for (int corr_index : target_correspondences_)
        {
            Point target_point = target_cloud_.getPoints().col(corr_index);
            corresponding_target_cloud.add(target_point);
        }
        return corresponding_target_cloud;
    }

}

