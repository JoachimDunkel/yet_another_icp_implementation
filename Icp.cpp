
#include "Icp.hpp"
#include "util.h"

namespace fast_icp
{


    ICP::ICP(const PointCloud &source_cloud, const PointCloud &target_cloud)
            : converged_threshold_(1e-8), max_iterations_(100), source_cloud_(source_cloud), target_cloud_(target_cloud){}

    Transform2D ICP::Align(PointCloud &transformed_cloud)
    {
        Transform2D transformation;
        transformation.setIdentity();

        transformed_cloud = source_cloud_.copy();

        double curr_error = std::numeric_limits<double>::max();
        double last_error = curr_error;

        do
        {
            auto target_correspondences = DetermineCorrespondences();

            transformation = ComputeTransformation(target_correspondences);

            //compute current error.

        }
        while(curr_error < last_error && curr_error > converged_threshold_);

        //transform transformed_cloud.


        return transformation;
    }

    std::vector<int> ICP::DetermineCorrespondences()
    {
        std::vector<int> corresponding_target_points;

        //Todo replace with kd-tree implementation
        for (int i = 0; i < source_cloud_.getPoints().cols(); ++i) {
            Point source_point = source_cloud_.getPoints().col(i);
            int closest_target_index = -1;
            float closest_distance = std::numeric_limits<float>::max();

            for (int j = 0; j < target_cloud_.getPoints().cols(); ++j) {
                Point target_point = target_cloud_.getPoints().col(j);
                auto distance = util::euclidDistance(source_point, target_point);
                if(distance < closest_distance)
                {
                    closest_distance = distance;
                    closest_target_index = j;
                }
            }

            corresponding_target_points.push_back(closest_target_index);
        }

        return corresponding_target_points;
    }

    Transform2D ICP::ComputeTransformation(const std::vector<int>& correspondences)
    {
        Point source_centroid = source_cloud_.getPoints().colwise().mean();
        Point target_centroid = target_cloud_.getPoints().colwise().mean();

        //compute transformation with svd
        PContainer centered_source_coordinates = source_cloud_.getPoints().colwise() - source_centroid;
        PContainer centered_target_coordinates = target_cloud_.getPoints().colwise() - target_centroid;

        auto W = centered_source_coordinates * centered_target_coordinates.transpose();

        Eigen::JacobiSVD<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);

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

}

