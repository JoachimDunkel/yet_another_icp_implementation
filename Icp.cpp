
#include "Icp.hpp"

namespace fast_icp
{
    ICP::ICP() : converged_threshold_(1e-8), max_iterations_(100)
    {}

    Transform2D ICP::Align(const PointCloud &source_cloud, const PointCloud &target_cloud, PointCloud &transformed_cloud)
    {
        Transform2D transformation;
        transformation.setIdentity();

        transformed_cloud = source_cloud.copy();

        double curr_error = std::numeric_limits<double>::max();
        double last_error = curr_error;

        do
        {
            //Determine correspondences
            //compute transformation with svd
            //set curr_error term.
        }
        while(curr_error < last_error && curr_error > converged_threshold_);


        return transformation;
    }

}

