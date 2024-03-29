#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>

#include "PointCloud.hpp"
#include "point_cloud_util.hpp"
#include "util.h"
#include "Icp.hpp"

using namespace fast_icp;
//#include <pcl/visualization/cloud_viewer.h>

int main()
{
//    "/home/jd/git/fast_icp/data/edge.pcd"

    std::string file_path = "/home/jd/git/fast_icp/data/edge.pcd";
    PCLCloud cloud_pcl;
    if(!point_cloud_util::readCloudFromFile(file_path, cloud_pcl)) return -1;

    PointCloud source_cloud;
    point_cloud_util::fromPCL(cloud_pcl, source_cloud);

    PointCloud target_cloud = source_cloud.copy();
    auto trans = fast_icp::util::transformationFrom(0, 0, DEG2RAD(15));

    target_cloud.transform(trans);

    fast_icp::ICP icp(source_cloud, target_cloud);
    icp.max_iterations_ = 50;
    icp.sampling_strategy_ = SAMPLING_STRATEGY::RANDOM_SUB_SAMPLE;
    icp.correspondence_strategy_ = CORRESPONDENCE_STRATEGY::PICK_CLOSEST_TARGET_POINT;

    PointCloud transformed_cloud;
    auto found_transformation = icp.Align(transformed_cloud);

    if(trans.isApprox(found_transformation)){
        std::cout << "success" << std::endl;
    }
    else{
        std::cout << "failure" << std::endl;
    }

    return 0;
}