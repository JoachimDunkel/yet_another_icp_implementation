#pragma once

#include "PointCloud.hpp"
#include "pcl/point_cloud.h"
#include "pcl/PointIndices.h"
#include "pcl/point_types_conversion.h"

#include <algorithm>
#include <cmath>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLCloud;

namespace fast_icp
{
class point_cloud_util
{
    typedef Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::RowMajor> PContainer3d;

public:
    static void createPoints(fast_icp::PointCloud &source_cloud, size_t num_points);

    static void toPCL(const PContainer &cloud_in, PCLCloud & pcl_cloud);
    static void fromPCL(const PCLCloud & pcl_cloud, PointCloud & cloud_out);

    static bool areEqual(const PContainer &a, const PContainer &b);
    static bool areEqual(const PCLPoint &a, const PCLPoint &b);
    static bool areEqual(const PContainer &a, const PCLCloud &b);

    static PContainer3d promoteTo3d(const PContainer &points_2d);
    static PContainer promoteTo2d(const PContainer3d &points_3d);

    static bool readCloudFromFile(const std::string& file_path, PCLCloud& cloud);
    static void storeCloudToFile(const std::string& file_path, const PCLCloud& cloud);

    static void printPCLTransformation(const Eigen::Matrix4f &matrix);

    static void viewSourceAndTargetCloudPCL(const PCLCloud::Ptr &source, const PCLCloud::Ptr &target);

    static void viewSourceAndTargetCloud(const PointCloud & source, const PointCloud & target);
};

}

