#include "gtest/gtest.h"
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <vector>


#include "point_cloud_util.hpp"

class AffineTransformTests : public ::testing::Test {
protected:
    virtual void SetUp() {
        cloud_sut = fast_icp::PointCloud();
        pcl_cloud = PCLCloud();
        point_cloud_util::createPoints(cloud_sut, num_points);
        point_cloud_util::toPCL(cloud_sut.getPoints(), pcl_cloud);
    }

    void Exercise(){
        EXPECT_TRUE(point_cloud_util::areEqual(cloud_sut.getPoints(), pcl_cloud));
    }

    fast_icp::PointCloud cloud_sut;
    PCLCloud pcl_cloud;
    const size_t num_points = 5;

};

TEST_F(AffineTransformTests, setupWorks)
{
    Exercise();
}

TEST_F(AffineTransformTests, sameAsPCLTransformation)
{
    float theta = M_PI/4;
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << 2.5, 0.0, 0.0;
    transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    PCLCloud transformed_cloud;
    pcl::transformPointCloud(pcl_cloud, pcl_cloud, transform);
    cloud_sut.transform(2.5, 0, theta);

    Exercise();
}