#include "gtest/gtest.h"
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <vector>


#include "../../point_cloud_util.hpp"
#include "../../util.h"
#include "../../Icp.hpp"

class ICPIntegrationTests : public ::testing::Test {
protected:
    virtual void SetUp()
    {
        source_cloud_ = PointCloud();
        target_cloud_ = PointCloud();
        source_pcl_.clear();
        file_path_ = "";
    }

    void LoadRectangle()
    {
        file_path_ = "/home/jd/git/fast_icp/data/rectangle.pcd";
        ReadFile();
    }

    void LoadEdge()
    {
        file_path_ = "/home/jd/git/fast_icp/data/edge.pcd";
        ReadFile();
    }

    void ReadFile()
    {
        auto could_read_file = point_cloud_util::readCloudFromFile(file_path_, source_pcl_);
        EXPECT_TRUE(could_read_file);
        point_cloud_util::fromPCL(source_pcl_, source_cloud_);
        target_cloud_ = source_cloud_.copy();
    }

    std::string file_path_;

    PointCloud target_cloud_;
    PointCloud source_cloud_;
    PCLCloud source_pcl_;

    Transform2D expected_transform_;

};

TEST_F(ICPIntegrationTests, RectangleWorksIfCorrespondencesCanBeFound)
{
    LoadRectangle();
    expected_transform_ = util::transformationFrom(0.2, 0.2, DEG2RAD(5));

    target_cloud_.transform(expected_transform_);

    ICP icp(source_cloud_, target_cloud_);

    PointCloud transformed_cloud;
    auto found_transformation = icp.Align(transformed_cloud);

    EXPECT_TRUE(source_cloud_ != target_cloud_);
    EXPECT_TRUE(transformed_cloud == target_cloud_);
    EXPECT_TRUE(expected_transform_.isApprox(found_transformation));
    EXPECT_TRUE(icp.neededIterations() == 1);
    EXPECT_TRUE(icp.isConverged());
}
