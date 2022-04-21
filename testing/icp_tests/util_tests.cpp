#include "gtest/gtest.h"
#include <vector>
#include "../../util.h"
#include "../../PointCloud.hpp"

#include <pcl/common/distances.h>

TEST(util_tests, euclid_dist_works)
{
    fast_icp::Point a,b;
    a.x() = 3;
    a.y() = 7;
    b.x() = 5;
    b.y() = 5;

    auto res = fast_icp::util::euclidDistance(a, b);

    pcl::PointXY a_pcl = pcl::PointXY();
    pcl::PointXY b_pcl = pcl::PointXY();
    a_pcl.x = 3;
    a_pcl.y = 7;
    b_pcl.x = 5;
    b_pcl.y = 5;

    float expected = pcl::euclideanDistance(a_pcl, b_pcl);
    EXPECT_FLOAT_EQ(expected, res);
}

TEST(util_tests, row_wise_mean_is_what_i_expect)
{

    fast_icp::PointCloud cloud;

    for (int i = 0; i < 4; ++i) {
        fast_icp::Point a;
        a.x() = (float)i;
        a.y() = (float)i;
        cloud.add(a);
    }

    fast_icp::Point p = cloud.getPoints().rowwise().mean();

    EXPECT_FLOAT_EQ(1.5, p.x());
    EXPECT_FLOAT_EQ(1.5, p.y());
}


TEST(util_tests, minus_operator_does_row_wise_substraction_as_expected)
{
    fast_icp::PointCloud cloud_1, cloud_2;

    float factor = 0.1;

    fast_icp::Point a;
    a.x() = (float)1;
    a.y() = (float)1;
    cloud_1.add(a);

    a.x() -= factor;
    a.y() -= factor;
    cloud_2.add(a);

    fast_icp::PContainer result = cloud_1.getPoints() - cloud_2.getPoints();

    for (int i = 0; i < result.cols(); ++i)
    {
        fast_icp::Point p = result.col(i);
        EXPECT_FLOAT_EQ(p.x(), factor);
        EXPECT_FLOAT_EQ(p.x(), factor);
    }

}