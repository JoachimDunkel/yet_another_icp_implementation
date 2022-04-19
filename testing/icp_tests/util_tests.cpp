#include "gtest/gtest.h"
#include <vector>
#include "../../util.h"

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
