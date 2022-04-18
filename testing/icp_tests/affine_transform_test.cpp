#include "gtest/gtest.h"
#include <pcl/point_cloud.h>
#include <iostream>

class AffineTransformTests : public ::testing::Test
{
protected:
    virtual void SetUp(){
    }
    virtual void TearDown(){
    }
};

TEST_F(AffineTransformTests, setupWorks)
{
  EXPECT_EQ(0,0);
}