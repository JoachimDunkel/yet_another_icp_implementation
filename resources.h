#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>

namespace fast_icp
{
    typedef Eigen::Vector2f Point;
    typedef Eigen::Matrix<float, 2, Eigen::Dynamic , Eigen::RowMajor> PContainer;
    typedef Eigen::Transform<float, 2, Eigen::Affine> Transform2D;
    typedef Eigen::Matrix<float, 2, 2> Rotation2D;
    typedef Eigen::Matrix<float, 2, 1> Translation2D;

    namespace resources
    {

    }
}