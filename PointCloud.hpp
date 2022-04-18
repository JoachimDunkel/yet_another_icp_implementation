#pragma once
#include <Eigen/Dense>
#include <vector>
typedef Eigen::Vector2f Point;
typedef Eigen::Matrix<float, 2, Eigen::Dynamic , Eigen::RowMajor> PContainer;
typedef Eigen::Transform<float, 2, Eigen::Affine> Transform2D;

namespace fast_icp
{
    class PointCloud
    {
    public:
        explicit PointCloud();

        void add(const Point & point);

        PointCloud copy() const;

        void transform(const float &x, const float &y, const float &theta);

        const PContainer & getPoints() const;
    private:
        PContainer points_;


    };
}

