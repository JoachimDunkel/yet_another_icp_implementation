#pragma once
#include <Eigen/Dense>
#include <vector>
typedef Eigen::Vector2f Point;

typedef Eigen::Matrix<float, 2, Eigen::Dynamic , Eigen::RowMajor> PContainer;

class PointCloud
{
public:
    explicit PointCloud();

    void add(const Point & point);

    PointCloud copy();

    void transform(const float &x, const float &y, const float &theta);

    const PContainer & getPoints();
private:
    PContainer points_;
};
