#pragma once
#include <vector>
#include "resources.h"

namespace fast_icp
{
    class PointCloud
    {
    public:
        explicit PointCloud();

        void add(const Point & point);

        PointCloud copy() const;

        void transform(const float &x, const float &y, const float &theta);
        void transform(const Transform2D & transformation);

        const PContainer & getPoints() const;
        size_t size() const;
        std::string getShape() const;

        bool operator == (const PointCloud &rhs) const;
        bool operator != (const PointCloud &rhs) const;

        PointCloud getSample(const std::vector<size_t> & samples) const;

    private:
        PContainer points_;


    };
}

