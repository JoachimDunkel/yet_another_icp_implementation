#pragma once

#include "../../PointCloud.hpp"
#include "pcl/point_cloud.h"
#include "pcl/PointIndices.h"
#include "pcl/point_types_conversion.h"

#include <algorithm>
#include <cmath>

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLCloud;

using namespace fast_icp;

namespace point_cloud_util {

    static void createPoints(fast_icp::PointCloud &source_cloud, size_t num_points){
        double x = 0.0;
        double y = 0.0;

        double step = 1;

        for (size_t i = 0; i < num_points; ++i) {
            source_cloud.add(Point(x, y));
            x += step;
            y += step;
        }
    }


    static void toPCL(const PContainer &cloud_in, PCLCloud & pcl_cloud){
        size_t num_points = cloud_in.cols();
        for (size_t i = 0; i < num_points; ++i)
        {
            auto point = cloud_in.col(i);
            PCLPoint pcl_p;
            pcl_p.x = point.x();
            pcl_p.y = point.y();
            pcl_p.z = 0;

            pcl_cloud.push_back(pcl_p);
        }
    }

    typedef Eigen::Matrix<float, 3, Eigen::Dynamic, Eigen::RowMajor> PContainer3d;

    static bool areEqual(const PContainer &a, const PContainer &b) {
        if (a.cols() != b.cols()) return false;

        for (int i = 0; i < a.cols(); ++i) {
            if (a.col(i) != b.col(i)) {
                return false;
            }
        }

        return true;
    }

    static bool areEqual(const PCLPoint &a, const PCLPoint &b)
    {
        double epsilon = 0.000000001;
        return(fabs(a.x) - fabs(b.x) < epsilon
               && fabs(a.y) - fabs(b.y) < epsilon
               && fabs(a.z) - fabs(b.z) < epsilon);
    }

    static bool areEqual(const PContainer &a, const PCLCloud &b)
    {
        if((size_t)a.cols() != b.points.size()) return false;

        PCLCloud a_pcl;
        toPCL(a, a_pcl);

        for (size_t i = 0; i < a_pcl.size(); ++i) {
            PCLPoint p_a = a_pcl.points.at(i);
            PCLPoint p_b = b.points.at(i);

            if(!areEqual(p_a, p_b)) return false;
        }

        return true;
    }


    static PContainer3d promoteTo3d(const PContainer &points_2d) {
        PContainer3d points_3d;
        points_3d.resize(Eigen::NoChange, points_2d.cols());

        for (int i = 0; i < points_2d.cols(); ++i) {
            auto p = points_2d.col(i);
            Eigen::Vector3f p_3d;
            p_3d.x() = p.x();
            p_3d.y() = p.y();
            p_3d.z() = 0;

            points_3d.col(i) = p_3d;
        }

        return points_3d;
    }

    static PContainer promoteTo2d(const PContainer3d &points_3d) {
        PContainer points_2d;
        points_2d.resize(Eigen::NoChange, points_3d.cols());

        for (int i = 0; i < points_3d.cols(); ++i) {
            auto p = points_3d.col(i);
            Eigen::Vector2f p_2d;
            p_2d.x() = p.x();
            p_2d.y() = p.y();

            points_2d.col(i) = p_2d;
        }

        return points_2d;
    }

}