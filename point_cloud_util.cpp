#include "point_cloud_util.hpp"
#include <pcl/visualization/cloud_viewer.h>

namespace fast_icp {

    void point_cloud_util::createPoints(fast_icp::PointCloud &source_cloud, size_t num_points)
    {
        double x = 0.0;
        double y = 0.0;

        double step = 1;

        for (size_t i = 0; i < num_points; ++i) {
            source_cloud.add(Point(x, y));
            x += step;
            y += step;
        }
    }


    void point_cloud_util::toPCL(const PContainer &cloud_in, PCLCloud & pcl_cloud)
    {
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

    void point_cloud_util::fromPCL(const PCLCloud & pcl_cloud, PointCloud & cloud_out)
    {
        for (auto pcl_point : pcl_cloud.points) {
            Point point;
            point.x() = pcl_point.x;
            point.y() = pcl_point.y;
            cloud_out.add(point);
        }
    }

    bool point_cloud_util::areEqual(const PContainer &a, const PContainer &b) {
        if (a.cols() != b.cols()) return false;

        for (int i = 0; i < a.cols(); ++i) {
            if (a.col(i) != b.col(i)) {
                return false;
            }
        }

        return true;
    }

    bool point_cloud_util::areEqual(const PCLPoint &a, const PCLPoint &b)
    {
        double epsilon = 0.000000001;
        return(fabs(a.x) - fabs(b.x) < epsilon
               && fabs(a.y) - fabs(b.y) < epsilon
               && fabs(a.z) - fabs(b.z) < epsilon);
    }

    bool point_cloud_util::areEqual(const PContainer &a, const PCLCloud &b)
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


    point_cloud_util::PContainer3d point_cloud_util::promoteTo3d(const PContainer &points_2d) {
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

    PContainer point_cloud_util::promoteTo2d(const PContainer3d &points_3d) {
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

    bool point_cloud_util::readCloudFromFile(const std::string& file_path, PCLCloud& cloud)
    {
        if (pcl::io::loadPCDFile<PCLPoint> (file_path, cloud) == -1) //* load the file
        {
            std::cout << "Could not read point cloud from: " << file_path << std::endl;
            return false;
        }
        return true;
    }

    void point_cloud_util::storeCloudToFile(const std::string& file_path, const PCLCloud& cloud)
    {
        pcl::io::savePCDFileASCII (file_path, cloud);
    }

    void point_cloud_util::printPCLTransformation(const Eigen::Matrix4f & matrix)
    {
        printf ("Rotation matrix :\n");
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
        printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
        printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
        printf ("Translation vector :\n");
        printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    }

    void point_cloud_util::viewSourceAndTargetCloudPCL(const PCLCloud::Ptr &source, const PCLCloud::Ptr &target)
    {
        pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
        viewer.showCloud (source, "source");
        viewer.showCloud (target, "target");
        while (!viewer.wasStopped ())
        {
        }

    }

    void point_cloud_util::viewSourceAndTargetCloud(const PointCloud &source, const PointCloud &target)
    {
        PCLCloud::Ptr source_cloud_pcl(new PCLCloud);
        PCLCloud::Ptr target_cloud_pcl(new PCLCloud);

        toPCL(source.getPoints(), *source_cloud_pcl);
        toPCL(target.getPoints(), *target_cloud_pcl);
        viewSourceAndTargetCloudPCL(source_cloud_pcl, target_cloud_pcl);
    }

}