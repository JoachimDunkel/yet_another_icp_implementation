#include "../../point_cloud_util.hpp"
#include "../../util.h"
#include "gtest/gtest.h"
#include "../../nanoflann.hpp"
#include <pcl/point_cloud.h>

using namespace fast_icp;

class CorrespondenceTests : public ::testing::Test{
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

    void compareBruteForceAndKnnSearch(){
        auto expected_transform_ = util::transformationFrom(0.1, 0.1, DEG2RAD(2));
        target_cloud_.transform(expected_transform_);

        std::vector<size_t> target_ids_naive_, target_ids_kd_tree_;

        for (int i = 0; i < source_cloud_.getPoints().cols(); ++i) {
            Point source_point = source_cloud_.getPoints().col(i);
            int closest_target_index = -1;
            auto closest_distance = std::numeric_limits<float>::max();

            for (int j = 0; j < target_cloud_.getPoints().cols(); ++j) {
                Point target_point = target_cloud_.getPoints().col(j);
                auto distance = util::euclidDistance(source_point, target_point);
                if(distance < closest_distance)
                {
                    closest_distance = distance;
                    closest_target_index = j;
                }
            }

            assert(closest_target_index != -1);
            target_ids_naive_.push_back(closest_target_index);
        }

        typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::Matrix<float, Eigen::Dynamic, 2>> KDTree;
        std::shared_ptr<KDTree> kd_tree;

        const Eigen::Matrix<float, Eigen::Dynamic, 2> kd_tree_mat = target_cloud_.getPoints().transpose();
        kd_tree.reset(new KDTree(2, kd_tree_mat));
        kd_tree->index->buildIndex();

        // do a knn search
        const size_t K = 1;
        std::vector<typename KDTree::IndexType> closest_target_id(K);
        std::vector<float> closest_target_dist(K);
        for (size_t i = 0; i < source_cloud_.size(); ++i) {
            Point source_point = source_cloud_.getPoints().col((long)i);
            float query_pt[2] = {source_point.x(),source_point.y()};

            kd_tree->index->knnSearch(&query_pt[0], K, &closest_target_id[0], &closest_target_dist[0]);
            target_ids_kd_tree_.push_back(closest_target_id[0]);
        }

        EXPECT_EQ(target_ids_naive_.size(), target_ids_kd_tree_.size());
        for (size_t i = 0; i < target_ids_naive_.size(); ++i)
        {
            size_t naive_id = target_ids_naive_.at(i);
            size_t kd_tree_id = target_ids_kd_tree_.at(i);
            EXPECT_EQ(naive_id, kd_tree_id);
        }
    }

};

TEST_F(CorrespondenceTests, nanoFlannKnnSearchWorksOnRectangle)
{
    LoadRectangle();
    compareBruteForceAndKnnSearch();
}

TEST_F(CorrespondenceTests, nanoFlannKnnSearchWorksOnEdge)
{
    LoadEdge();
    compareBruteForceAndKnnSearch();
}