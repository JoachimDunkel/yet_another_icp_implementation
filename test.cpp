//#include <iostream>
//#include <random>
//#include <algorithm>
//#include <math.h>
//#include "PointCloud.hpp"
//
//typedef Eigen::Matrix<float, 3, Eigen::Dynamic , Eigen::RowMajor> PContainer3d;
//
//const size_t num_points = 5;
//
//void createPoints(PointCloud & source_cloud){
//
//  double x = 0.0;
//  double y = 0.0;
//
//  double step = 1;
//
//  for (size_t i = 0; i < num_points; ++i) {
//    source_cloud.add(Point(x, y));
//    x += step;
//    y += step;
//  }
//}
//
//
//bool areEqual(const PContainer & a, const PContainer & b)
//{
//  if(a.cols() != b.cols()) return false;
//
//  for (int i = 0; i < a.cols(); ++i)
//  {
//      if(a.col(i) != b.col(i)){
//        return false;
//      }
//  }
//
//  return true;
//}
//
//PContainer3d promoteTo3d(const PContainer & points_2d )
//{
//  PContainer3d points_3d;
//  points_3d.resize(Eigen::NoChange, points_2d.cols());
//
//  for (int i = 0; i < points_2d.cols(); ++i)
//  {
//    auto p = points_2d.col(i);
//    Eigen::Vector3f p_3d;
//    p_3d.x() = p.x();
//    p_3d.y() = p.y();
//    p_3d.z() = 0;
//
//    points_3d.col(i) = p_3d;
//  }
//
//  return points_3d;
//}
//
//PContainer promoteTo2d(const PContainer3d & points_3d)
//{
//  PContainer points_2d;
//  points_2d.resize(Eigen::NoChange, points_3d.cols());
//
//  for (int i = 0; i < points_3d.cols(); ++i) {
//    auto p = points_3d.col(i);
//    Eigen::Vector2f p_2d;
//    p_2d.x() = p.x();
//    p_2d.y() = p.y();
//
//    points_2d.col(i) = p_2d;
//  }
//
//  return points_2d;
//}
//
//using namespace Eigen;
//
//int main(){
//
//  PointCloud source_cloud;
//  createPoints(source_cloud);
//  auto target_cloud = source_cloud.copy();
//
//  auto mVertices = promoteTo3d(source_cloud.getPoints());
//
//  source_cloud.transform(1,1, M_PI_4);
//
//  Eigen::Transform<float, 3, Eigen::Affine> t = Transform < float, 3, Affine>::Identity();
//  auto asdf = AngleAxisf(M_PI_4, Vector3f::UnitZ());
//  t.translate(Eigen::Vector3f(1,1,0));
//  t.rotate(asdf);
//
//  mVertices = t * mVertices.colwise().homogeneous();
//
//  auto to2d = promoteTo2d(mVertices);
//
//  if(areEqual(to2d, source_cloud.getPoints())){
//    std::cout << "SUCCESS" << std::endl;
//  }
//  else{
//    std::cout << "FAILURE" << std::endl;
//  }
//
//  return 0;
//}