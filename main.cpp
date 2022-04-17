#include <iostream>
#include <random>
#include <algorithm>
#include <cmath>
#include "PointCloud.hpp"

const size_t num_points = 5;

void createPoints(PointCloud & source_cloud){

  double x = 0.0;
  double y = 0.0;

  double step = 1;

  for (size_t i = 0; i < num_points; ++i) {
    source_cloud.add(Point(x, y));
    x += step;
    y += step;
  }
}

using namespace Eigen;

int main(){

  PointCloud source_cloud;
  createPoints(source_cloud);
  auto target_cloud = source_cloud.copy();


  Eigen::Matrix<float, 3, Eigen::Dynamic , Eigen::RowMajor> mVertices;
  mVertices.resize(Eigen::NoChange, num_points);

  for (int i = 0; i < num_points; ++i)
  {
    auto p =  source_cloud.getPoints().col(i);
    Eigen::Vector3f p_3d;
    p_3d.x() = p.x();
    p_3d.y() = p.y();
    p_3d.z() = 0;

    mVertices.col(i) = p_3d;
  }

  std::cout << "=== 2d:  ===" << std::endl;
  source_cloud.transform(1,1, M_PI_4);
  std::cout << "=== 3d:  ===" << std::endl;

  Eigen::Transform<float, 3, Eigen::Affine> t = Transform < float, 3, Affine>::Identity();
  auto asdf = AngleAxisf(M_PI_4, Vector3f::UnitZ());
  t.translate(Eigen::Vector3f(1,1,0));

  t.rotate(asdf);


  std::cout << t* mVertices.colwise().homogeneous() << std::endl;

  return 0;
}