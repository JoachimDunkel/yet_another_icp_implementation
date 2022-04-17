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

  source_cloud.transform(1,1,M_PI_4);

  return 0;
}