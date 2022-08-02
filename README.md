## Overview

This repository provides an implementation of the infamous `Iterative Closest Point` (Point-to-Point) variant.
It was created as part of a university course.

This library uses Eigen for Matrix representations and Singular Value Decomposition and nanoflann for it's kdTree implemenation.
For testing it depends on PCL, which is compared to for reference solutions.

In order to speed up ICP's convergence rate, differente correspondece strategies are possible.
1. Picking the closest correspondence.
2. Random sampling correspondences.


## TODO
Improve convergence speed performance by using eigen data types more efficently.
