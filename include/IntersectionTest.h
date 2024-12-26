// This file is derived from the Open3D Ball Pivoting Algorithm (BPA) module.
// Original source: https://github.com/intel-isl/Open3D
// License: MIT License
// Modifications for PCL compatibility by [Your Name], 2024

// include/IntersectionTest.h
#ifndef INTERSECTION_TEST_H
#define INTERSECTION_TEST_H

#include <Eigen/Dense>
#include <cmath>

class IntersectionTest {
public:
    // Coplanar判定: 四点が同一平面上にあるかどうかを判定
    static bool PointsCoplanar(const Eigen::Vector3d& p0,
                                const Eigen::Vector3d& p1,
                                const Eigen::Vector3d& p2,
                                const Eigen::Vector3d& p3,
                                double epsilon = 1e-3); // メッシュが異様に少ない時は大きくする

    // 線分間の最小距離を計算
    static double LineSegmentsMinimumDistance(const Eigen::Vector3d& p1,
                                              const Eigen::Vector3d& p2,
                                              const Eigen::Vector3d& p3,
                                              const Eigen::Vector3d& p4);
};

#endif // INTERSECTION_TEST_H
