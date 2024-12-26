// This file is derived from the Open3D Ball Pivoting Algorithm (BPA) module.
// Original source: https://github.com/intel-isl/Open3D
// License: MIT License
// Modifications for PCL compatibility by [Your Name], 2024

// IntersectionTest.cpp
#include "IntersectionTest.h"

// PointsCoplanar 実装
bool IntersectionTest::PointsCoplanar(const Eigen::Vector3d& p0,
                                       const Eigen::Vector3d& p1,
                                       const Eigen::Vector3d& p2,
                                       const Eigen::Vector3d& p3,
                                       double epsilon) {
    Eigen::Vector3d normal = (p1 - p0).cross(p2 - p0);
    double norm = normal.norm();
    if (norm < epsilon) {
        return false;
    }
    normal.normalize();

    double distance = normal.dot(p3 - p0);
    return std::abs(distance) < epsilon;
}

// LineSegmentsMinimumDistance 実装
double IntersectionTest::LineSegmentsMinimumDistance(const Eigen::Vector3d& p1,
                                                     const Eigen::Vector3d& p2,
                                                     const Eigen::Vector3d& p3,
                                                     const Eigen::Vector3d& p4) {
    Eigen::Vector3d u = p2 - p1;
    Eigen::Vector3d v = p4 - p3;
    Eigen::Vector3d w = p1 - p3;

    double a = u.dot(u);
    double b = u.dot(v);
    double c = v.dot(v);
    double d = u.dot(w);
    double e = v.dot(w);

    double D = a * c - b * b;
    double sc, tc;

    if (D < 1e-8) {
        sc = 0.0;
        tc = (b > c ? d / b : e / c);
    } else {
        sc = (b * e - c * d) / D;
        tc = (a * e - b * d) / D;
    }

    sc = std::max(0.0, std::min(1.0, sc));
    tc = std::max(0.0, std::min(1.0, tc));

    Eigen::Vector3d dP = w + (sc * u) - (tc * v);
    return dP.norm();
}
