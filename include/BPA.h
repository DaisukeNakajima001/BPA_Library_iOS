// This file is derived from the Open3D Ball Pivoting Algorithm (BPA) module.
// Original source: https://github.com/intel-isl/Open3D
// License: MIT License
// Modifications for PCL compatibility by [Your Name], 2024

#ifndef BPA_H
#define BPA_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/Dense>
#include <vector>
#include <list>
#include <memory>
#include <unordered_set>
#include "IntersectionTest.h" // 追加

class BallPivoting {
public:
    BallPivoting(const std::vector<Eigen::Vector3d>& points,
                const std::vector<Eigen::Vector3d>& normals);

    ~BallPivoting();

    pcl::PolygonMesh Run(const std::vector<double>& radii);

private:
    class Vertex;   // 前方宣言
    class Edge;     // 前方宣言
    class Triangle; // 前方宣言

    using VertexPtr = Vertex*;
    using EdgePtr = std::shared_ptr<Edge>;
    using TrianglePtr = std::shared_ptr<Triangle>;

    class Vertex {
    public:
        enum Type { Orphan = 0, Front = 1, Inner = 2 };

        Vertex(int idx, const Eigen::Vector3d& point, const Eigen::Vector3d& normal)
            : idx_(idx), point_(point), normal_(normal), type_(Orphan) {}

        void UpdateType();

        int idx_;
        Eigen::Vector3d point_;
        Eigen::Vector3d normal_;
        std::unordered_set<EdgePtr> edges_;
        Type type_;
    };

    class Edge {
    public:
        enum Type { Border = 0, Front = 1, Inner = 2 };

        Edge(VertexPtr source, VertexPtr target)
            : source_(source), target_(target), triangle0_(nullptr), triangle1_(nullptr), type_(Front) {}

        void AddAdjacentTriangle(TrianglePtr tri);
        VertexPtr GetOppositeVertex();

        VertexPtr source_;
        VertexPtr target_;
        TrianglePtr triangle0_;
        TrianglePtr triangle1_;
        Type type_;
    };

    class Triangle {
    public:
        Triangle(VertexPtr v0, VertexPtr v1, VertexPtr v2, const Eigen::Vector3d& ball_center)
            : vert0_(v0), vert1_(v1), vert2_(v2), ball_center_(ball_center) {}

        VertexPtr vert0_;
        VertexPtr vert1_;
        VertexPtr vert2_;
        Eigen::Vector3d ball_center_;
    };

    bool IsCompatible(VertexPtr v0, VertexPtr v1, VertexPtr v2);
    Eigen::Vector3d ComputeFaceNormal(const Eigen::Vector3d& v0,
                                      const Eigen::Vector3d& v1,
                                      const Eigen::Vector3d& v2);
    bool ComputeBallCenter(int vidx1, int vidx2, int vidx3, double radius, Eigen::Vector3d& center);
    VertexPtr FindCandidateVertex(const EdgePtr& edge, double radius, Eigen::Vector3d& candidate_center);
    void CreateTriangle(VertexPtr v0, VertexPtr v1, VertexPtr v2, const Eigen::Vector3d& center);
    EdgePtr GetLinkingEdge(VertexPtr v0, VertexPtr v1);
    bool TryTriangleSeed(VertexPtr v0, VertexPtr v1, VertexPtr v2, const std::vector<int>& nb_indices, double radius, Eigen::Vector3d& center);
    bool TrySeed(VertexPtr& v, double radius);
    void FindSeedTriangle(double radius);
    void ExpandTriangulation(double radius);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_;
    std::vector<VertexPtr> vertices_;
    std::list<EdgePtr> edge_front_;
    std::list<EdgePtr> border_edges_;
    std::vector<Eigen::Vector3i> triangles_;
};

pcl::PolygonMesh RunBPA(const std::vector<Eigen::Vector3d>& points,
                        const std::vector<Eigen::Vector3d>& normals,
                        const std::vector<double>& radii);

#endif  // BPA_H