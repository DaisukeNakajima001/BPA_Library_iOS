// This file is derived from the Open3D Ball Pivoting Algorithm (BPA) module.
// Original source: https://github.com/intel-isl/Open3D
// License: MIT License
// Modifications for PCL compatibility by [Your Name], 2024

#include "BPA.h"
#include <pcl/conversions.h>
#include <cmath>
#include <cassert>
#include <iostream>

// BallPivoting::Vertex Implementation
void BallPivoting::Vertex::UpdateType() {
    if (edges_.empty()) {
        type_ = Orphan;
    } else {
        for (const auto& edge : edges_) {
            if (edge->type_ != Edge::Type::Inner) {
                type_ = Front;
                return;
            }
        }
        type_ = Inner;
    }
}

// BallPivoting::Edge Implementation
void BallPivoting::Edge::AddAdjacentTriangle(TrianglePtr tri) {
    if (tri != triangle0_ && tri != triangle1_) {
        if (triangle0_ == nullptr) {
            triangle0_ = tri;
            type_ = Front;
            // Orientation の更新
            VertexPtr opp = GetOppositeVertex();
            if (opp) {
                Eigen::Vector3d tr_norm = (target_->point_ - source_->point_).cross(opp->point_ - source_->point_);
                double norm = tr_norm.norm();
                if (norm > 0) {
                    tr_norm /= norm;
                } else {
                    tr_norm = Eigen::Vector3d::Zero();
                }

                Eigen::Vector3d pt_norm = (source_->normal_ + target_->normal_ + opp->normal_).normalized();
                if (tr_norm.dot(pt_norm) < 0) {
                    std::swap(target_, source_);
                }
            } else {
                pcl::console::print_error("[Edge::AddAdjacentTriangle] GetOppositeVertex() returned nullptr.\n");
            }
        } else if (triangle1_ == nullptr) {
            triangle1_ = tri;
            type_ = Inner;
        } else {
            pcl::console::print_warn("[Edge::AddAdjacentTriangle] Warning: More than two adjacent triangles.\n");
        }
    }
}

BallPivoting::VertexPtr BallPivoting::Edge::GetOppositeVertex() {
    if (triangle0_ != nullptr) {
        if (triangle0_->vert0_ != source_ && triangle0_->vert0_ != target_)
            return triangle0_->vert0_;
        if (triangle0_->vert1_ != source_ && triangle0_->vert1_ != target_)
            return triangle0_->vert1_;
        return triangle0_->vert2_;
    }
    return nullptr;
}

// BallPivoting Implementation
BallPivoting::BallPivoting(const std::vector<Eigen::Vector3d>& points,
                           const std::vector<Eigen::Vector3d>& normals) {
    assert(points.size() == normals.size());
    pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_cloud_->resize(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        (*pcl_cloud_)[i].x = static_cast<float>(points[i].x());
        (*pcl_cloud_)[i].y = static_cast<float>(points[i].y());
        (*pcl_cloud_)[i].z = static_cast<float>(points[i].z());
    }
    kdtree_.setInputCloud(pcl_cloud_);

    vertices_.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        vertices_.emplace_back(new Vertex(static_cast<int>(i), points[i], normals[i]));
    }
}

BallPivoting::~BallPivoting() {
    for (auto v : vertices_) {
        delete v;
    }
}

bool BallPivoting::IsCompatible(VertexPtr v0, VertexPtr v1, VertexPtr v2) {
    // 三頂点から計算した三角形法線
    Eigen::Vector3d normal = ComputeFaceNormal(v0->point_, v1->point_, v2->point_);

    // もし三角形法線が頂点v0の法線と逆向きなら反転
    if (normal.dot(v0->normal_) < -1e-16) {
        normal *= -1;
    }

    // 各頂点の法線と同じ向きかどうかを判定
    bool ret = (normal.dot(v0->normal_) > -1e-16) &&
               (normal.dot(v1->normal_) > -1e-16) &&
               (normal.dot(v2->normal_) > -1e-16);
    return ret;
}


Eigen::Vector3d BallPivoting::ComputeFaceNormal(const Eigen::Vector3d& v0,
                                               const Eigen::Vector3d& v1,
                                               const Eigen::Vector3d& v2) {
    Eigen::Vector3d normal = (v1 - v0).cross(v2 - v0);
    double norm = normal.norm();
    if (norm > 0) {
        normal /= norm;
    }
    return normal;
}

bool BallPivoting::ComputeBallCenter(int vidx1, int vidx2, int vidx3, double radius, Eigen::Vector3d& center) {
    const Eigen::Vector3d& v1 = vertices_[vidx1]->point_;
    const Eigen::Vector3d& v2 = vertices_[vidx2]->point_;
    const Eigen::Vector3d& v3 = vertices_[vidx3]->point_;

    double a = (v3 - v2).squaredNorm();
    double b = (v1 - v3).squaredNorm();
    double c = (v2 - v1).squaredNorm();

    double alpha = a * (b + c - a);
    double beta = b * (a + c - b);
    double gamma = c * (a + b - c);
    double sum = alpha + beta + gamma;

    if (sum < 1e-16) {
        return false;
    }

    alpha /= sum;
    beta /= sum;
    gamma /= sum;

    Eigen::Vector3d circ_center = alpha * v1 + beta * v2 + gamma * v3;
    double circ_radius2 = a * b * c / ((std::sqrt(a) + std::sqrt(b) + std::sqrt(c)) *
                                       (std::sqrt(b) + std::sqrt(c) - std::sqrt(a)) *
                                       (std::sqrt(c) + std::sqrt(a) - std::sqrt(b)) *
                                       (std::sqrt(a) + std::sqrt(b) - std::sqrt(c)));

    double height_sq = radius * radius - circ_radius2;
    if (height_sq >= 0.0) {
        Eigen::Vector3d tr_norm = (v2 - v1).cross(v3 - v1);
        double norm = tr_norm.norm();
        if (norm > 0) {
            tr_norm /= norm;
        } else {
            return false;
        }

        Eigen::Vector3d pt_norm = (vertices_[vidx1]->normal_ + vertices_[vidx2]->normal_ + vertices_[vidx3]->normal_).normalized();
        if (tr_norm.dot(pt_norm) < 0) {
            tr_norm *= -1;
        }

        double height = std::sqrt(height_sq);
        center = circ_center + height * tr_norm;
        return true;
    }
    return false;
}

BallPivoting::EdgePtr BallPivoting::GetLinkingEdge(VertexPtr v0, VertexPtr v1) {
    for (const auto& edge : v0->edges_) {
        if ((edge->source_ == v0 && edge->target_ == v1) ||
            (edge->source_ == v1 && edge->target_ == v0)) {
            return edge;
        }
    }
    return nullptr;
}

void BallPivoting::CreateTriangle(VertexPtr v0, VertexPtr v1, VertexPtr v2, const Eigen::Vector3d& center) {
    TrianglePtr triangle = std::make_shared<Triangle>(v0, v1, v2, center);
    
    // エッジのリンクを取得または新規作成
    EdgePtr e0 = GetLinkingEdge(v0, v1);
    if (e0 == nullptr) {
        e0 = std::make_shared<Edge>(v0, v1);
    }
    e0->AddAdjacentTriangle(triangle);
    v0->edges_.insert(e0);
    v1->edges_.insert(e0);

    EdgePtr e1 = GetLinkingEdge(v1, v2);
    if (e1 == nullptr) {
        e1 = std::make_shared<Edge>(v1, v2);
    }
    e1->AddAdjacentTriangle(triangle);
    v1->edges_.insert(e1);
    v2->edges_.insert(e1);

    EdgePtr e2 = GetLinkingEdge(v2, v0);
    if (e2 == nullptr) {
        e2 = std::make_shared<Edge>(v2, v0);
    }
    e2->AddAdjacentTriangle(triangle);
    v2->edges_.insert(e2);
    v0->edges_.insert(e2);

    // 頂点のタイプを更新
    v0->UpdateType();
    v1->UpdateType();
    v2->UpdateType();

    // 三角形の法線を計算し、頂点法線と整合性を確認
    Eigen::Vector3d face_normal = ComputeFaceNormal(v0->point_, v1->point_, v2->point_);
    Eigen::Vector3d average_normal = (v0->normal_ + v1->normal_ + v2->normal_) / 3.0;
    average_normal.normalize();

    if (face_normal.dot(average_normal) > -1e-16) {
        triangles_.emplace_back(Eigen::Vector3i(v0->idx_, v1->idx_, v2->idx_));
    } else {
        triangles_.emplace_back(Eigen::Vector3i(v0->idx_, v2->idx_, v1->idx_));
    }

    // フロンティアエッジを更新
    if (e0->type_ == Edge::Front) {
        edge_front_.push_front(e0);
    }
    if (e1->type_ == Edge::Front) {
        edge_front_.push_front(e1);
    }
    if (e2->type_ == Edge::Front) {
        edge_front_.push_front(e2);
    }
}

BallPivoting::VertexPtr BallPivoting::FindCandidateVertex(const EdgePtr& edge, double radius, Eigen::Vector3d& candidate_center) {
    VertexPtr src = edge->source_;
    VertexPtr tgt = edge->target_;

    VertexPtr opp = edge->GetOppositeVertex();
    if (opp == nullptr) {
        return nullptr;
    }

    Eigen::Vector3d mid_point = 0.5 * (src->point_ + tgt->point_);

    if (edge->triangle0_ == nullptr) {
        return nullptr;
    }
    Eigen::Vector3d center = edge->triangle0_->ball_center_;

    Eigen::Vector3d v = tgt->point_ - src->point_;
    v.normalize();

    Eigen::Vector3d a = (center - mid_point).normalized();

    std::vector<int> indices;
    std::vector<float> dists2;
    kdtree_.radiusSearch(pcl::PointXYZ(mid_point.x(), mid_point.y(), mid_point.z()), 2 * radius, indices, dists2);

    VertexPtr min_candidate = nullptr;
    double min_angle = 2 * M_PI;

    for (auto nbidx : indices) {
        VertexPtr candidate = vertices_[nbidx];
        if (candidate->idx_ == src->idx_ || candidate->idx_ == tgt->idx_ || candidate->idx_ == opp->idx_) {
            continue;
        }

        // Coplanar 判定
        bool coplanar = IntersectionTest::PointsCoplanar(
            src->point_, tgt->point_, opp->point_, candidate->point_
        );

        if (coplanar) {
            // LineSegmentsMinimumDistance を用いて干渉を確認
            double dist1 = IntersectionTest::LineSegmentsMinimumDistance(
                mid_point, candidate->point_, src->point_, opp->point_
            );
            double dist2 = IntersectionTest::LineSegmentsMinimumDistance(
                mid_point, candidate->point_, tgt->point_, opp->point_
            );

            if (dist1 < 1e-9 || dist2 < 1e-9) {  // 干渉判定でメッシュが異様に少ない時は大きくする
                // 干渉があるため、候補から除外
                continue;
            }
        }

        // ボールの中心を計算
        Eigen::Vector3d new_center;
        if (!ComputeBallCenter(src->idx_, tgt->idx_, candidate->idx_, radius, new_center)) {
            continue;
        }

        // 法線の整合性を確認
        Eigen::Vector3d b = (new_center - mid_point).normalized();
        double cos_angle = a.dot(b);
        cos_angle = std::min(cos_angle, 1.0);
        cos_angle = std::max(cos_angle, -1.0);
        double angle = std::acos(cos_angle);

        Eigen::Vector3d c = a.cross(b);
        if (c.dot(v) < 0) {
            angle = 2 * M_PI - angle;
        }

        if (angle >= min_angle) {
            continue;
        }

        // ボールが空かどうかチェック
        bool empty_ball = true;
        for (auto nbidx2 : indices) {
            VertexPtr nb = vertices_[nbidx2];
            if (nb->idx_ == src->idx_ || nb->idx_ == tgt->idx_ || nb->idx_ == candidate->idx_) {
                continue;
            }
            if ((new_center - nb->point_).norm() < radius - 1e-16) {
                empty_ball = false;
                break;
            }
        }

        if (empty_ball) {
            min_angle = angle;
            min_candidate = candidate;
            candidate_center = new_center;
        }
    }

    return min_candidate;
}

bool BallPivoting::TryTriangleSeed(VertexPtr v0, VertexPtr v1, VertexPtr v2, const std::vector<int>& nb_indices, double radius, Eigen::Vector3d& center) {
    if (!IsCompatible(v0, v1, v2)) {
        return false;
    }

    EdgePtr e0 = GetLinkingEdge(v0, v2);
    EdgePtr e1 = GetLinkingEdge(v1, v2);
    if ((e0 != nullptr && e0->type_ == Edge::Inner) ||
        (e1 != nullptr && e1->type_ == Edge::Inner)) {
        return false;
    }

    if (!ComputeBallCenter(v0->idx_, v1->idx_, v2->idx_, radius, center)) {
        return false;
    }

    // ボールが空かどうかチェック
    for (const auto& nbidx : nb_indices) {
        VertexPtr v = vertices_[nbidx];
        if (v->idx_ == v0->idx_ || v->idx_ == v1->idx_ || v->idx_ == v2->idx_) {
            continue;
        }
        if ((center - v->point_).norm() < radius - 1e-16) {
            return false;
        }
    }

    return true;
}

bool BallPivoting::TrySeed(VertexPtr& v, double radius) {
    std::vector<int> indices;
    std::vector<float> dists2;
    kdtree_.radiusSearch(pcl::PointXYZ(v->point_.x(), v->point_.y(), v->point_.z()), 2 * radius, indices, dists2);
    if (indices.size() < 3) {
        return false;
    }

    for (size_t i = 0; i < indices.size(); ++i) {
        VertexPtr nb0 = vertices_[indices[i]];
        if (nb0->type_ != Vertex::Orphan || nb0->idx_ == v->idx_) {
            continue;
        }

        for (size_t j = i + 1; j < indices.size(); ++j) {
            VertexPtr nb1 = vertices_[indices[j]];
            if (nb1->type_ != Vertex::Orphan || nb1->idx_ == v->idx_) {
                continue;
            }

            Eigen::Vector3d center;
            if (TryTriangleSeed(v, nb0, nb1, indices, radius, center)) {
                // エッジの整合性を確認
                EdgePtr e0 = GetLinkingEdge(v, nb1);
                EdgePtr e1 = GetLinkingEdge(nb0, nb1);
                EdgePtr e2 = GetLinkingEdge(v, nb0);
                if ((e0 != nullptr && e0->type_ != Edge::Front) ||
                    (e1 != nullptr && e1->type_ != Edge::Front) ||
                    (e2 != nullptr && e2->type_ != Edge::Front)) {
                    continue;
                }

                // 三角形を作成
                CreateTriangle(v, nb0, nb1, center);

                // フロンティアエッジが追加された場合はシード成功
                if (!edge_front_.empty()) {
                    return true;
                }
            }
        }
    }

    return false;
}

void BallPivoting::FindSeedTriangle(double radius) {
    for (size_t vidx = 0; vidx < vertices_.size(); ++vidx) {
        VertexPtr v = vertices_[vidx];
        if (v->type_ == Vertex::Orphan) {
            if (TrySeed(v, radius)) {
                ExpandTriangulation(radius);
            }
        }
    }
}

void BallPivoting::ExpandTriangulation(double radius) {
    while (!edge_front_.empty()) {
        EdgePtr edge = edge_front_.front();
        edge_front_.pop_front();

        if (edge->type_ != Edge::Front) {
            continue;
        }

        Eigen::Vector3d center;
        VertexPtr candidate = FindCandidateVertex(edge, radius, center);

        if (candidate == nullptr || candidate->type_ == Vertex::Inner || !IsCompatible(candidate, edge->source_, edge->target_)) {
            edge->type_ = Edge::Border;
            border_edges_.push_back(edge);
            continue;
        }

        // エッジタイプがFrontであることを確認
        EdgePtr e0 = GetLinkingEdge(candidate, edge->source_);
        EdgePtr e1 = GetLinkingEdge(candidate, edge->target_);
        if ((e0 != nullptr && e0->type_ != Edge::Front) ||
            (e1 != nullptr && e1->type_ != Edge::Front)) {
            edge->type_ = Edge::Border;
            border_edges_.push_back(edge);
            continue;
        }

        CreateTriangle(edge->source_, edge->target_, candidate, center);

        if (e0 && e0->type_ == Edge::Front) {
            edge_front_.push_front(e0);
        }
        if (e1 && e1->type_ == Edge::Front) {
            edge_front_.push_front(e1);
        }
    }
}

pcl::PolygonMesh BallPivoting::Run(const std::vector<double>& radii) {
    pcl::PolygonMesh mesh;

    for (double radius : radii) {
        pcl::console::print_info("[Run] ################################\n");
        pcl::console::print_info("[Run] Processing radius: %.4f\n", radius);

        if (radius <= 0) {
            pcl::console::print_error("[Run] Warning: Invalid radius (%.4f) encountered. Skipping this radius.\n", radius);
            pcl::console::print_info("[Run] ################################\n");
            continue; // 無効な半径はスキップ
        }

        // 境界エッジの再チェック
        for (auto it = border_edges_.begin(); it != border_edges_.end();) {
            EdgePtr edge = *it;
            TrianglePtr triangle = edge->triangle0_;

            if (triangle == nullptr) {
                ++it;
                continue;
            }

            Eigen::Vector3d center;
            if (!ComputeBallCenter(triangle->vert0_->idx_, triangle->vert1_->idx_, triangle->vert2_->idx_, radius, center)) {
                ++it;
                continue;
            }

            std::vector<int> indices;
            std::vector<float> dists2;
            kdtree_.radiusSearch(pcl::PointXYZ(center.x(), center.y(), center.z()), radius, indices, dists2);

            bool empty_ball = true;
            for (auto idx : indices) {
                if (idx != triangle->vert0_->idx_ && idx != triangle->vert1_->idx_ && idx != triangle->vert2_->idx_) {
                    empty_ball = false;
                    break;
                }
            }

            if (empty_ball) {
                edge->type_ = Edge::Front;
                edge_front_.push_back(edge);
                it = border_edges_.erase(it);
                pcl::console::print_info("[Run] Edge (%d-%d) moved to front edges.\n", edge->source_->idx_, edge->target_->idx_);
                continue;
            }
            ++it;
        }

        // メッシュ再構築
        if (edge_front_.empty()) {
            FindSeedTriangle(radius);
        } else {
            ExpandTriangulation(radius);
        }

        pcl::console::print_info("[Run] Completed processing radius: %.4f\n", radius);
        pcl::console::print_info("[Run] Current number of triangles: %lu\n", triangles_.size());
        pcl::console::print_info("[Run] ################################\n");
    }

    // PCLメッシュの構築
    pcl::PointCloud<pcl::PointXYZ> pcl_points;
    pcl_points.reserve(vertices_.size());
    for (const auto& v : vertices_) {
        pcl_points.emplace_back(pcl::PointXYZ(v->point_.x(), v->point_.y(), v->point_.z()));
    }

    pcl::toPCLPointCloud2(pcl_points, mesh.cloud);
    mesh.polygons.resize(triangles_.size());
    for (size_t i = 0; i < triangles_.size(); ++i) {
        pcl::Vertices polygon;
        polygon.vertices.resize(3);
        polygon.vertices[0] = static_cast<uint32_t>(triangles_[i][0]);
        polygon.vertices[1] = static_cast<uint32_t>(triangles_[i][1]);
        polygon.vertices[2] = static_cast<uint32_t>(triangles_[i][2]);
        mesh.polygons[i] = polygon;
    }

    return mesh;
}


pcl::PolygonMesh RunBPA(const std::vector<Eigen::Vector3d>& points,
                        const std::vector<Eigen::Vector3d>& normals,
                        const std::vector<double>& radii) {
    BallPivoting bp(points, normals);
    return bp.Run(radii);
}
