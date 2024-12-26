#include "BPA.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/print.h>
#include <vector>
#include <Eigen/Dense>

// 法線をざっくり揃える処理
void AlignNormalsToDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz,
                             pcl::PointCloud<pcl::Normal>::Ptr normals) {
    // 点群の重心を計算
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);
    for (const auto& point : cloud_xyz->points) {
        centroid += Eigen::Vector3d(point.x, point.y, point.z);
    }
    centroid /= static_cast<double>(cloud_xyz->size());

    // 各法線を重心方向に合わせる
    for (size_t i = 0; i < normals->size(); ++i) {
        Eigen::Vector3d normal(normals->points[i].normal_x,
                               normals->points[i].normal_y,
                               normals->points[i].normal_z);
        Eigen::Vector3d direction = Eigen::Vector3d(cloud_xyz->points[i].x,
                                                    cloud_xyz->points[i].y,
                                                    cloud_xyz->points[i].z) - centroid;

        // 法線と方向の内積を計算
        if (normal.dot(direction) < 0) {
            // 法線が反対方向の場合、反転
            normals->points[i].normal_x *= -1;
            normals->points[i].normal_y *= -1;
            normals->points[i].normal_z *= -1;
        }
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        pcl::console::print_error("Usage: %s <input_pcd_file>\n", argv[0]);
        return -1;
    }

    std::string input_pcd_file = argv[1];

    // PCDファイルを読み込む
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *cloud_xyz) == -1) {
        pcl::console::print_error("Failed to load PCD file: %s\n", input_pcd_file.c_str());
        return -1;
    }

    pcl::console::print_info("Loaded PCD file: %s\n", input_pcd_file.c_str());
    pcl::console::print_info("Point count: %lu\n", cloud_xyz->size());

    // 法線を推定
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_xyz);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03); // 検索半径を適宜調整
    ne.compute(*normals);

    pcl::console::print_info("Normals estimated.\n");

    // 法線を方向に揃える
    AlignNormalsToDirection(cloud_xyz, normals);
    pcl::console::print_info("Normals aligned to direction.\n");

    // 点群と法線を結合
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> eigen_normals;
    for (size_t i = 0; i < cloud_xyz->size(); ++i) {
        points.emplace_back(Eigen::Vector3d(cloud_xyz->points[i].x,
                                            cloud_xyz->points[i].y,
                                            cloud_xyz->points[i].z));
        eigen_normals.emplace_back(Eigen::Vector3d(normals->points[i].normal_x,
                                                   normals->points[i].normal_y,
                                                   normals->points[i].normal_z));
    }

    // BPAの半径リストを定義
    std::vector<double> radii = {0.001, 0.002, 0.003, 0.04, 0.005, 0.006, 0.007, 0.008, 0.009};

    // BPAを実行
    pcl::console::print_info("Running BPA...\n");
    pcl::PolygonMesh mesh = RunBPA(points, eigen_normals, radii);

        // メッシュの数をデバッグ出力
    size_t triangle_count = mesh.polygons.size();
    pcl::console::print_info("Generated triangles: %lu\n", triangle_count);

    // メッシュを保存
    std::string output_ply_file = "output_mesh.ply";
    pcl::io::savePLYFile(output_ply_file, mesh);
    pcl::console::print_info("Mesh saved to: %s\n", output_ply_file.c_str());

    return 0;
}

