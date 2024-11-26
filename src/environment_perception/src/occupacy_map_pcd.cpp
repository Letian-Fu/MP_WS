#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <iostream>

int main(int argc, char** argv) {

    // 参数配置
    // std::string ground_pcd_file = "/home/roboert/MP_WS/src/environment_perception/pcd/ground_cloud.pcd";
    std::string clustered_pcd_file = "/home/roboert/MP_WS/src/environment_perception/pcd/clustered_cloud.pcd";

    double resolution = 0.02;  // OctoMap 分辨率
    // std::string output_file = "/home/roboert/MP_WS/src/environment_perception/octomap.bt";
    std::string output_file = "/home/roboert/MP_WS/src/environment_perception/octomap_without_ground.bt";

    // // 加载地面点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(ground_pcd_file, *ground_cloud) == -1) {
    //     std::cerr << "无法加载地面点云文件: " << ground_pcd_file << std::endl;
    //     return -1;
    // }

    // if (ground_cloud->points.empty()) {
    //     std::cerr << "地面点云为空，无法生成 OctoMap！" << std::endl;
    //     return -1;
    // }

    // std::cout << "地面点云加载成功，点的数量: " << ground_cloud->points.size() << std::endl;

    // 加载聚类点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(clustered_pcd_file, *clustered_cloud) == -1) {
        std::cerr << "无法加载聚类点云文件: " << clustered_pcd_file << std::endl;
        return -1;
    }

    if (clustered_cloud->points.empty()) {
        std::cerr << "聚类点云为空，无法生成 OctoMap！" << std::endl;
        return -1;
    }

    std::cout << "聚类点云加载成功，点的数量: " << clustered_cloud->points.size() << std::endl;

    // 创建 OctoMap
    octomap::OcTree tree(resolution);

    // // 添加地面点云到 OctoMap
    // std::cout << "开始添加地面点云到 OctoMap..." << std::endl;
    // for (const auto& point : ground_cloud->points) {
    //     tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);  // 插入地面点
    // }

    // 添加聚类点云到 OctoMap
    std::cout << "开始添加聚类点云到 OctoMap..." << std::endl;
    for (const auto& point : clustered_cloud->points) {
        tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);  // 插入聚类点
    }

    // 更新 OctoMap 的内部节点
    tree.updateInnerOccupancy();

    // 保存 OctoMap 到文件
    if (tree.writeBinary(output_file)) {
        std::cout << "OctoMap 已保存到文件: " << output_file << std::endl;
    } else {
        std::cerr << "无法保存 OctoMap" << std::endl;
    }

    return 0;
}