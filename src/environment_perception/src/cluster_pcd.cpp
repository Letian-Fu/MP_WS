#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/common/centroid.h>
#include <iostream>
#include <string>
#include <algorithm>
#include "DBSCAN_kdtree.h"


// 保存点云到文件的函数
void savePointCloudToFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& filename) {
    if (cloud->empty()) {
        std::cerr << "点云为空，未保存到文件: " << filename << std::endl;
        return;
    }
    if (pcl::io::savePCDFileASCII(filename, *cloud) == 0) {
        std::cout << "成功保存点云到文件: " << filename << std::endl;
    } else {
        std::cerr << "保存点云文件失败: " << filename << std::endl;
    }
}

void saveClusterToFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_cluster, const std::string& filename) {
    // 保存为 PCD 文件（ASCII 格式）
    if (cloud_cluster->empty()) {
        std::cerr << "点云为空，未保存到文件: " << filename << std::endl;
        return;
    }
    if (pcl::io::savePCDFileASCII(filename, *cloud_cluster) == 0) {
        std::cout << "成功保存点云到文件: " << filename << std::endl;
    } else {
        std::cout << "保存点云文件失败: " << filename << std::endl;
    }
}

int main() {
    // 检查输入参数，确保指定了 PCD 文件路径
    std::string input_pcd_file = "/home/roboert/MP_WS/src/environment_perception/original_cloud.pcd";

    // **步骤 1：加载原始点云**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *cloud) == -1) {
        std::cerr << "无法读取文件: " << input_pcd_file << std::endl;
        return -1;
    }
    std::cout << "加载点云成功，点数: " << cloud->points.size() << std::endl;

    // **步骤 2：范围滤波**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.3, 2.5);  // 保留 z 值在 0.5 到 2.0 米之间的点
    pass.filter(*cloud_filtered);

    if (cloud_filtered->empty()) {
        std::cerr << "范围滤波后点云为空！" << std::endl;
        return -1;
    }
    std::cout << "范围滤波后点云点数: " << cloud_filtered->points.size() << std::endl;

    // 保存范围滤波后的点云
    savePointCloudToFile(cloud_filtered, "/home/roboert/MP_WS/src/environment_perception/pcd/range_filtered_cloud.pcd");

    // **步骤 3：体素滤波**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_filtered);
    voxel_filter.setLeafSize(0.02f, 0.02f, 0.02f);  // 5cm 的体素大小
    voxel_filter.filter(*cloud_voxel_filtered);

    if (cloud_voxel_filtered->empty()) {
        std::cerr << "体素滤波后点云为空！" << std::endl;
        return -1;
    }
    std::cout << "体素滤波后点云点数: " << cloud_voxel_filtered->points.size() << std::endl;

    // 保存体素滤波后的点云
    savePointCloudToFile(cloud_voxel_filtered, "/home/roboert/MP_WS/src/environment_perception/pcd/voxel_filtered_cloud.pcd");

    // **步骤 3：高斯滤波（移动最小二乘法）**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gaussian_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(cloud_voxel_filtered);  // 对体素滤波后的点云进行高斯平滑
    mls.setSearchRadius(0.1);                 // 设置高斯核的搜索半径
    mls.setPolynomialOrder(2);                // 设置多项式阶数
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::NONE);  // 不插值
    mls.process(*cloud_gaussian_filtered);

    std::cout << "高斯滤波后点云点数: " << cloud_gaussian_filtered->points.size() << std::endl;
    // 保存高斯滤波后的点云
    savePointCloudToFile(cloud_gaussian_filtered, "/home/roboert/MP_WS/src/environment_perception/pcd/voxel_gaussian_cloud.pcd");

    // **步骤 4：RANSAC 平面分割**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_gaussian_filtered);
    seg.segment(*ground_inliers, *coefficients);

    if (ground_inliers->indices.empty()) {
        std::cerr << "未检测到地面平面！" << std::endl;
    } else {
        std::cout << "RANSAC 检测到地面平面，点数: " << ground_inliers->indices.size() << std::endl;

        // 提取地面点云
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_gaussian_filtered);
        extract.setIndices(ground_inliers);
        extract.setNegative(false);  // 提取地面点
        extract.filter(*cloud_ground);

        // 保存地面点云
        savePointCloudToFile(cloud_ground, "/home/roboert/MP_WS/src/environment_perception/pcd/ground_cloud.pcd");

        // 提取非地面点云
        extract.setNegative(true);  // 提取非地面点
        extract.filter(*cloud_non_ground);

        // 保存非地面点云
        savePointCloudToFile(cloud_non_ground, "/home/roboert/MP_WS/src/environment_perception/pcd/non_ground_cloud.pcd");
    }

    if (cloud_non_ground->empty()) {
        std::cerr << "非地面点云为空，无法进行聚类！" << std::endl;
        return -1;
    }

    // 检查地面点云范围
    std::cout << "地面点云范围: " << std::endl;
    for (const auto& point : cloud_ground->points) {
        std::cout << "x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
        break; // 打印前几个点即可
    }

    // 检查非地面点云范围
    std::cout << "非地面点云范围: " << std::endl;
    for (const auto& point : cloud_non_ground->points) {
        std::cout << "x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
        break; // 打印前几个点即可
    }


    std::cout << "总点数: " << cloud_gaussian_filtered->points.size() << std::endl;
    std::cout << "地面点数: " << cloud_ground->points.size() << std::endl;
    std::cout << "非地面点数: " << cloud_non_ground->points.size() << std::endl;

    // **步骤 5：DBSCAN 聚类**
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud_non_ground);
    // 保存聚类结果
    std::vector<pcl::PointIndices> cluster_indices;

    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // ec.setClusterTolerance(0.1); // 聚类半径（单位：米）
    // ec.setMinClusterSize(30);    // 每个聚类的最小点数
    // ec.setMaxClusterSize(10000); // 每个聚类的最大点数
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud_non_ground);
    // ec.extract(cluster_indices); // 提取聚类结果

    DBSCANKdtreeCluster<pcl::PointXYZ> ec;
    ec.setCorePointMinPts(20);
    ec.setClusterTolerance(0.1);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_non_ground);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
        std::cerr << "DBSCAN 聚类后无聚类结果！" << std::endl;
        return -1;
    }

    std::cout << "DBSCAN 聚类后，检测到 " << cluster_indices.size() << " 个簇。" << std::endl;

    // **步骤 5：保存和发布聚类结果**
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered(new pcl::PointCloud<pcl::PointXYZRGB>());
    int cluster_id = 0;
    for (const auto& indices : cluster_indices) {
        // 为每个簇分配随机颜色
        uint8_t r = rand() % 256;
        uint8_t g = rand() % 256;
        uint8_t b = rand() % 256;
        if(cluster_id == 0){
            cluster_id++;
            continue;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (const auto& index : indices.indices) {
            pcl::PointXYZRGB point;
            point.x = cloud_non_ground->points[index].x;
            point.y = cloud_non_ground->points[index].y;
            point.z = cloud_non_ground->points[index].z;
            point.r = r;
            point.g = g;
            point.b = b;
            cloud_clustered->points.push_back(point);
            single_cluster->points.push_back(point);
        }
        // 保存簇之前，确保点云的 width 和 height 设置正确
        single_cluster->width = single_cluster->points.size();
        single_cluster->height = 1;  // 非结构化点云
        single_cluster->is_dense = true;  // 假设点云没有无效点
        // 保存当前簇到文件
        std::string filename = "/home/roboert/MP_WS/src/environment_perception/pcd/cluster_" + std::to_string(cluster_id) + ".pcd";
        saveClusterToFile(single_cluster, filename);

        cluster_id++;
    }

    cloud_clustered->width = cloud_clustered->points.size();
    cloud_clustered->height = 1;
    cloud_clustered->is_dense = true;

    std::cout << "聚类后点云总点数: " << cloud_clustered->points.size() << std::endl;

    // 保存聚类后的完整点云
    saveClusterToFile(cloud_clustered, "/home/roboert/MP_WS/src/environment_perception/pcd/clustered_cloud.pcd");

    // 打印地面点云范围
    std::cout << "地面点云范围: " << std::endl;
    for (const auto& point : cloud_ground->points) {
        std::cout << "x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
        break; // 打印前几个点即可
    }

    // 打印聚类点云范围
    std::cout << "聚类点云范围: " << std::endl;
    for (const auto& point : cloud_clustered->points) {
        std::cout << "x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
        break; // 打印前几个点即可
    }
    // 合并地面点云和聚类结果点云并保存
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZRGB>());

    // 将地面点云转换为 RGB 格式并添加到 map 点云
    for (const auto& point : cloud_ground->points) {
        pcl::PointXYZRGB point_rgb;
        point_rgb.x = point.x;
        point_rgb.y = point.y;
        point_rgb.z = point.z;
        point_rgb.r = 128;  // 地面点云用灰色表示
        point_rgb.g = 128;
        point_rgb.b = 128;
        cloud_map->points.push_back(point_rgb);
    }

    // 将聚类结果点云添加到 map 点云
    for (const auto& point : cloud_clustered->points) {
        cloud_map->points.push_back(point);
    }
    std::cout << "cloud_map 总点数: " << cloud_map->points.size() << std::endl;
    std::cout << "地面点云点数: " << cloud_ground->points.size() << std::endl;
    std::cout << "聚类结果点云点数: " << cloud_clustered->points.size() << std::endl;

    double min_x = std::numeric_limits<double>::max(), max_x = std::numeric_limits<double>::lowest();
    double min_y = min_x, max_y = max_x;
    double min_z = min_x, max_z = max_x;

    for (const auto& point : cloud_map->points) {
        min_x = std::min(min_x, static_cast<double>(point.x));
        max_x = std::max(max_x, static_cast<double>(point.x));
        min_y = std::min(min_y, static_cast<double>(point.y));
        max_y = std::max(max_y, static_cast<double>(point.y));
        min_z = std::min(min_z, static_cast<double>(point.z));
        max_z = std::max(max_z, static_cast<double>(point.z));
    }

    std::cout << "合并点云范围:" << std::endl;
    std::cout << "x: [" << min_x << ", " << max_x << "], "
            << "y: [" << min_y << ", " << max_y << "], "
            << "z: [" << min_z << ", " << max_z << "]" << std::endl;

    // 设置点云的宽度和高度
    cloud_map->width = cloud_map->points.size();
    cloud_map->height = 1;
    cloud_map->is_dense = true;

    // 保存合并后的点云为 map_cloud.pcd
    saveClusterToFile(cloud_clustered, "/home/roboert/MP_WS/src/environment_perception/pcd/map_cloud.pcd");

    return 0;
}