#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/mls.h>
#include <iostream>
#include <string>
#include "DBSCAN_kdtree.h"

// 发布器
ros::Publisher pub_original;
ros::Publisher pub_clustered;
ros::Publisher pub_range_filtered;
ros::Publisher pub_voxel_filtered;
ros::Publisher pub_gaussian_filtered;
ros::Publisher pub_ground;
ros::Publisher pub_non_ground;


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

// 回调函数：对点云进行处理
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // 将 ROS 点云消息转换为 PCL 格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (cloud->empty()) {
        std::cerr << "输入点云为空！" << std::endl;
        return;
    }

    std::cout << "收到点云数据，原始点数: " << cloud->points.size() << std::endl;
    // **保存原始点云数据**
    std::string original_filename = "/home/roboert/MP_WS/src/environment_perception/original_cloud.pcd";
    savePointCloudToFile(cloud, original_filename);

    // **步骤 1：范围滤波**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_range_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);                 // 对原始点云进行范围过滤
    pass.setFilterFieldName("z");              // 仅对 z 维度进行过滤
    pass.setFilterLimits(0.3, 2.5);            // 保留 z 在 0.5 到 2.0 米之间的点
    pass.setFilterLimitsNegative(false);       // 设为 true 则保留范围以外的点
    pass.filter(*cloud_range_filtered);

    std::cout << "范围滤波后点云点数: " << cloud_range_filtered->points.size() << std::endl;

    // 发布范围滤波后的点云
    sensor_msgs::PointCloud2 output_filtered;
    pcl::toROSMsg(*cloud_range_filtered, output_filtered);
    output_filtered.header = cloud_msg->header;
    pub_range_filtered.publish(output_filtered);

    // **步骤 2：体素滤波**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_range_filtered);  // 对范围滤波后的点云进行体素滤波
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);     // 5cm 的体素大小
    voxel_filter.filter(*cloud_voxel_filtered);

    std::cout << "体素滤波后点云点数: " << cloud_voxel_filtered->points.size() << std::endl;

    // 发布体素滤波后的点云
    sensor_msgs::PointCloud2 output_voxel_filtered;
    pcl::toROSMsg(*cloud_voxel_filtered, output_voxel_filtered);
    output_voxel_filtered.header = cloud_msg->header;
    pub_voxel_filtered.publish(output_voxel_filtered);

    // **步骤 3：高斯滤波（移动最小二乘法）**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gaussian_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(cloud_voxel_filtered);  // 对体素滤波后的点云进行高斯平滑
    // mls.setInputCloud(cloud_range_filtered);  // 对体素滤波后的点云进行高斯平滑
    mls.setSearchRadius(0.1);                 // 设置高斯核的搜索半径
    mls.setPolynomialOrder(2);                // 设置多项式阶数
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::NONE);  // 不插值
    mls.process(*cloud_gaussian_filtered);

    std::cout << "高斯滤波后点云点数: " << cloud_gaussian_filtered->points.size() << std::endl;

    // 发布高斯滤波后的点云
    sensor_msgs::PointCloud2 output_gaussian;
    pcl::toROSMsg(*cloud_gaussian_filtered, output_gaussian);
    output_gaussian.header = cloud_msg->header;
    pub_gaussian_filtered.publish(output_gaussian);

    // **步骤 3：RANSAC 平面分割**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_non_ground(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices()); // 地面点索引
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients()); // 平面模型系数

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);       // 优化模型系数
    seg.setModelType(pcl::SACMODEL_PLANE);   // 设置模型类型为平面
    seg.setMethodType(pcl::SAC_RANSAC);      // 使用 RANSAC 算法
    seg.setDistanceThreshold(0.1);          // 平面距离阈值（单位：米）
    seg.setInputCloud(cloud_gaussian_filtered);
    seg.segment(*ground_inliers, *coefficients);

    if (ground_inliers->indices.empty()) {
        std::cout << "未检测到地面平面！" << std::endl;
    } else {
        std::cout << "RANSAC 检测到地面平面，点数: " << ground_inliers->indices.size() << std::endl;

        // 提取地面点云
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_gaussian_filtered);
        extract.setIndices(ground_inliers);
        extract.setNegative(false); // 提取地面点
        extract.filter(*cloud_ground);

        // 提取非地面点云
        extract.setNegative(true); // 提取非地面点
        extract.filter(*cloud_non_ground);
    }

    std::cout << "总点数: " << cloud_gaussian_filtered->points.size() << std::endl;
    std::cout << "地面点数: " << cloud_ground->points.size() << std::endl;
    std::cout << "非地面点数: " << cloud_non_ground->points.size() << std::endl;

    // 发布地面点云
    sensor_msgs::PointCloud2 output_ground;
    pcl::toROSMsg(*cloud_ground, output_ground);
    output_ground.header = cloud_msg->header;
    pub_ground.publish(output_ground);

    // 发布非地面点云
    sensor_msgs::PointCloud2 output_non_ground;
    pcl::toROSMsg(*cloud_non_ground, output_non_ground);
    output_non_ground.header = cloud_msg->header;
    pub_non_ground.publish(output_non_ground);


    // **步骤 4：聚类（对非地面点云）**
    // 创建 KD-Tree 用于加速搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud_non_ground);

    // 保存聚类结果
    std::vector<pcl::PointIndices> cluster_indices;

    // **步骤 4：欧氏距离聚类（对非地面点云）**
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1); // 聚类半径（单位：米）
    ec.setMinClusterSize(50);    // 每个聚类的最小点数
    ec.setMaxClusterSize(10000); // 每个聚类的最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_non_ground);
    ec.extract(cluster_indices); // 提取聚类结果

    // std::cout << "欧式 聚类后，检测到 " << cluster_indices.size() << " 个簇。" << std::endl;

    // **步骤 4：DBSCAN 聚类（对非地面点云）**
    // pcl::DBSCANClusterExtraction<pcl::PointXYZ> dbscan;
    // dbscan.setInputCloud(cloud_non_ground);
    // dbscan.setEpsilon(0.1);         // 设置 DBSCAN 的邻域半径（单位：米）
    // dbscan.setMinClusterSize(50);   // 设置簇的最小点数
    // dbscan.setMaxClusterSize(10000); // 设置簇的最大点数
    // dbscan.extract(cluster_indices);

    // DBSCANKdtreeCluster<pcl::PointXYZ> ec;
    // ec.setCorePointMinPts(20);
    // ec.setClusterTolerance(0.1);
    // ec.setMinClusterSize(50);
    // ec.setMaxClusterSize(10000);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud_non_ground);
    // ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
        std::cerr << "DBSCAN 聚类后无聚类结果！" << std::endl;
        return;
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
        std::string filename = "/home/roboert/MP_WS/src/environment_perception/cluster_" + std::to_string(cluster_id) + ".pcd";
        // saveClusterToFile(single_cluster, filename);

        cluster_id++;
    }

    cloud_clustered->width = cloud_clustered->points.size();
    cloud_clustered->height = 1;
    cloud_clustered->is_dense = true;

    std::cout << "聚类后点云总点数: " << cloud_clustered->points.size() << std::endl;

    // 发布聚类后的点云
    sensor_msgs::PointCloud2 output_clustered;
    pcl::toROSMsg(*cloud_clustered, output_clustered);
    output_clustered.header = cloud_msg->header;
    pub_clustered.publish(output_clustered);

    // 保存聚类后的完整点云
    saveClusterToFile(cloud_clustered, "/home/roboert/MP_WS/src/environment_perception/clustered_cloud.pcd");
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "pointcloud_processing_with_dbscan");
    ros::NodeHandle nh;

    // 创建点云发布器
    pub_original = nh.advertise<sensor_msgs::PointCloud2>("original_points", 10);
    pub_range_filtered = nh.advertise<sensor_msgs::PointCloud2>("range_filtered_points", 10);
    pub_voxel_filtered = nh.advertise<sensor_msgs::PointCloud2>("voxel_filtered_points", 10);
    pub_gaussian_filtered = nh.advertise<sensor_msgs::PointCloud2>("gaussian_filtered_points", 10);
    pub_clustered = nh.advertise<sensor_msgs::PointCloud2>("clustered_points", 10);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("ground_points", 10);
    pub_non_ground = nh.advertise<sensor_msgs::PointCloud2>("non_ground_points", 10);

    // 订阅相机发布的点云话题
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 10, pointCloudCallback);
    // ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 10, pointCloudCallback);

    std::cout << "点云处理节点已启动，等待点云数据..." << std::endl;
    ros::spin();

    return 0;
}