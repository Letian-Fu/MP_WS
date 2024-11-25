#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/mls.h>
#include <iostream>

// 发布器
ros::Publisher pub_original;
ros::Publisher pub_range_filtered;
ros::Publisher pub_voxel_filtered;
ros::Publisher pub_gaussian_filtered;

// 回调函数：对点云进行处理
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // 将 ROS 点云消息转换为 PCL 格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    std::cout << "收到点云数据，原始点数: " << cloud->points.size() << std::endl;

    // 发布原始点云
    sensor_msgs::PointCloud2 output_original;
    pcl::toROSMsg(*cloud, output_original);
    output_original.header = cloud_msg->header;  // 保持原始点云坐标系
    pub_original.publish(output_original);

    // **步骤 1：范围滤波**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_range_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);                 // 对原始点云进行范围过滤
    pass.setFilterFieldName("z");              // 仅对 z 维度进行过滤
    pass.setFilterLimits(0.5, 2.0);            // 保留 z 在 0.5 到 2.0 米之间的点
    pass.setFilterLimitsNegative(false);       // 设为 true 则保留范围以外的点
    pass.filter(*cloud_range_filtered);

    std::cout << "范围滤波后点云点数: " << cloud_range_filtered->points.size() << std::endl;

    // 发布范围滤波后的点云
    sensor_msgs::PointCloud2 output_range;
    pcl::toROSMsg(*cloud_range_filtered, output_range);
    output_range.header = cloud_msg->header;
    pub_range_filtered.publish(output_range);

    // **步骤 2：体素滤波**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_range_filtered);  // 对范围滤波后的点云进行体素滤波
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);     // 5cm 的体素大小
    voxel_filter.filter(*cloud_voxel_filtered);

    std::cout << "体素滤波后点云点数: " << cloud_voxel_filtered->points.size() << std::endl;

    // 发布体素滤波后的点云
    sensor_msgs::PointCloud2 output_voxel;
    pcl::toROSMsg(*cloud_voxel_filtered, output_voxel);
    output_voxel.header = cloud_msg->header;
    pub_voxel_filtered.publish(output_voxel);

    // **步骤 3：高斯滤波（移动最小二乘法）**
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gaussian_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud(cloud_voxel_filtered);  // 对体素滤波后的点云进行高斯平滑
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
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "range_voxel_gaussian_filter");
    ros::NodeHandle nh;

    // 创建点云发布器
    pub_original = nh.advertise<sensor_msgs::PointCloud2>("original_points", 10);
    pub_range_filtered = nh.advertise<sensor_msgs::PointCloud2>("range_filtered_points", 10);
    pub_voxel_filtered = nh.advertise<sensor_msgs::PointCloud2>("voxel_filtered_points", 10);
    pub_gaussian_filtered = nh.advertise<sensor_msgs::PointCloud2>("gaussian_filtered_points", 10);

    // 订阅相机发布的点云话题
    ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 10, pointCloudCallback);

    std::cout << "点云处理节点已启动，等待点云数据..." << std::endl;
    ros::spin();

    return 0;
}