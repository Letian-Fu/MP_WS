#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "pointcloud_to_octomap");
    ros::NodeHandle nh;

    // 发布 OctoMap
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 1, true);

    // 参数配置
    std::string input_pcd_file;
    nh.param<std::string>("input_pcd_file", input_pcd_file, "/home/roboert/MP_WS/src/environment_perception/clustered_cloud.pcd");

    double resolution;
    nh.param<double>("resolution", resolution, 0.1);

    std::string output_file;
    nh.param<std::string>("output_file", output_file, "/home/roboert/MP_WS/src/environment_perception/octomap.bt");

    const std::string frame_id = "camera_link";  // 坐标系

    // 加载点云文件
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *cloud) == -1) {
        std::cerr << "无法加载点云文件: " << input_pcd_file << std::endl;
        return -1;
    }

    if (cloud->points.empty()) {
        std::cerr << "点云为空，无法生成 OctoMap！" << std::endl;
        return -1;
    }

    std::cout << "点云加载成功，点的数量: " << cloud->points.size() << std::endl;

    // 创建 OctoMap
    octomap::OcTree tree(resolution);
    for (const auto& point : cloud->points) {
        tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }
    tree.updateInnerOccupancy();  // 更新 OctoMap 的内部节点

    // 保存 OctoMap 到文件
    if (tree.writeBinary(output_file)) {
        std::cout << "OctoMap 已保存到文件: " << output_file << std::endl;
    } else {
        std::cerr << "无法保存 OctoMap" << std::endl;
    }

    // 发布 OctoMap
    ros::Rate rate(0.2);  // 发布频率 0.2 Hz
    while (ros::ok()) {
        octomap_msgs::Octomap octomap_msg;
        // 确保正确转换为二进制地图消息
        if (!octomap_msgs::binaryMapToMsg(tree, octomap_msg)) {
            std::cerr << "无法将 OctoMap 转换为 ROS 消息格式" << std::endl;
            break;  // 如果转换失败，退出循环
        }

        // 设置消息头
        octomap_msg.header.frame_id = frame_id;
        octomap_msg.header.stamp = ros::Time::now();

        // 发布消息
        octomap_pub.publish(octomap_msg);
        std::cout << "已发布 OctoMap" << std::endl;

        rate.sleep();
    }

    return 0;
}