#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>

class PointCloudToOctomap {
public:
    PointCloudToOctomap() : nh_("~"), resolution_(0.1) {
        // 获取参数
        nh_.param<double>("resolution", resolution_, 0.1);

        // 发布 OctoMap
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap", 1, true);

        // 订阅点云消息
        pointcloud_sub_ = nh_.subscribe("/clustered_points", 1, &PointCloudToOctomap::pointCloudCallback, this);
        tf_listener_ptr_ = new tf2_ros::TransformListener(tf_buffer_);
        std::cout << "节点初始化完成，等待点云数据..." << std::endl;
    }

private:
    ros::NodeHandle nh_;                          // ROS 节点句柄
    ros::Subscriber pointcloud_sub_;              // 点云订阅者
    ros::Publisher octomap_pub_;                  // OctoMap 发布者
    tf2_ros::Buffer tf_buffer_;                   // TF2 缓存
    tf2_ros::TransformListener *tf_listener_ptr_ = nullptr;      // TF2 监听器
    
    double resolution_;                           // OctoMap 分辨率

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        // 获取点云的 frame_id
        std::string frame_id = cloud_msg->header.frame_id;
        // 转换点云消息为 PCL 格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

        // 转换点云到 base_link 坐标系
        sensor_msgs::PointCloud2 cloud_transformed;
        // try {
        //     // 查找从点云原始坐标系到 base_link 的变换
        //     geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
        //         "world", frame_id, cloud_msg->header.stamp, ros::Duration(0.1));

        //     // 使用 TF2 转换点云
        //     tf2::doTransform(*cloud_msg, cloud_transformed, transform_stamped);
        // } catch (tf2::TransformException& ex) {
        //     ROS_ERROR("TF2 Transform Exception: %s", ex.what());
        //     return;
        // }
    //    pcl::fromROSMsg(cloud_transformed, *cloud);
        
 
        pcl::fromROSMsg(*cloud_msg, *cloud);

        if (cloud->points.empty()) {
            std::cerr << "接收到的点云为空，无法生成 OctoMap！" << std::endl;
            return;
        }

        std::cout << "接收到点云，点的数量: " << cloud->points.size() << ", frame_id: " << frame_id << std::endl;

        // 创建 OctoMap
        octomap::OcTree tree(resolution_);
        for (const auto& point : cloud->points) {
            tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
        }
        tree.updateInnerOccupancy();  // 更新 OctoMap 的内部节点

        // 将 OctoMap 转换为 ROS 消息
        octomap_msgs::Octomap octomap_msg;
        if (!octomap_msgs::binaryMapToMsg(tree, octomap_msg)) {
            std::cerr << "无法将 OctoMap 转换为 ROS 消息格式" << std::endl;
            return;
        }

        // 设置消息头
        octomap_msg.header.frame_id = frame_id;  // 使用点云的 frame_id
        // octomap_msg.header.frame_id = "world";  // 设置为世界坐标系 base_link
        octomap_msg.header.stamp = ros::Time::now();

        // 发布 OctoMap
        octomap_pub_.publish(octomap_msg);
        std::cout << "已发布 OctoMap，frame_id: " << frame_id << std::endl;
    }
};

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "pointcloud_to_octomap");

    // 创建节点
    PointCloudToOctomap node;

    // 运行 ROS 循环
    ros::spin();

    return 0;
}