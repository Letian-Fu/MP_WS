#include <ros/ros.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>
#include "gp_planner/BoundingBoxArray.h"


// 全局变量
ros::Publisher pub_obstacle_info;
Eigen::Vector3d previous_positions;
double previous_velocity, previous_size;
Eigen::Vector3d previous_directions;
int image_width = 640;
int image_height = 480;
tf2_ros::Buffer tf_buffer;
tf2_ros::TransformListener *tf_listener_ptr = nullptr;
ros::Time previous_time;

bool is_ekf_initialized = false; // EKF初始化标志
bool is_ekf = false; //是否使用EKF

// // EKF相关变量（不考虑加速度）
// Eigen::VectorXd X(6);        // 状态向量 [x, y, z, vx, vy, vz]
// Eigen::MatrixXd P(6, 6);     // 状态协方差矩阵
// Eigen::MatrixXd F(6, 6);     // 状态转移矩阵
// Eigen::MatrixXd Q(6, 6);     // 过程噪声协方差矩阵
// Eigen::MatrixXd H(3, 6);     // 观测矩阵
// Eigen::MatrixXd R(3, 3);     // 测量噪声协方差矩阵

// void initializeEKF(double x, double y, double z) {
//     // 初始化状态向量（初始位置，速度为0）
//     X << x, y, z, 0, 0, 0;

//     // 初始化状态协方差矩阵 P
//     P.setIdentity();
//     P *= 1.0; // 初始协方差

//     // 初始化状态转移矩阵 F
//     F.setIdentity();
//     F(0, 3) = 1.0; // x_new = x + vx * dt
//     F(1, 4) = 1.0; // y_new = y + vy * dt
//     F(2, 5) = 1.0; // z_new = z + vz * dt

//     // 初始化过程噪声协方差矩阵 Q
//     Q.setIdentity();
//     Q *= 0.1; // 调整噪声参数

//     // 初始化观测矩阵 H （仅观测位置）
//     H.setZero();
//     H(0, 0) = 1.0; // x
//     H(1, 1) = 1.0; // y
//     H(2, 2) = 1.0; // z

//     // 初始化测量噪声协方差矩阵 R
//     R.setIdentity();
//     R *= 0.01; // 测量噪声

//     is_ekf_initialized = true;
// }

// void predictEKF(double dt) {
//     // 更新状态转移矩阵 F 的时间相关部分
//     F(0, 3) = dt;
//     F(1, 4) = dt;
//     F(2, 5) = dt;

//     // 预测状态
//     X = F * X;

//     // 预测协方差
//     P = F * P * F.transpose() + Q;
// }

// void updateEKF(double measured_x, double measured_y, double measured_z) {
//     // 构造测量值向量
//     Eigen::VectorXd Z(3);
//     Z << measured_x, measured_y, measured_z;

//     // 计算卡尔曼增益
//     Eigen::MatrixXd S = H * P * H.transpose() + R;
//     Eigen::MatrixXd K = P * H.transpose() * S.inverse();

//     // 更新状态向量
//     X = X + K * (Z - H * X);

//     // 更新协方差矩阵
//     Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
//     P = (I - K * H) * P;
// }

// EKF相关变量（考虑加速度）
Eigen::VectorXd X(9);        // 状态向量 [x, y, z, vx, vy, vz, ax, ay, az]
Eigen::MatrixXd P(9, 9);     // 状态协方差矩阵
Eigen::MatrixXd F(9, 9);     // 状态转移矩阵
Eigen::MatrixXd Q(9, 9);     // 过程噪声协方差矩阵
Eigen::MatrixXd H(3, 9);     // 观测矩阵
Eigen::MatrixXd R(3, 3);     // 测量噪声协方差矩阵

// 初始化EKF
void initializeEKF(double x, double y, double z) {
    // 初始化状态向量（初始位置，速度和加速度为0）
    X << x, y, z, 0, 0, 0, 0, 0, 0;

    // 初始化状态协方差矩阵 P
    P.setIdentity();
    P *= 1.0; // 初始协方差，可以根据场景调整

    // 初始化状态转移矩阵 F
    F.setIdentity();
    F(0, 3) = 1.0; // x_new = x + vx * dt
    F(1, 4) = 1.0; // y_new = y + vy * dt
    F(2, 5) = 1.0; // z_new = z + vz * dt
    // 位置与加速度的关系
    F(0, 6) = 0.5; // x_new += 0.5 * ax * dt^2
    F(1, 7) = 0.5; // y_new += 0.5 * ay * dt^2
    F(2, 8) = 0.5; // z_new += 0.5 * az * dt^2
    // 速度与加速度的关系
    F(3, 6) = 1.0; // vx_new = vx + ax * dt
    F(4, 7) = 1.0; // vy_new = vy + ay * dt
    F(5, 8) = 1.0; // vz_new = vz + az * dt

    // 初始化过程噪声协方差矩阵 Q
    Q.setIdentity();
    Q *= 0.1; // 初始过程噪声

    // 初始化观测矩阵 H （仅观测位置）
    H.setZero();
    H(0, 0) = 1.0; // 观测位置x
    H(1, 1) = 1.0; // 观测位置y
    H(2, 2) = 1.0; // 观测位置z

    // 初始化测量噪声协方差矩阵 R
    R.setIdentity();
    R *= 0.01; // 初始测量噪声

    previous_time = ros::Time::now(); // 初始化时间戳
    is_ekf_initialized = true;
}

// 动态调整过程噪声协方差矩阵 Q
void adjustProcessNoise(double dt) {
    // 当前速度向量
    Eigen::Vector3d velocity(X(3), X(4), X(5));
    double speed = velocity.norm();

    // 当前加速度向量
    Eigen::Vector3d acceleration(X(6), X(7), X(8));
    double accel_magnitude = acceleration.norm();

    // 动态调整 Q 的缩放因子
    double scale_factor = 1.0;
    if (accel_magnitude > 0.5) {  // 如果加速度较大，说明运动变化剧烈
        scale_factor = 10.0;      // 增大过程噪声
    } else if (speed < 0.1) {     // 如果速度接近0，说明物体静止
        scale_factor = 0.1;       // 减小过程噪声
    }

    // 重新设置 Q
    Q.setIdentity();
    Q *= scale_factor * dt;
}

// 动态调整测量噪声协方差矩阵 R
void adjustMeasurementNoise(const Eigen::VectorXd& residual) {
    // 计算残差的大小
    double residual_norm = residual.norm();

    // 动态调整 R
    double scale_factor = 1.0;
    if (residual_norm > 1.0) {  // 如果残差较大，说明测量值波动剧烈
        scale_factor = 10.0;   // 增大测量噪声
    } else {
        scale_factor = 0.1;    // 测量值较稳定，减小测量噪声
    }

    // 重新设置 R
    R.setIdentity();
    R *= scale_factor;
}

// 预测步骤
void predictEKF(double dt) {
    // 更新状态转移矩阵 F 的时间相关部分
    F.setIdentity();
    F(0, 3) = dt;              // x_new = x + vx * dt
    F(1, 4) = dt;              // y_new = y + vy * dt
    F(2, 5) = dt;              // z_new = z + vz * dt
    F(0, 6) = 0.5 * dt * dt;   // x_new += 0.5 * ax * dt^2
    F(1, 7) = 0.5 * dt * dt;   // y_new += 0.5 * ay * dt^2
    F(2, 8) = 0.5 * dt * dt;   // z_new += 0.5 * az * dt^2
    F(3, 6) = dt;              // vx_new = vx + ax * dt
    F(4, 7) = dt;              // vy_new = vy + ay * dt
    F(5, 8) = dt;              // vz_new = vz + az * dt

    // 动态调整过程噪声
    adjustProcessNoise(dt);

    // 预测状态
    X = F * X;

    // 更新协方差
    P = F * P * F.transpose() + Q;
}

// 更新步骤
void updateEKF(double measured_x, double measured_y, double measured_z) {
    // 构造测量值向量
    Eigen::VectorXd Z(3);
    Z << measured_x, measured_y, measured_z;

    // 计算残差
    Eigen::VectorXd residual = Z - H * X;

    // 动态调整测量噪声
    adjustMeasurementNoise(residual);

    // 计算卡尔曼增益
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    // 更新状态
    X = X + K * residual;

    // 更新协方差
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(X.size(), X.size());
    P = (I - K * H) * P;
}

// 回调函数：处理点云和边界框信息
void callback(const gp_planner::BoundingBoxArray::ConstPtr &bounding_boxes_msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 2. 提取 ROI 范围内的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);

    // 遍历所有边界框
    double left_x = bounding_boxes_msg->data[0];
    double right_x = left_x + bounding_boxes_msg->data[1];
    double up_y = bounding_boxes_msg->data[2];
    double down_y = up_y + bounding_boxes_msg->data[3];
    int center_x_pixel = (left_x + right_x)/2;
    int center_y_pixel = (up_y + down_y)/2;

    pass.setFilterFieldName("x");
    pass.setFilterLimits(left_x, right_x);
    pass.filter(*filtered_cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(up_y, down_y);
    pass.filter(*filtered_cloud);

    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.0, 20); // 根据实际情况设置 Z 轴范围
    pass.filter(*filtered_cloud);

    std::cout << "Filtered cloud size: " << filtered_cloud->size() << std::endl;


    if (filtered_cloud->size() == 0) {
        std::cerr << "No points found in ROI!" << std::endl;
        return;
    }

    // 增加噪声滤除
    pcl::PointCloud<pcl::PointXYZ>::Ptr denoised_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_cloud);
    sor.setMeanK(50); // 平均邻居点数
    sor.setStddevMulThresh(1.0); // 标准差倍数
    sor.filter(*denoised_cloud);

    if (denoised_cloud->size() == 0) {
        std::cerr << "No points found in ROI after noise removal!" << std::endl;
        return;
    }

    // 3. 对 ROI 内的点云进行聚类
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(denoised_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05); // 点之间的最大距离
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(denoised_cloud);
    ec.extract(cluster_indices);

    // 提取最大聚类
    pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZ>());
    int largest_cluster_size = 0;

    for (const auto &indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto &index : indices.indices) {
            cluster->points.push_back(filtered_cloud->points[index]);
        }
        if (cluster->size() > largest_cluster_size) {
            largest_cluster_size = cluster->size();
            *largest_cluster = *cluster;
        }
    }

    std::cout << "Largest cluster size: " << largest_cluster->size() << std::endl;

    if (largest_cluster->size() == 0) {
        std::cerr << "No significant cluster found!" << std::endl;
        return;
    }

    // 4. 使用 RANSAC 拟合球体
    pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> sac_seg;
    sac_seg.setOptimizeCoefficients(true);
    sac_seg.setModelType(pcl::SACMODEL_SPHERE);
    sac_seg.setMethodType(pcl::SAC_RANSAC);
    sac_seg.setDistanceThreshold(0.02); // 距离阈值
    sac_seg.setMaxIterations(1000);    // 增加迭代次数
    sac_seg.setRadiusLimits(0.05, 0.5); // 限制球体半径范围
    sac_seg.setInputCloud(largest_cluster);
    sac_seg.segment(*inliers, *coefficients_sphere);

    if (inliers->indices.empty()) {
        std::cerr << "No sphere found in the cluster!" << std::endl;
        std_msgs::Float64MultiArray obstacle_info;
        obstacle_info.data.push_back(previous_positions[0]);
        obstacle_info.data.push_back(previous_positions[1]);
        obstacle_info.data.push_back(previous_positions[2]);
        obstacle_info.data.push_back(previous_size);
        obstacle_info.data.push_back(previous_velocity);
        obstacle_info.data.push_back(previous_directions[0]);
        obstacle_info.data.push_back(previous_directions[1]);
        obstacle_info.data.push_back(previous_directions[2]);
        pub_obstacle_info.publish(obstacle_info);
        return;
    }

    // 5. 提取球体参数
    float center_x = coefficients_sphere->values[0];
    float center_y = coefficients_sphere->values[1];
    float center_z = coefficients_sphere->values[2];
    float radius = coefficients_sphere->values[3];
    float obs_size = radius * 2; // 球体的直径作为大小

    // 6. 转换为世界坐标系
    geometry_msgs::PointStamped sphere_center_cam;
    geometry_msgs::PointStamped sphere_center_world;

    // 设置球体中心点的相机坐标
    sphere_center_cam.header.frame_id = cloud_msg->header.frame_id;
    sphere_center_cam.header.stamp = cloud_msg->header.stamp;
    sphere_center_cam.point.x = center_x;
    sphere_center_cam.point.y = center_y;
    sphere_center_cam.point.z = center_z;

    try {
        //  geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform(
        //     "world", sphere_center_cam.header.frame_id, cloud_msg->header.stamp, ros::Duration(0.1));
        geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform(
            "base_link", sphere_center_cam.header.frame_id, cloud_msg->header.stamp, ros::Duration(0.1));
        tf2::doTransform(sphere_center_cam, sphere_center_world, transform_stamped);

        std::cout<<"Sphere center in world frame: ( "<< 
                sphere_center_world.point.x<<","<<
                sphere_center_world.point.y<<","<<
                sphere_center_world.point.z<<")"<<std::endl;

    } catch (tf2::TransformException &ex) {
        std::cerr <<"TF2 Transform Exception: "<< ex.what()<<std::endl;
        return;
    }

    double world_center_x = sphere_center_world.point.x;
    double world_center_y = sphere_center_world.point.y;
    double world_center_z = sphere_center_world.point.z;

    if(is_ekf){
        // 初始化或更新EKF
        if (!is_ekf_initialized) {
            initializeEKF(world_center_x, world_center_y, world_center_z);
        } else {
            // 计算时间差
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - previous_time).toSec();
            if (dt > 0) {
                predictEKF(dt); // 执行预测
                updateEKF(world_center_x, world_center_y, world_center_z); // 执行更新
            }
        }
        // 更新 previous_time
        previous_time = ros::Time::now();
        // 从EKF状态中提取位置和速度
        Eigen::Vector3d estimated_position(X(0), X(1), X(2));
        Eigen::Vector3d estimated_velocity(X(3), X(4), X(5));
        // 计算速度大小
        double velocity = estimated_velocity.norm();

        // 归一化速度向量，避免除以零的错误
        Eigen::Vector3d direction = (velocity > 1e-6) 
                                    ? estimated_velocity / velocity 
                                    : Eigen::Vector3d(0, 0, 0);
        // 8. 发布障碍物信息
        std_msgs::Float64MultiArray obstacle_info;
        obstacle_info.data.push_back(world_center_x);
        obstacle_info.data.push_back(world_center_y);
        obstacle_info.data.push_back(world_center_z);
        obstacle_info.data.push_back(obs_size);
        obstacle_info.data.push_back(velocity);
        obstacle_info.data.push_back(direction[0]);
        obstacle_info.data.push_back(direction[1]);
        obstacle_info.data.push_back(direction[2]);
        pub_obstacle_info.publish(obstacle_info);

        // 输出结果
        std::cout << "World center: " << world_center_x << ", " << world_center_y << ", " << world_center_z << std::endl;
        std::cout << "Size: " << obs_size << ", Velocity: " << estimated_velocity.norm() << ", Direction: " << direction.transpose() << std::endl;
    }
    else{
        // 获取当前时间
        ros::Time current_time = ros::Time::now();

        // 7. 计算运动方向和速度
        Eigen::Vector3d current_positions(world_center_x, world_center_y, world_center_z);
        Eigen::Vector3d direction = current_positions - previous_positions;
        double velocity = 0;
        // 检查时间差，避免初始状态或无效情况
        if (!previous_time.isZero()) {
            double time_diff = (current_time - previous_time).toSec(); // 计算时间差（秒）
            if (time_diff > 0) {
                velocity = direction.norm() / time_diff; // 速度 = 位移 / 时间
                if (velocity != 0) direction.normalize();
            }
        }
        direction.normalize();

        // 8. 发布障碍物信息
        std_msgs::Float64MultiArray obstacle_info;
        obstacle_info.data.push_back(world_center_x);
        obstacle_info.data.push_back(world_center_y);
        obstacle_info.data.push_back(world_center_z);
        obstacle_info.data.push_back(obs_size);
        obstacle_info.data.push_back(velocity);
        obstacle_info.data.push_back(direction[0]);
        obstacle_info.data.push_back(direction[1]);
        obstacle_info.data.push_back(direction[2]);
        pub_obstacle_info.publish(obstacle_info);

        // 输出结果
        std::cout << "World center: " << world_center_x << ", " << world_center_y << ", " << world_center_z << std::endl;
        std::cout << "Size: " << obs_size << ", Velocity: " << velocity << ", Direction: " << direction.transpose() << std::endl;

        previous_positions = current_positions;
        previous_size = obs_size;
        previous_directions = direction;
        previous_velocity = velocity;
        previous_time = ros::Time::now();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    // 创建 TF Buffer 和 Listener
    tf_listener_ptr = new tf2_ros::TransformListener(tf_buffer);

    // 创建发布者
    pub_obstacle_info = nh.advertise<std_msgs::Float64MultiArray>("/obstacle_info_detected", 10);
    message_filters::Subscriber<gp_planner::BoundingBoxArray> bouding_boxes_sub(nh, "/bounding_boxes", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/camera/depth_registered/points", 10);
    typedef message_filters::sync_policies::ApproximateTime<gp_planner::BoundingBoxArray,sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(50), bouding_boxes_sub, cloud_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();
    return 0;
}