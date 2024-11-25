#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> // 用于旋转和变换


/**
 * @brief 将 XYZ 和 RPY 转换为 4x4 的变换矩阵
 * @param x X 坐标，单位为毫米
 * @param y Y 坐标，单位为毫米
 * @param z Z 坐标，单位为毫米
 * @param roll 旋转角度（Roll，单位为角度）
 * @param pitch 旋转角度（Pitch，单位为角度）
 * @param yaw 旋转角度（Yaw，单位为角度）
 * @return Eigen::Matrix4d 4x4 齐次变换矩阵
 */
Eigen::Matrix4d rpyToMatrix(double x, double y, double z, double roll, double pitch, double yaw) {
    // 将角度转换为弧度
    double roll_rad = roll * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad = yaw * M_PI / 180.0;

    // 构造角轴旋转
    Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw_rad, Eigen::Vector3d::UnitZ());

    // 计算旋转矩阵
    Eigen::Matrix3d rotation = yawAngle.toRotationMatrix() * pitchAngle.toRotationMatrix() * rollAngle.toRotationMatrix();

    // 使用 Affine3d 直接构造变换矩阵
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.linear() = rotation;              // 设置旋转部分
    transform.translation() = Eigen::Vector3d(x / 1000.0, y / 1000.0, z / 1000.0); // 设置平移部分

    return transform.matrix(); // 返回 4x4 矩阵
}

// 从文件读取位姿数据
std::vector<Eigen::Matrix4d> readPosesFromFile(const std::string& filename) {
    std::vector<Eigen::Matrix4d> poses;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开位姿文件: " << filename << std::endl;
        return poses;
    }

    double x, y, z, roll, pitch, yaw;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        char comma;
        if (iss >> x >> comma >> y >> comma >> z >> comma >> roll >> comma >> pitch >> comma >> yaw) {
            poses.push_back(rpyToMatrix(x, y, z, roll, pitch, yaw));
        }
    }
    file.close();
    return poses;
}

// 检测指定的Aruco标定板（Marker ID为1）并计算相机位姿
std::vector<Eigen::Matrix4d> detectArucoMarkers(const std::string& folder, int num_images, const cv::Ptr<cv::aruco::Dictionary>& dictionary, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs, float marker_size) {
    std::vector<Eigen::Matrix4d> camera_poses;

    for (int i = 1; i <= num_images; ++i) {
        std::string image_path = folder + "/" + std::to_string(i) + "_Color.png";
        cv::Mat image = cv::imread(image_path);
        if (image.empty()) {
            std::cerr << "无法读取图片: " << image_path << std::endl;
            continue;
        }

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;

        // 检测Aruco码
        cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids);
        if (marker_ids.empty()) {
            std::cerr << "未检测到Aruco标记: " << image_path << std::endl;
            continue;
        }

        // 在图像上绘制检测到的Aruco标记
        cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);

        // 查找Marker ID = 1
        // 查找Marker ID = 1
        bool found = false;
        for (int j = 0; j < marker_ids.size(); ++j) {
            std::cout << "Marker ID: " << marker_ids[j] << std::endl;

            if (marker_ids[j] == 1) {
                // 检查检测到的角点是否为空
                if (marker_corners[j].empty()) {
                    std::cerr << "Marker ID = 1 的角点为空，跳过该标记。" << std::endl;
                    continue;
                }

                // 输出每个角点的坐标
                std::cout << "Marker corners: " << std::endl;
                for (int i = 0; i < marker_corners[j].size(); ++i) {
                    std::cout << "  Corner " << i << ": " << marker_corners[j][i] << std::endl;
                }

                // 确保marker_size是一个正数
                if (marker_size <= 0) {
                    std::cerr << "Marker size is non-positive! Check your marker size." << std::endl;
                    continue;
                }
                // 估计姿态
                std::vector<cv::Vec3d> rvecs, tvecs;
                rvecs.clear();
                tvecs.clear();
                 // 只传递ID为1的标记的角点
                std::vector<std::vector<cv::Point2f>> singleMarkerCorners;
                singleMarkerCorners.push_back(marker_corners[j]);
                cv::aruco::estimatePoseSingleMarkers(singleMarkerCorners, marker_size, camera_matrix, dist_coeffs, rvecs, tvecs);
                if (rvecs.size() > 0 && tvecs.size() > 0) {
                    std::cout << "Estimated rotation vector: " << rvecs[0] << std::endl;
                    std::cout << "Estimated translation vector: " << tvecs[0] << std::endl;
                } else {
                    std::cerr << "Pose estimation failed!" << std::endl;
                }

                found = true;

                // 绘制坐标轴
                cv::aruco::drawAxis(image, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], marker_size);

                // 将旋转向量转换为旋转矩阵
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvecs[0], rotation_matrix);

                // 输出旋转矩阵
                std::cout << "Rotation matrix: " << rotation_matrix << std::endl;

                // 将OpenCV的旋转矩阵转换为Eigen矩阵
                Eigen::Matrix3d rotation_eigen;
                rotation_eigen << rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
                                rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2);

                // 创建4x4的变换矩阵，并填充旋转和平移
                Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                pose.block<3, 3>(0, 0) = rotation_eigen;
                pose(0, 3) = tvecs[0][0]; 
                pose(1, 3) = tvecs[0][1];
                pose(2, 3) = tvecs[0][2];

                std::cout << "Pose matrix: " << pose << std::endl;

                camera_poses.push_back(pose);

                // 保存带有坐标轴的图片
                std::string output_image_path = folder + "/image_" + std::to_string(i) + "_pose.png";
                cv::imwrite(output_image_path, image);  // 保存图片
                std::cout << "保存图片: " << output_image_path << std::endl;

                break;
            }
        }

        if (!found) {
            std::cerr << "未找到Marker ID = 1。" << std::endl;
        }


        // 显示图像并等待按键
        cv::imshow("Aruco Marker Detection", image);
        cv::waitKey(500);  // 等待500毫秒后显示下一张图像
    }

    cv::destroyAllWindows();  // 关闭所有OpenCV显示窗口
    return camera_poses;
}

// 计算标定误差（例如，通过比较预测位姿与实际位姿）
double calculateCalibrationError(const std::vector<Eigen::Matrix4d>& robot_poses,
                                 const std::vector<Eigen::Matrix4d>& camera_poses,
                                 const cv::Mat& R_handeye, const cv::Mat& T_handeye) {
    double total_error = 0.0;
    int num_poses = robot_poses.size();

    for (int i = 0; i < num_poses; ++i) {
        // 将cv::Mat类型的R_handeye和T_handeye转换为Eigen类型
        Eigen::Matrix3d R_handeye_eigen;
        Eigen::Vector3d T_handeye_eigen;

        // 从cv::Mat中获取旋转矩阵和位移向量
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                R_handeye_eigen(r, c) = R_handeye.at<double>(r, c);
            }
            T_handeye_eigen(r) = T_handeye.at<double>(r, 0);
        }

        // 预测的位姿：R_handeye * camera_poses[i] + T_handeye
        Eigen::Matrix4d predicted_pose = Eigen::Matrix4d::Identity();
        predicted_pose.block<3, 3>(0, 0) = R_handeye_eigen;
        predicted_pose.block<3, 1>(0, 3) = T_handeye_eigen;

        // 真实的机器人位姿
        Eigen::Matrix4d actual_pose = robot_poses[i];

        // 计算平移误差
        Eigen::Vector3d translation_error = predicted_pose.block<3, 1>(0, 3) - actual_pose.block<3, 1>(0, 3);
        total_error += translation_error.norm();

        // 计算旋转误差
        Eigen::Matrix3d predicted_rotation = predicted_pose.block<3, 3>(0, 0);
        Eigen::Matrix3d actual_rotation = actual_pose.block<3, 3>(0, 0);

        Eigen::Matrix3d rotation_error = predicted_rotation.transpose() * actual_rotation;
        double angle_error = std::acos((rotation_error.trace() - 1.0) / 2.0);
        total_error += angle_error;
    }

    return total_error / num_poses;
}

// 用于保存标定结果到YAML文件
void saveCalibrationResults(const cv::Mat& R_handeye, const cv::Mat& T_handeye, const std::string& output_file) {
    cv::FileStorage fs(output_file, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "无法打开YAML文件进行写入: " << output_file << std::endl;
        return;
    }

    fs << "HandEyeCalibrationResult" << "{";
    fs << "RotationMatrix" << R_handeye;
    fs << "TranslationVector" << T_handeye;
    fs << "}";

    fs.release();  // 关闭文件
}

// 执行手眼标定
void performHandEyeCalibration(const std::vector<Eigen::Matrix4d>& robot_poses,
                               const std::vector<Eigen::Matrix4d>& camera_poses,
                               cv::Mat& R_handeye,cv::Mat& T_handeye,
                               const std::string& output_file) {
    std::vector<cv::Mat> R_gripper2base, T_gripper2base;
    std::vector<cv::Mat> R_camera2board, T_camera2board;

    // 将机器人位姿分为旋转矩阵和平移向量
    for (const auto& pose : robot_poses) {
        cv::Mat rotation_mat = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat translation_mat = cv::Mat::zeros(3, 1, CV_64F);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotation_mat.at<double>(i, j) = pose(i, j);
            }
            translation_mat.at<double>(i, 0) = pose(i, 3);
        }

        R_gripper2base.push_back(rotation_mat);
        T_gripper2base.push_back(translation_mat);
    }

    // 将相机位姿分为旋转矩阵和平移向量
    for (const auto& pose : camera_poses) {
        cv::Mat rotation_mat = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat translation_mat = cv::Mat::zeros(3, 1, CV_64F);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotation_mat.at<double>(i, j) = pose(i, j);
            }
            translation_mat.at<double>(i, 0) = pose(i, 3);
        }

        R_camera2board.push_back(rotation_mat);
        T_camera2board.push_back(translation_mat);
    }

    // 定义标定方法，通常使用 Tsai 方法
    cv::HandEyeCalibrationMethod method = cv::CALIB_HAND_EYE_TSAI;

    // 执行手眼标定
    cv::calibrateHandEye(R_gripper2base, T_gripper2base,
                         R_camera2board, T_camera2board,
                         R_handeye, T_handeye, method);

    // 输出手眼标定结果
    std::cout << "手眼标定结果 (旋转矩阵):\n" << R_handeye << std::endl;
    std::cout << "手眼标定结果 (平移向量):\n" << T_handeye << std::endl;

    // 保存标定结果到YAML文件
    saveCalibrationResults(R_handeye, T_handeye, output_file);

    // 计算并输出标定误差
    double calibration_error = calculateCalibrationError(robot_poses, camera_poses, R_handeye, T_handeye);
    std::cout << "标定误差: " << calibration_error << std::endl;
}

int main() {
    std::string image_folder = "/home/roboert/MP_WS/src/camera_calibration/images/handeye";
    std::string pose_file = image_folder + "/pose.txt";
    int num_images = 10;
    float marker_size = 0.08; // Aruco标记尺寸，单位米

    // 相机内参
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
        1329.390848067253, 0, 955.6659619447905,
        0, 1328.783888724376, 579.702502266785,
        0, 0, 1);

    // 畸变系数
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << 
        0.1382685429942082, -0.1952215931821368, 0.007095175790574298, -0.002257258892116025, -0.02426428955782208);

    // Aruco字典
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    // 读取位姿数据
    auto robot_poses = readPosesFromFile(pose_file);
    // 检测Aruco标定板
    auto camera_poses = detectArucoMarkers(image_folder, num_images, dictionary, camera_matrix, dist_coeffs, marker_size);

    if (robot_poses.size() == camera_poses.size()) {
        cv::Mat R_handeye, T_handeye;
        // 从performHandEyeCalibration函数中获取R_handeye和T_handeye
        std::string output_file = "/home/roboert/MP_WS/src/camera_calibration/handeye_calibration_result.yaml";  // 输出YAML文件路径

        performHandEyeCalibration(robot_poses, camera_poses,R_handeye,T_handeye,output_file);
    } else {
        std::cerr << "位姿数量与图片数量不匹配，无法进行标定。" << std::endl;
    }

    return 0;
}
