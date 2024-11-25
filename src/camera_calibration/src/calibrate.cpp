#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <string>

class CameraCalibrator {
public:
    CameraCalibrator() {
        chessboard_size_ = cv::Size(7, 4); // 内角点数
        square_size_ = 0.015; // 每个方格的实际大小（单位：米）

        // 准备对象点
        for (int i = 0; i < chessboard_size_.height; i++) {
            for (int j = 0; j < chessboard_size_.width; j++) {
                objp_.push_back(cv::Point3f(j * square_size_, i * square_size_, 0));
            }
        }
    }

    void calibrate(const std::vector<std::string>& image_paths) {
        std::vector<std::vector<cv::Point2f>> imgpoints;
        cv::Mat image, gray;
        int i=1;
        // 读取每张图像
        for (const auto& path : image_paths) {
            image = cv::imread(path);
            if (image.empty()) {
                ROS_WARN("Could not open or find the image: %s", path.c_str());
                continue;
            }
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, chessboard_size_, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

            if (found) {
                imgpoints.push_back(corners);
                cv::drawChessboardCorners(image, chessboard_size_, corners, found);
                std::string output_image_path = "/home/roboert/MP_WS/src/camera_calibration/images/calibrate/image_" + std::to_string(i) + "_corners.png";
                cv::imwrite(output_image_path, image);  // 保存图片
                std::cout << "保存图片: " << output_image_path << std::endl;
                cv::imshow("Chessboard Corners", image);
                cv::waitKey(500); // 显示500ms
            } else {
                ROS_WARN("Chessboard corners not found in image: %s", path.c_str());
            }
            i++;
        }

        cv::destroyAllWindows();

        // 执行标定
        std::vector<std::vector<cv::Point3f>> objpoints(imgpoints.size(), objp_);
        // 执行标定
        cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F); // 初始相机矩阵为单位矩阵
        cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64F); // 初始畸变系数为0
        std::vector<cv::Mat> rvecs, tvecs;

        double rms = cv::calibrateCamera(objpoints, imgpoints, gray.size(), camera_matrix, dist_coeffs, rvecs, tvecs);

        // 打印内参矩阵和重投影误差
        std::cout << "Camera matrix:\n" << camera_matrix << std::endl;
        std::cout << "Distortion coefficients:\n" << dist_coeffs << std::endl;
        std::cout << "RMS reprojection error: " << rms << std::endl;

        // 保存标定结果
        cv::FileStorage fs("/home/roboert/MP_WS/src/camera_calibration/camera_calibration.yaml", cv::FileStorage::WRITE);
        fs << "camera_matrix" << camera_matrix;
        fs << "distortion_coefficients" << dist_coeffs;
        fs.release();
        ROS_INFO("Calibration completed and saved to camera_calibration.yaml.");

        // 计算重投影误差
        calculateReprojectionError(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist_coeffs);

    }

private:
    cv::Size chessboard_size_;
    float square_size_;
    std::vector<cv::Point3f> objp_;
    void calculateReprojectionError(const std::vector<std::vector<cv::Point3f>>& objpoints,
                                     const std::vector<std::vector<cv::Point2f>>& imgpoints,
                                     const std::vector<cv::Mat>& rvecs,
                                     const std::vector<cv::Mat>& tvecs,
                                     const cv::Mat& camera_matrix,
                                     const cv::Mat& dist_coeffs) {
        double total_error = 0.0;
        int total_points = 0;

        for (int i = 0; i < objpoints.size(); ++i) {
            std::vector<cv::Point2f> projected_points;
            // 计算投影点
            cv::projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, projected_points);

            // 计算每个点的误差
            double err = cv::norm(imgpoints[i], projected_points, cv::NORM_L2);
            total_error += err * err; // 误差平方和
            total_points += objpoints[i].size(); // 所有图像点的总数
        }

        // 输出总的重投影误差
        double mean_error = std::sqrt(total_error / total_points);
        std::cout << "Mean reprojection error: " << mean_error << std::endl;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_calibrator");

    CameraCalibrator calibrator;

    // 定义图像路径
    std::vector<std::string> image_paths = {
        "/home/roboert/MP_WS/src/camera_calibration/images/calibrate/calib_1_Color.png",
        "/home/roboert/MP_WS/src/camera_calibration/images/calibrate/calib_2_Color.png",
        "/home/roboert/MP_WS/src/camera_calibration/images/calibrate/calib_3_Color.png",
        "/home/roboert/MP_WS/src/camera_calibration/images/calibrate/calib_4_Color.png",
        "/home/roboert/MP_WS/src/camera_calibration/images/calibrate/calib_5_Color.png",
        "/home/roboert/MP_WS/src/camera_calibration/images/calibrate/calib_6_Color.png"
    };

    calibrator.calibrate(image_paths);

    return 0;
}