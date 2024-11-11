#include "headers.h"
#include "SDF.h"

using namespace gp_planner;
using namespace std;
using namespace Eigen;

void add_obstacle(const std::vector<int>& position, const std::vector<int>& size, std::vector<gtsam::Matrix>& map, std::vector<gtsam::Vector>& corners) {
    int half_size_row = std::floor((size[0] - 1) / 2);
    int half_size_col = std::floor((size[1] - 1) / 2);
    int half_size_z = std::floor((size[2] - 1) / 2);

    for (int i = position[0] - half_size_row; i <= position[0] + half_size_row; ++i) {
        for (int j = position[1] - half_size_col; j <= position[1] + half_size_col; ++j) {
            for (int k = position[2] - half_size_z; k <= position[2] + half_size_z; ++k) {
                if (i >= 0 && i < map[0].rows() && j >= 0 && j < map[0].cols() && k >= 0 && k < map.size()) {
                    map[k](i, j) = 1.0;
                }
            }
        }
    }

    gtsam::Vector corner(6);
    corner << position[0] - half_size_row, position[0] + half_size_row,
             position[1] - half_size_col, position[1] + half_size_col,
             position[2] - half_size_z, position[2] + half_size_z;
    corners.push_back(corner);
}

std::vector<gtsam::Matrix> signedDistanceField3D(const std::vector<gtsam::Matrix>& ground_truth_map, double cell_size) {
    std::vector<gtsam::Matrix> field(ground_truth_map.size());
    for (size_t k = 0; k < ground_truth_map.size(); ++k) {
        field[k] = gtsam::Matrix::Zero(ground_truth_map[k].rows(), ground_truth_map[k].cols());

        for (int i = 0; i < ground_truth_map[k].rows(); ++i) {
            for (int j = 0; j < ground_truth_map[k].cols(); ++j) {
                if (ground_truth_map[k](i, j) > 0.75) {
                    double distance = std::numeric_limits<double>::max();
                    for (int ni = -1; ni <= 1; ++ni) {
                        for (int nj = -1; nj <= 1; ++nj) {
                            if (i + ni >= 0 && i + ni < ground_truth_map[k].rows() && j + nj >= 0 && j + nj < ground_truth_map[k].cols()) {
                                double dist = std::sqrt(std::pow(i + ni - j, 2) + std::pow(j + nj - j, 2));
                                distance = std::min(distance, dist);
                            }
                        }
                    }
                    field[k](i, j) = distance * cell_size;
                } else {
                    field[k](i, j) = -1.0;
                }
            }
        }
    }
    return field;
}

int main(int argc, char **argv){

    // 初始化数据集
    gtsam::Point3 origin(-1.0, -1.0, -0.5);
    double cell_size = 0.05;
    size_t rows = 40, cols = 40, z = 40;
    std::vector<gtsam::Matrix> map(z, gtsam::Matrix::Zero(rows, cols));

    // 添加障碍物
    std::vector<gtsam::Vector> corners;
    add_obstacle({20, 9, 10}, {3, 3, 3}, map, corners);
    add_obstacle({20, 20, 5}, {30, 30, 3}, map, corners);

    // 计算签名距离场
    auto field = signedDistanceField3D(map, cell_size);

    // 创建 SignedDistanceField 对象
    SDF sdf(origin, cell_size, rows, cols, z);
    for (size_t z = 0; z < field.size(); ++z) {
        sdf.initFieldData(z, field[z]);
    }
    sdf.print();

    std::cout << "Calculating signed distance field done" << std::endl;
    string file_name = "/home/roboert/MP_WS/src/gp_planner/src/local_planner/sdf_data.txt";
    sdf.saveSDF(file_name);
    for(int i=0;i<100000;i++);
    std::vector<gtsam::Matrix> map_2(z, gtsam::Matrix::Zero(rows, cols));
    SDF loaded_sdf;
    loaded_sdf.print();
    loaded_sdf.loadSDF(file_name);
    loaded_sdf.print();
    string file_name_2 = "/home/roboert/MP_WS/src/gp_planner/src/local_planner/sdf_data_2.txt";
    loaded_sdf.saveSDF(file_name_2);
    return 0;
}