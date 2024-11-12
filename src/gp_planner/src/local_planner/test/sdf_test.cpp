#include "headers.h"
#include "SDF.h"

using namespace gp_planner;
using namespace std;
using namespace Eigen;

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