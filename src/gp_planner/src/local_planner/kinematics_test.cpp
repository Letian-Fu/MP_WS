#include "ArmKinematics.h"
#include "headers.h"

using namespace gp_planner;
using namespace std;
using namespace Eigen;

int main(int argc, char **argv){
    int dof = 6;
    VectorXd a = VectorXd::Zero(6); 
    a << 0, -0.427, -0.357, 0, 0, 0;
    VectorXd d = VectorXd::Zero(6);
    d << 0.147, 0, 0, 0.141, 0.116, 0.105;
    VectorXd theta_bias = VectorXd::Zero(6);
    theta_bias << 0, -1.5708, 0, -1.5708, 0, 0;
    VectorXd alpha = VectorXd::Zero(6);
    alpha << 1.5708, 0.0, 0.0, 1.5708, -1.5708, 0.0;
    gtsam::Pose3 base_pose(gtsam::Rot3(), gtsam::Point3(0, 0, 0));
    Arm cr5(dof, a, alpha, d, base_pose, theta_bias);
    VectorXd conf = VectorXd::Zero(6);
    conf << 0, 0, 0, 0, 0, 0;
    vector<gtsam::Pose3> result;
    cr5.forwardKinematics(conf, boost::none, result, boost::none, boost::none, boost::none, boost::none);
    cout<<result.size()<<endl;
    for(size_t i=0;i<result.size();i++){
        cout<<result[i]<<endl;
    }
    return 0;
}