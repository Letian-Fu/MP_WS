#include "headers.h"
#include "ArmModel.h"
#include "kinematics.h"

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
    // cout<<result.size()<<endl;
    // for(size_t i=0;i<result.size();i++){
    //     cout<<result[i]<<endl;
    // }
    MatrixXd result2 = rrt_planner::transformMatrix_DH(alpha,a,d,theta_bias,conf);

    vector<BodySphere> body_spheres;
    VectorXd xs(14),zs(14),ys(14),rs(14);
    vector<size_t> js={0,0,1,1,1,1,1,1,2,2,2,3,4,5};
    xs<<0,0,0,0.105,0.210,0.315,0.420,0,0.11,0.22,0,0,0,0;
    ys<<-0.1,0,0,0,0,0,0,0,0,0,0,0,0,0;
    zs<<0,0,0.1,0.1,0.1,0.1,0.1,0,0,0,0,0,0,0;
    rs<<0.08,0.08,0.06,0.06,0.06,0.06,0.06,0.08,0.06,0.06,0.08,0.07,0.06,0.06;
    for(int i=0;i<1;i++){
        body_spheres.push_back(BodySphere(js[i],rs[i],gtsam::Point3(xs[i],ys[i],zs[i])));
    }

    ArmModel cr5_model(cr5,body_spheres);
    vector<gtsam::Point3> sph_centers;
    cr5_model.sphereCenters(conf,sph_centers, boost::none);
    for(size_t i=0;i<sph_centers.size();i++){
        cout<<i<<endl<<sph_centers[i].transpose()<<endl;
    }
    return 0;
}