#ifndef GP_PLANNER_ARMKINEMATICS_H
#define GP_PLANNER_ARMKINEMATICS_H

#pragma once

#include "headers.h"

namespace gp_planner{

class Arm{
private:
    size_t dof_;        // degree of freedom
    size_t nr_links_;   // number of links
    gtsam::Vector a_, alpha_, d_;  // raw DH parameters
    mutable gtsam::Pose3 base_pose_;  // base pose of the first link, allow change in const
    gtsam::Vector theta_bias_;  // bias of theta
    std::vector<gtsam::Pose3> link_trans_notheta_;  // transformation of each link, no theta matrix

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef gtsam::Vector Pose;
    typedef gtsam::Vector Velocity;
    /// default contructor, for serialization
    Arm() {}

    /// Contructor take in number of joints for the arm, its DH parameters
    /// the base pose (default zero pose), and theta bias (default zero)
    Arm(size_t dof, const gtsam::Vector& a, const gtsam::Vector& alpha,
        const gtsam::Vector& d)
        : Arm(dof, a, alpha, d,
            gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
            gtsam::Vector::Zero(dof)) {}

    Arm(size_t dof, const gtsam::Vector& a, const gtsam::Vector& alpha,
        const gtsam::Vector& d, const gtsam::Pose3& base_pose)
        : Arm(dof, a, alpha, d, base_pose, gtsam::Vector::Zero(dof)) {}

    Arm(size_t dof, const gtsam::Vector& a, const gtsam::Vector& alpha,
        const gtsam::Vector& d, const gtsam::Pose3& base_pose,
        const gtsam::Vector& theta_bias);

    /// Default destructor
    ~Arm();
    
    /**
   *  Forward kinematics: joint configuration to poses in workspace
   *  Velocity kinematics: optional joint velocities to linear velocities in
   *workspace, no anuglar rate
   *
   *  @param jp joint position in config space
   *  @param jv joint velocity in config space
   *  @param jpx joint pose in work space
   *  @param jvx joint velocity in work space
   *  @param J_jpx_jp et al. optional Jacobians
   **/
    void forwardKinematics(
        const gtsam::Vector& jp, boost::optional<const gtsam::Vector&> jv,
        std::vector<gtsam::Pose3>& jpx,
        boost::optional<std::vector<gtsam::Vector3>&> jvx,
        boost::optional<std::vector<gtsam::Matrix>&> J_jpx_jp = boost::none,
        boost::optional<std::vector<gtsam::Matrix>&> J_jvx_jp = boost::none,
        boost::optional<std::vector<gtsam::Matrix>&> J_jvx_jv = boost::none) const;
    
    /// update base pose in const
    void updateBasePose(const gtsam::Pose3& p) const { base_pose_ = p; }

    /// accesses
    size_t dof() const { return dof_; }
    size_t nr_links() const { return nr_links_; }
    const gtsam::Vector& a() const { return a_; }
    const gtsam::Vector& d() const { return d_; }
    const gtsam::Vector& alpha() const { return alpha_; }
    const gtsam::Pose3& base_pose() const { return base_pose_; }

private:
    /// Calculate the homogenous tranformation and matrix for joint j with angle
    /// theta in the configuration space
    gtsam::Pose3 getJointTrans(size_t i, double theta) const {
        assert(i < dof());
        // DH transformation for each link, with theta matrix
        return gtsam::Pose3(gtsam::Rot3::Rz(theta + theta_bias_(i)),
                            gtsam::Point3(0, 0, 0)) *
            link_trans_notheta_[i];
    }

    gtsam::Matrix4 getH(size_t i, double theta) const {
        return getJointTrans(i, theta).matrix();
    }

    /// dH/dtheta
    gtsam::Matrix4 getdH(size_t i, double theta) const {
        assert(i < dof());
        const double c = cos(theta + theta_bias_(i)),
                    s = sin(theta + theta_bias_(i));
        const gtsam::Matrix4 dRot =
            (gtsam::Matrix4() << -s, -c, 0, 0, c, -s, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                .finished();
        return dRot * link_trans_notheta_[i].matrix();
    }

    // construct skewMatrix based on rotating vector
    inline gtsam::Matrix3 skewSymmetric(const gtsam::Vector& w) const{
        assert(w.size() == 3);
        return (gtsam::Matrix3()<< 
            0, -w(2), w(1),
            w(2), 0, -w(0),
            -w(1), w(0), 0).finished();
    }

    /**
     * Calculate a single column j of the Jacobian (Jv(j)) for a given link.
     * This function computes the Jacobian column related to the linear velocity
     * of a specific link in the robot, given the transformation matrices from the
     * origin frame to the frames of two consecutive links.
     *
     * @param Hoi The 4x4 homogeneous transformation matrix from the origin frame to the i-th link frame.
     * @param Hoj The 4x4 homogeneous transformation matrix from the origin frame to the j-th link frame.
     * @return The calculated Jacobian column as a dynamic-sized vector.
     */
    gtsam::Vector3 getJvj(const gtsam::Matrix4& Hoi,
                            const gtsam::Matrix4& Hoj) const {
        assert(Hoi.cols() >= 4);
        assert(Hoj.cols() >= 4);
        // z axis vector in the origin tranformation
        gtsam::Vector3 z_axis_j = Hoj.col(2).head<3>();
        gtsam::Vector3 pos_o_i = Hoi.col(3).head<3>();
        gtsam::Vector3 pos_o_j = Hoj.col(3).head<3>();

        gtsam::Matrix3 rot_z_j = skewSymmetric(z_axis_j);
        gtsam::Vector3 diff_pos = pos_o_i - pos_o_j;

        return rot_z_j * diff_pos;
    }

    /**
     * Calculate the derivative of a single column j of the Jacobian (Jv(j)) for a given link.
     * This function computes the derivative of the Jacobian column related to the linear velocity
     * of a specific link in the robot, given the transformation matrices and their derivatives
     * from the origin frame to the frames of two consecutive links.
     *
     * @param Hoi The 4x4 homogeneous transformation matrix from the origin frame to the i-th link frame.
     * @param Hoj The 4x4 homogeneous transformation matrix from the origin frame to the j-th link frame.
     * @param dHoi The derivative of the 4x4 homogeneous transformation matrix from the origin frame to the i-th link frame.
     * @param dHoj The derivative of the 4x4 homogeneous transformation matrix from the origin frame to the j-th link frame.
     * @return The calculated derivative of the Jacobian column as a dynamic-sized vector.
     */
    gtsam::Vector3 getdJvj(const gtsam::Matrix4& Hoi, const gtsam::Matrix4& Hoj,
                            const gtsam::Matrix4& dHoi,
                            const gtsam::Matrix4& dHoj) const {
        assert(Hoi.cols() >= 4);
        assert(Hoj.cols() >= 4);
        assert(dHoi.cols() >= 4);
        assert(dHoj.cols() >= 4);

        // Extract the z-axis rotation vectors and their derivatives
        gtsam::Vector3 z_axis_j = Hoj.col(2).head<3>();
        gtsam::Vector3 dz_axis_j = dHoj.col(2).head<3>();

        // Extract the position vectors and their derivatives
        gtsam::Vector3 pos_o_i = Hoi.col(3).head<3>();
        gtsam::Vector3 dpos_o_i = dHoi.col(3).head<3>();
        gtsam::Vector3 pos_o_j = Hoj.col(3).head<3>();
        gtsam::Vector3 dpos_o_j = dHoj.col(3).head<3>();

        // Calculate the skew-symmetric matrices for the rotation vectors
        gtsam::Matrix3 rot_z_j = skewSymmetric(z_axis_j);
        gtsam::Matrix3 drot_z_j = skewSymmetric(dz_axis_j);

        // Calculate the derivative of the Jacobian column
        gtsam::Vector3 term1 = rot_z_j * (dpos_o_i - dpos_o_j);
        gtsam::Vector3 term2 = drot_z_j * (pos_o_i - pos_o_j);

        return term1 + term2;
    }

};

}   // namespace gp_planner

#endif //GP_PLANNER_ARMKINEMATICS_H