#ifndef GP_PLANNER_COSTFUNCTION_H
#define GP_PLANNER_COSTFUNCTION_H

#pragma once

#include "headers.h"
#include "ArmModel.h"
#include "GPInterpolate.h"
#include "SDF.h"

namespace gp_planner{

gtsam::Vector PriorCostConf(const gtsam::Vector& conf, const gtsam::Vector& prior,gtsam::OptionalMatrixType H){
    if(H) (*H) = gtsam::Matrix::Identity(conf.size(),conf.size());
    return conf-prior;
}

gtsam::Vector PriorCostVel(const gtsam::Vector& vel, const gtsam::Vector& prior,gtsam::OptionalMatrixType H){
    if(H) (*H) = gtsam::Matrix::Identity(vel.size(),vel.size());
    return vel-prior;
}

inline double hingeLossLimitCost(
    double p, double down_limit, double up_limit, double thresh,
    boost::optional<double&> H_p = boost::none) {
  if (p < down_limit + thresh) {
    if (H_p) *H_p = -1.0;
    return down_limit + thresh - p;

  } else if (p <= up_limit - thresh) {
    if (H_p) *H_p = 0.0;
    return 0.0;

  } else {
    if (H_p) *H_p = 1.0;
    return p - up_limit + thresh;
  }
}

gtsam::Vector LimitCostConf(const gtsam::Vector& conf, 
                            const gtsam::Vector& down_limit, const gtsam::Vector& up_limit, const gtsam::Vector& limit_thresh, 
                            gtsam::OptionalMatrixType H1){
    if (H1) *H1 = gtsam::Matrix::Zero(conf.size(), conf.size());
    gtsam::Vector err(conf.size());
    for (size_t i = 0; i < (size_t)conf.size(); i++) {
        if (H1) {
            double Hp;
            err(i) = hingeLossLimitCost(conf(i), down_limit(i), up_limit(i),
                                            limit_thresh(i), Hp);
            (*H1)(i, i) = Hp;
        } else {
            err(i) = hingeLossLimitCost(conf(i), down_limit(i), up_limit(i),
                                            limit_thresh(i));
        }
    }
    return err;
}

gtsam::Vector LimitCostVel(const gtsam::Vector& vel, 
                            const gtsam::Vector& vel_limit, const gtsam::Vector& limit_thresh, 
                            gtsam::OptionalMatrixType H1){
    if (H1) *H1 = gtsam::Matrix::Zero(vel.size(), vel.size());
    gtsam::Vector err(vel.size());
    for (size_t i = 0; i < (size_t)vel.size(); i++) {
    if (H1) {
        double Hp;
        err(i) = hingeLossLimitCost(vel(i), -vel_limit(i), vel_limit(i),
                                        limit_thresh(i), Hp);
        (*H1)(i, i) = Hp;
    } else {
        err(i) = hingeLossLimitCost(vel(i), -vel_limit(i), vel_limit(i),
                                        limit_thresh(i));
    }
    }
    return err;
}

// hinge loss obstacle cost function
// inline double hingeLossObstacleCost(
//     const gtsam::Point3& point, const SDF& sdf, double eps,
//     gtsam::OptionalJacobian<1, 3> H_point = std::nullopt) {
//     gtsam::Vector3 field_gradient;
//     double dist_signed;
//     try {
//         dist_signed = sdf.getSignedDistance(point, field_gradient);
//         // cout<<"dist_signed: "<<dist_signed<<endl;
//     } catch (SDFQueryOutOfRange&) {
//         // std::cout << "[hingeLossObstacleCost] WARNING: querying signed distance
//         // out of range, "
//         //    "assume zero obstacle cost." << std::endl;
//         if (H_point) *H_point = gtsam::Matrix13::Zero();
//         return 0.0;
//     }

//     if (dist_signed > eps) {
//         // faraway no error
//         if (H_point) *H_point = gtsam::Matrix13::Zero();
//         return 0.0;

//     } else {
//         // outside but < eps or inside object
//         if (H_point) *H_point = -field_gradient.transpose();
//         return eps - dist_signed;
//     }
// }

inline double hingeLossObstacleCost(
    const gtsam::Point3& point, const SDF& sdf, double eps,
    gtsam::OptionalJacobian<1, 3> H_point = std::nullopt) {
    gtsam::Vector3 field_gradient;
    double dist_signed;
    try {
        dist_signed = sdf.getSignedDistance(point, field_gradient);
        // cout<<"dist_signed: "<<dist_signed<<endl;
    } catch (SDFQueryOutOfRange&) {
        // std::cout << "[hingeLossObstacleCost] WARNING: querying signed distance
        // out of range, "
        //    "assume zero obstacle cost." << std::endl;
        if (H_point) *H_point = gtsam::Matrix13::Zero();
        return 0.0;
    }

    if (dist_signed > eps) {
        // faraway no error
        if (H_point) *H_point = gtsam::Matrix13::Zero();
        return 0.0;

    } else if (dist_signed >= 0.0) {
        // 情况 2: 0 ≤ d(c) ≤ ε, 损失为 (1 / 2ε) * (d(c) - ε)^2
        double diff = dist_signed - eps;
        double cost = (0.5 / eps) * diff * diff;
        if (H_point) {
            // 雅可比: ∂L/∂point = (1 / ε) * (d(c) - ε) * (-∇d(c))
            *H_point = (1.0 / eps) * diff * (-field_gradient.transpose());
        }
        return cost;
    } else {
        // 情况 3: d(c) < 0, 损失为 (1 / 2) * ε - d(c)
        double cost = (0.5 * eps) - dist_signed;
        if (H_point) {
            // 雅可比: ∂L/∂point = -(-∇d(c)) = ∇d(c)
            *H_point = -field_gradient.transpose();
        }
        return cost;
    }
}

gtsam::Vector ObsCost(const gtsam::Vector& conf, 
                        const ArmModel& arm, 
                        double epsilon,
                        const SDF& sdf,
                        gtsam::OptionalMatrixType H1){
    // if Jacobians used, initialize as zeros
    // size: arm_nr_points_ * DOF
    // cout<<"cost epsilon: "<< epsilon <<endl;
    if (H1) *H1 = gtsam::Matrix::Zero(arm.nr_body_spheres(), arm.dof());
    // run forward kinematics of this configuration
    std::vector<gtsam::Point3> sph_centers;
    vector<gtsam::Matrix> J_px_jp;
    if (H1)
        arm.sphereCenters(conf, sph_centers, J_px_jp);
    else
        arm.sphereCenters(conf, sph_centers);
    // allocate cost vector
    gtsam::Vector err(arm.nr_body_spheres());
    // for each point on arm stick, get error
    for (size_t sph_idx = 0; sph_idx < arm.nr_body_spheres(); sph_idx++) {
        const double total_eps = arm.sphere_radius(sph_idx) + epsilon;
        // cout<<"total_eps: "<<total_eps<<"  "<<epsilon<<endl;
        if (H1) {
            gtsam::Matrix13 Jerr_point;
            err(sph_idx) = hingeLossObstacleCost(sph_centers[sph_idx], sdf,
                                                total_eps, Jerr_point);
            // chain rules
            H1->row(sph_idx) = Jerr_point * J_px_jp[sph_idx];
        } else {
            err(sph_idx) =
                hingeLossObstacleCost(sph_centers[sph_idx], sdf, total_eps);
            // if(err(sph_idx)>0)  cout<<"hingeLoss: "<<sph_idx<<"  "<<sph_centers[sph_idx].transpose()<<"  "<<arm.sphere_radius(sph_idx)<<"  "<<err(sph_idx)<<endl;
        }
    }
    return err;
}

gtsam::Vector ObsCostGP(const gtsam::Vector& conf1, const gtsam::Vector& conf2,
                        const gtsam::Vector& vel1, const gtsam::Vector& vel2,
                        const ArmModel& arm, double epsilon, const SDF& sdf,
                        const gtsam::SharedNoiseModel Qc_model, double delta_t, double tau,
                        const gtsam::Matrix& Qc, const gtsam::Matrix& Lambda, const gtsam::Matrix& Psi,
                        gtsam::OptionalMatrixType H1,
                        gtsam::OptionalMatrixType H2,
                        gtsam::OptionalMatrixType H3,
                        gtsam::OptionalMatrixType H4){
    const bool use_H = (H1 || H2 || H3 || H4);
    // if Jacobians used, initialize Jerr_conf as zeros
    // size: arm_nr_points_ * DOF
    gtsam::Matrix Jerr_conf = gtsam::Matrix::Zero(arm.nr_body_spheres(), arm.dof());
    // get conf by interpolation, except last pose
    gtsam::Vector conf;
    gtsam::Matrix Jconf_c1, Jconf_c2, Jconf_v1, Jconf_v2;
    if (use_H)
        conf = interpolatePose(conf1, vel1, conf2, vel2, Lambda, Psi, 
                                Jconf_c1, Jconf_v1, Jconf_c2, Jconf_v2);
    else
        conf = interpolatePose(conf1, vel1, conf2, vel2, Lambda, Psi);

    // run forward kinematics of this configuration
    vector<gtsam::Point3> sph_centers;
    vector<gtsam::Matrix> J_px_jp;
    if (H1)
        arm.sphereCenters(conf, sph_centers, J_px_jp);
    else
        arm.sphereCenters(conf, sph_centers);

    // allocate cost vector
    gtsam::Vector err(arm.nr_body_spheres());

    // for each point on arm stick, get error
    for (size_t sph_idx = 0; sph_idx < arm.nr_body_spheres(); sph_idx++) {
        const double total_eps = arm.sphere_radius(sph_idx) + epsilon;

        if (H1) {
            gtsam::Matrix13 Jerr_point;
            err(sph_idx) = hingeLossObstacleCost(sph_centers[sph_idx], sdf,
                                                total_eps, Jerr_point);

            // chain rules
            Jerr_conf.row(sph_idx) = Jerr_point * J_px_jp[sph_idx];

        } else {
            err(sph_idx) =
                hingeLossObstacleCost(sph_centers[sph_idx], sdf, total_eps);
        }
    }

    // update jacobians
    if (use_H)
        updatePoseJacobians(Jerr_conf, Jconf_c1, Jconf_v1, Jconf_c2,
                                    Jconf_v2, H1, H2, H3, H4);

    return err;
}

gtsam::Vector GPCost(const gtsam::Vector& pose1, const gtsam::Vector& vel1,
                    const gtsam::Vector& pose2, const gtsam::Vector& vel2,
                    double delta_t,
                    gtsam::OptionalMatrixType H1,
                    gtsam::OptionalMatrixType H2,
                    gtsam::OptionalMatrixType H3,
                    gtsam::OptionalMatrixType H4){
    size_t dof = pose1.size();
    // state vector
    gtsam::Vector x1 = (gtsam::Vector(2 * dof) << pose1, vel1).finished();
    gtsam::Vector x2 = (gtsam::Vector(2 * dof) << pose2, vel2).finished();

    // Jacobians
    if (H1)
      *H1 =
          (gtsam::Matrix(2 * dof, dof) << gtsam::Matrix::Identity(dof, dof),
           gtsam::Matrix::Zero(dof, dof))
              .finished();
    if (H2)
      *H2 = (gtsam::Matrix(2 * dof, dof)
                 << delta_t * gtsam::Matrix::Identity(dof, dof),
             gtsam::Matrix::Identity(dof, dof))
                .finished();
    if (H3)
      *H3 = (gtsam::Matrix(2 * dof, dof)
                 << -1.0 * gtsam::Matrix::Identity(dof, dof),
             gtsam::Matrix::Zero(dof, dof))
                .finished();
    if (H4)
      *H4 = (gtsam::Matrix(2 * dof, dof) << gtsam::Matrix::Zero(dof, dof),
             -1.0 * gtsam::Matrix::Identity(dof, dof))
                .finished();

    // transition matrix & error
    return calcPhi(dof, delta_t) * x1 - x2;
}

}   //namespace gp_planner

#endif //GP_PLANNER_COSTFUNCTION_H