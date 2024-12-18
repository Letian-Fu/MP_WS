#include "OptimizerSetting.h"

namespace gp_planner{

/* ************************************************************************** */
OptimizerSetting::OptimizerSetting()
    :   dof(0),
        total_step(10),
        total_time(1.0),
        flag_pos_limit(false),
        flag_vel_limit(false),
        epsilon(0.2),
        cost_sigma(0.1),
        obs_check_inter(5),
        opt_type(GaussNewton),
        opt_verbosity(None),
        final_iter_no_increase(true),
        rel_thresh(1e-2),
        max_iter(50) {}

/* ************************************************************************** */
OptimizerSetting::OptimizerSetting(size_t system_dof)
    :   dof(system_dof),
        total_step(10),
        total_time(1.0),
        flag_pos_limit(false),
        flag_vel_limit(false),
        joint_pos_limits_up(1e6 * gtsam::Vector::Ones(system_dof)),
        joint_pos_limits_down(-1e6 * gtsam::Vector::Ones(system_dof)),
        vel_limits(1e6 * gtsam::Vector::Ones(system_dof)),
        pos_limit_thresh(0.001 * gtsam::Vector::Ones(system_dof)),
        vel_limit_thresh(0.001 * gtsam::Vector::Ones(system_dof)),
        epsilon(0.2),
        cost_sigma(0.1),
        obs_check_inter(5),
        opt_type(GaussNewton),
        opt_verbosity(None),
        final_iter_no_increase(true),
        rel_thresh(1e-2),
        max_iter(50),
        conf_prior_model(gtsam::noiseModel::Isotropic::Sigma(system_dof, 0.0001)),
        vel_prior_model(gtsam::noiseModel::Isotropic::Sigma(system_dof, 0.0001)),
        pos_limit_model(gtsam::noiseModel::Isotropic::Sigma(system_dof, 0.001)),
        vel_limit_model(gtsam::noiseModel::Isotropic::Sigma(system_dof, 0.001)),
        Qc_model(gtsam::noiseModel::Unit::Create(system_dof)) {}
      

/* ************************************************************************** */
void OptimizerSetting::set_pos_limit_model(const gtsam::Vector& v) {
    pos_limit_model = gtsam::noiseModel::Diagonal::Sigmas(v);
}

/* ************************************************************************** */
void OptimizerSetting::set_vel_limit_model(const gtsam::Vector& v) {
    vel_limit_model = gtsam::noiseModel::Diagonal::Sigmas(v);
}

/* ************************************************************************** */
void OptimizerSetting::set_conf_prior_model(double sigma) {
    conf_prior_model = gtsam::noiseModel::Isotropic::Sigma(dof, sigma);
}

/* ************************************************************************** */
void OptimizerSetting::set_vel_prior_model(double sigma) {
    vel_prior_model = gtsam::noiseModel::Isotropic::Sigma(dof, sigma);
}

/* ************************************************************************** */
void OptimizerSetting::set_Qc_model(const gtsam::Matrix& Qc) {
    Qc_model = gtsam::noiseModel::Gaussian::Covariance(Qc);
}

}   //namespace