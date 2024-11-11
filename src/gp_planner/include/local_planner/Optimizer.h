#ifndef GP_PLANNER_OPTIMIZER_H
#define GP_PLANNER_OPTIMIZER_H

#pragma once

#include "headers.h"
#include "GPInterpolate.h"
#include "PriorFactor.h"
#include "GaussianFactor.h"
#include "ObsFactor.h"
#include "LimitFactor.h"
#include "OptimizerSetting.h"
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

namespace gp_planner{

gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph,
                    const gtsam::Values& init_values,
                    const OptimizerSetting& setting,
                    bool iter_no_increase = true);

/* ************************************************************************** */
gtsam::Values initArmTrajStraightLine(
    const gtsam::Vector& init_conf, const gtsam::Vector& end_conf, size_t total_step);

gtsam::Values Optimizer(const ArmModel& arm, const SDF& sdf,
                        const gtsam::Vector& start_conf, const gtsam::Vector& end_conf,
                        const gtsam::Vector& start_vel, const gtsam::Vector& end_vel,
                        const gtsam::Values& init_values,
                        const OptimizerSetting& setting){
    const double delta_t = setting.total_time / static_cast<double>(setting.total_step);
    const double inter_dt = delta_t / static_cast<double>(setting.obs_check_inter + 1);

    //build graph
    gtsam::NonlinearFactorGraph graph;
    for(size_t i=0;i<=setting.total_step;i++){
        gtsam::Key pose_key = gtsam::Symbol('x',i);
        gtsam::Key vel_key = gtsam::Symbol('v',i);
        // start and end
        if(i == 0){
            graph.add(PriorFactorConf(pose_key,start_conf,setting.conf_prior_model));
            graph.add(PriorFactorVel(vel_key,start_vel,setting.vel_prior_model));
        } else if(i == setting.total_step){
            graph.add(PriorFactorConf(pose_key,end_conf,setting.conf_prior_model));
            graph.add(PriorFactorVel(vel_key,end_vel,setting.vel_prior_model));
        }
        // Limits for joints and velocity
        if(setting.flag_pos_limit){
            graph.add(JointLimitFactor(pose_key,setting.pos_limit_model,setting.joint_pos_limits_down,setting.joint_pos_limits_up,setting.pos_limit_thresh));
        }
        if(setting.flag_vel_limit){
            graph.add(VelLimitFactor(vel_key,setting.vel_limit_model,setting.vel_limits,setting.vel_limit_thresh));
        }
        // non-interpolated cost factor
        graph.add(ObsFactor(pose_key, arm, sdf, setting.cost_sigma, setting.epsilon));
        
        if(i>0){
            gtsam::Key last_pose_key = gtsam::Symbol('x', i - 1);
            gtsam::Key last_vel_key = gtsam::Symbol('v', i - 1);
            // GP factor
            graph.add(GPFactor(last_pose_key,last_vel_key,pose_key,vel_key,delta_t,setting.Qc_model));
            // interpolated cost factor
            if(setting.obs_check_inter > 0){
                for(size_t j = 1; j<=setting.obs_check_inter; j++){
                    const double tau = inter_dt * static_cast<double>(j);
                    graph.add(ObsGPFactor(last_pose_key, last_vel_key, pose_key, vel_key,
                                        arm, sdf, setting.cost_sigma, setting.epsilon, setting.Qc_model, delta_t, tau));
                }
            }
        }
    }
    return optimize(graph, init_values, setting);
}

double CollisionCost(const ArmModel& arm, const SDF& sdf,
                    const gtsam::Values& result,
                    const OptimizerSetting& setting){
    double coll_cost = 0;
    ObsFactor obs_factor = ObsFactor(gtsam::Symbol('x', 0), arm, sdf, setting.cost_sigma, 0);
    for(size_t i=0; i<result.size()/2;i++){
        coll_cost += (obs_factor.evaluateError(result.at<gtsam::Vector>(gtsam::Symbol('x',i)),nullptr)).sum();
    }
    return coll_cost;
}

gtsam::Values interpolateTraj(const gtsam::Values& opt_values,
                            const gtsam::SharedNoiseModel Qc_model,
                            double delta_t, size_t inter_step){
    double inter_dt = delta_t / static_cast<double>(inter_step + 1);
    size_t last_pos_idx;
    size_t inter_pos_count = 0;
    // results
    gtsam::Values results;

    // TODO: gtsam keyvector has issue: free invalid pointer
    gtsam::KeyVector key_vec = opt_values.keys();

    gtsam::Matrix Qc;
    gtsam::Matrix Lambda;
    gtsam::Matrix Psi;

    Qc = getQc(Qc_model);

    // sort key list
    std::sort(key_vec.begin(), key_vec.end());

    for(size_t i=0;i<key_vec.size();i++){
        gtsam::Key key = key_vec[i];
        if(gtsam::Symbol(key).chr()=='x'){
            size_t pos_idx = gtsam::Symbol(key).index();
            if (pos_idx != 0) {
                // skip first pos to interpolate
                for (size_t inter_idx = 1; inter_idx <= inter_step + 1; inter_idx++) {
                    if (inter_idx == inter_step + 1) {
                        // last pose
                        results.insert(gtsam::Symbol('x', inter_pos_count),
                                    opt_values.at<gtsam::Vector>(gtsam::Symbol('x', pos_idx)));
                        results.insert(gtsam::Symbol('v', inter_pos_count),
                                    opt_values.at<gtsam::Vector>(gtsam::Symbol('v', pos_idx)));

                    } else {
                        // inter pose
                        double tau = static_cast<double>(inter_idx) * inter_dt;
                        Lambda = calcLambda(Qc,delta_t,tau);
                        Psi = calcPsi(Qc, delta_t, tau);
                        gtsam::Vector conf1 = opt_values.at<gtsam::Vector>(gtsam::Symbol('x', last_pos_idx));
                        gtsam::Vector vel1 = opt_values.at<gtsam::Vector>(gtsam::Symbol('v', last_pos_idx));
                        gtsam::Vector conf2 = opt_values.at<gtsam::Vector>(gtsam::Symbol('x', pos_idx));
                        gtsam::Vector vel2 = opt_values.at<gtsam::Vector>(gtsam::Symbol('v', pos_idx));
                        gtsam::Vector conf = interpolatePose(conf1, vel1, conf2, vel2, Lambda, Psi);
                        gtsam::Vector vel = interpolateVelocity(conf1, vel1, conf2, vel2, Lambda, Psi);
                        results.insert(gtsam::Symbol('x', inter_pos_count), conf);
                        results.insert(gtsam::Symbol('v', inter_pos_count), vel);
                    }
                    inter_pos_count++;
                }   

            } else {
                // cache first pose
                results.insert(gtsam::Symbol('x', 0), opt_values.at<gtsam::Vector>(gtsam::Symbol('x', 0)));
                results.insert(gtsam::Symbol('v', 0), opt_values.at<gtsam::Vector>(gtsam::Symbol('v', 0)));
                inter_pos_count++;
            }
            last_pos_idx = pos_idx;
        }
    }
    return results;
}

}   //namespace

#endif //GP_PLANNER_OPTIMIZER_H
