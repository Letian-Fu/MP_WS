#ifndef GP_PLANNER_OBSFACTOR_H
#define GP_PLANNER_OBSFACTOR_H

#pragma once

#include "headers.h"
#include "CostFunction.h"

namespace gp_planner{

class ObsFactor : public gtsam::NoiseModelFactor1<ArmModel::Pose>{
public:
    typedef ArmModel Robot;
    typedef Robot::Pose Pose;
private:
    typedef ObsFactor This;
    typedef gtsam::NoiseModelFactor1<Pose>  Base;
    // obstacle cost settings
    double epsilon_;  // distance from object that start non-zero cost

    // arm: planar one, all alpha = 0
    const Robot& robot_;

    // signed distance field from matlab
    const SDF& sdf_;

 public:
    /// shorthand for a smart pointer to a factor
    typedef std::shared_ptr<This> shared_ptr;

    /**
     * Constructor
     * @param cost_model cost function covariance, should to identity model
     * @param field      signed distance field
     * @param nn_index   nearest neighbour index of signed distance field
     */
    ObsFactor(gtsam::Key poseKey, const Robot& robot,
                        const SDF& sdf, double cost_sigma, double epsilon)
        : Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(),
                                                    cost_sigma),
        poseKey),
        epsilon_(epsilon),
        robot_(robot),
        sdf_(sdf) {}

    ~ObsFactor() override  {}

    /// error function
    /// numerical jacobians / analytic jacobians from cost function
    gtsam::Vector evaluateError(
        const typename Robot::Pose& conf,
        gtsam::OptionalMatrixType H1) const override{
            // cout<<"cost epsilon: "<<epsilon_<<endl;
            return ObsCost(conf , robot_, epsilon_, sdf_, H1);
        }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override{
        return std::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /** print contents */
    void print(const std::string& s = "",
                const gtsam::KeyFormatter& keyFormatter =
                    gtsam::DefaultKeyFormatter) const override{
        std::cout << s << "ObsFactor :" << std::endl;
        Base::print("", keyFormatter);
    }

    // /** Serialization function */
    // friend class boost::serialization::access;
    // template <class ARCHIVE>
    // void serialize(ARCHIVE& ar, const unsigned int version) {
    //     ar& boost::serialization::make_nvp(
    //         "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
    // }
};

class ObsGPFactor: public gtsam::NoiseModelFactor4<ArmModel::Pose, ArmModel::Velocity, ArmModel::Pose, ArmModel::Velocity>{
public:
    typedef ArmModel Robot;
    typedef Robot::Pose Pose;
    typedef Robot::Velocity Velocity;
private:
    typedef ObsGPFactor This;
    typedef gtsam::NoiseModelFactor4<Pose,Velocity,Pose,Velocity>   Base;
    // obstacle settings
    double epsilon_;  // global eps_ for hinge loss function

    // physical arm, with body sphere information
    const Robot& robot_;

    // signed distance field from matlab
    const SDF& sdf_;

    const gtsam::SharedNoiseModel& Qc_model_;
    double delta_t_;
    double tau_;
    gtsam::Matrix Qc_;
    gtsam::Matrix Lambda_;
    gtsam::Matrix Psi_;

public:
    /// shorthand for a smart pointer to a factor
    typedef std::shared_ptr<This> shared_ptr;

    /**
     * Constructor
     * @param cost_model cost function covariance, should to identity model
     * @param Qc_model   dim is equal to DOF
     * @param field      signed distance field
     * @param check_inter  how many points needed to be interpolated. 0 means no
     * GP interpolation
     */
    ObsGPFactor(gtsam::Key pose1Key, gtsam::Key vel1Key,
                        gtsam::Key pose2Key, gtsam::Key vel2Key,
                        const Robot& robot, const SDF& sdf,
                        double cost_sigma, double epsilon,
                        const gtsam::SharedNoiseModel& Qc_model, double delta_t,
                        double tau)
        :   Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(),
                                                cost_sigma),
            pose1Key, vel1Key, pose2Key, vel2Key),
            epsilon_(epsilon),
            robot_(robot),
            sdf_(sdf),
            Qc_model_(Qc_model),
            delta_t_(delta_t),
            tau_(tau){
                Qc_ = getQc(Qc_model_);
                Lambda_ = calcLambda(Qc_,delta_t_,tau_);
                Psi_ = calcPsi(Qc_, delta_t_, tau_);
            }
    ~ObsGPFactor() override {}

    /// error function
    /// numerical jacobians / analytic jacobians from cost function
    gtsam::Vector evaluateError(
        const typename Robot::Pose& conf1, const typename Robot::Velocity& vel1,
        const typename Robot::Pose& conf2, const typename Robot::Velocity& vel2,
        gtsam::OptionalMatrixType H1,
        gtsam::OptionalMatrixType H2,
        gtsam::OptionalMatrixType H3,
        gtsam::OptionalMatrixType H4) const override{
        return ObsCostGP(conf1, conf2, vel1, vel2,
                        robot_, epsilon_, sdf_, 
                        Qc_model_, delta_t_, tau_,
                        Qc_, Lambda_, Psi_,
                        H1, H2, H3, H4);
    }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override{
        return std::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /** print contents */
    void print(const std::string& s = "",
                const gtsam::KeyFormatter& keyFormatter =
                    gtsam::DefaultKeyFormatter) const override{
        std::cout << s << "ObsGPFactor :" << std::endl;
        Base::print("", keyFormatter);
    }

    // /** Serialization function */
    // friend class boost::serialization::access;
    // template <class ARCHIVE>
    // void serialize(ARCHIVE& ar, const unsigned int version) {
    //     ar& boost::serialization::make_nvp(
    //         "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
    // }
};


}   //namespace

#endif  //GP_PLANNER_OBSFACTOR_H