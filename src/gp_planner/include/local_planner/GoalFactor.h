#ifndef GP_PLANNER_GOALFACTOR_H
#define GP_PLANNER_GOALFACTOR_H

#pragma once

#include "headers.h"
#include "CostFunction.h"

namespace gp_planner{
class GoalFactor : public gtsam::NoiseModelFactor1<gtsam::Vector>{
private:
    typedef GoalFactor This;
    typedef gtsam::NoiseModelFactor1<gtsam::Vector> Base;

    Arm arm_;
    gtsam::Point3 dest_point_;

public:
    typedef std::shared_ptr<This> shared_ptr;
    GoalFactor(gtsam::Key poseKey, const gtsam::SharedNoiseModel& cost_model,
                const Arm& arm, const gtsam::Point3& dest_point)
    : Base(cost_model, poseKey), arm_(arm), dest_point_(dest_point){}
    ~GoalFactor() override {}
    gtsam::Vector evaluateError(const gtsam::Vector& conf, gtsam::OptionalMatrixType H1) const override {
        // fk
        std::vector<gtsam::Pose3> joint_pos;
        std::vector<gtsam::Matrix> J_jpx_jp;
        arm_.forwardKinematics(conf, boost::none, joint_pos, boost::none, J_jpx_jp);

        if (H1) {
            gtsam::Matrix36 Hpp;
            gtsam::Point3 end_point = joint_pos[arm_.dof() - 1].translation(Hpp);
            *H1 = Hpp * J_jpx_jp[arm_.dof() - 1];
            return end_point - dest_point_;
        } else {
            return joint_pos[arm_.dof() - 1].translation() - dest_point_;
        }
    }

    gtsam::NonlinearFactor::shared_ptr clone() const override {
        return std::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
        std::cout << s << "GoalFactorArm :" << std::endl;
        Base::print("", keyFormatter);
        std::cout << "dest : " << dest_point_.transpose() << std::endl;
    }

};

}   //namespace


#endif  //