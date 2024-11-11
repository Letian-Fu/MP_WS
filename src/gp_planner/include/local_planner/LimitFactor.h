#ifndef GP_PLANNER_LIMITFACTOR_H
#define GP_PLANNER_LIMITFACTOR_H

#pragma once

#include "headers.h"
#include "CostFunction.h"

namespace gp_planner{

class JointLimitFactor : public gtsam::NoiseModelFactor1<gtsam::Vector>{
private:
    typedef JointLimitFactor This;
    typedef gtsam::NoiseModelFactor1<gtsam::Vector> Base;

    // joint limit value
    gtsam::Vector down_limit_, up_limit_;

    // hinge loss threshold
    gtsam::Vector limit_thresh_;

public:
    /// shorthand for a smart pointer to a factor
    typedef std::shared_ptr<This> shared_ptr;

    /**
     * Constructor
     * @param cost_model joint limit cost weight
     * @param limit_thresh hinge loss threshold
     */
    JointLimitFactor(gtsam::Key poseKey,
                    const gtsam::SharedNoiseModel& cost_model,
                    const gtsam::Vector& down_limit,
                    const gtsam::Vector& up_limit,
                    const gtsam::Vector& limit_thresh)
        : Base(cost_model, poseKey),
            down_limit_(down_limit),
            up_limit_(up_limit),
            limit_thresh_(limit_thresh) {
        // check dimensions
        if ((size_t)down_limit.size() != cost_model->dim() ||
            (size_t)up_limit.size() != cost_model->dim() ||
            (size_t)limit_thresh.size() != cost_model->dim())
        throw std::runtime_error(
            "[JointLimitFactor] ERROR: limit vector dim does not fit.");
    }

    ~JointLimitFactor() override {}

    /// error function
    gtsam::Vector evaluateError(
        const gtsam::Vector& conf,
        gtsam::OptionalMatrixType H1) const override{
        return LimitCostConf(conf, down_limit_, up_limit_, limit_thresh_, H1);
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
        std::cout << s << "JointLimitFactor :" << std::endl;
        Base::print("", keyFormatter);
        std::cout << "Limit cost threshold : " << limit_thresh_ << std::endl;
    }

private:
    // /** Serialization function */
    // friend class boost::serialization::access;
    // template <class ARCHIVE>
    // void serialize(ARCHIVE& ar, const unsigned int version) {
    // ar& boost::serialization::make_nvp(
    //     "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
    // }
};

class VelLimitFactor : public gtsam::NoiseModelFactor1<gtsam::Vector>{
private:
    // typedefs
    typedef VelLimitFactor This;
    typedef gtsam::NoiseModelFactor1<gtsam::Vector> Base;

    // joint velocity limit value for each joint
    gtsam::Vector vel_limit_;

    // hinge loss threshold
    gtsam::Vector limit_thresh_;

    public:
    /// shorthand for a smart pointer to a factor
    typedef std::shared_ptr<This> shared_ptr;

    /**
     * Constructor
     * @param cost_model joint limit cost weight
     * @param limit_thresh hinge loss threshold
     */
    VelLimitFactor(gtsam::Key poseKey,
                                const gtsam::SharedNoiseModel& cost_model,
                                const gtsam::Vector& vel_limit,
                                const gtsam::Vector& limit_thresh)
        : Base(cost_model, poseKey),
            vel_limit_(vel_limit),
            limit_thresh_(limit_thresh) {
        // check dimensions
        if ((size_t)vel_limit.size() != cost_model->dim() ||
            (size_t)limit_thresh.size() != cost_model->dim())
        throw std::runtime_error(
            "[VelLimitFactor] ERROR: limit vector dim does not fit.");
        // velocity limit should > 0
        if (vel_limit.minCoeff() <= 0.0)
        throw std::runtime_error(
            "[VelLimitFactor] ERROR: velocity limit <= 0.");
    }

    ~VelLimitFactor() override {}

    /// error function
    gtsam::Vector evaluateError(
        const gtsam::Vector& vel,
        gtsam::OptionalMatrixType H1) const override{
        return LimitCostVel(vel, vel_limit_, limit_thresh_, H1);
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
        std::cout << s << "VelLimitFactor :" << std::endl;
        Base::print("", keyFormatter);
        std::cout << "Limit cost threshold : " << limit_thresh_ << std::endl;
    }

private:
    /** Serialization function */
    // friend class boost::serialization::access;
    // template <class ARCHIVE>
    // void serialize(ARCHIVE& ar, const unsigned int version) {
    //     ar& boost::serialization::make_nvp(
    //         "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
    // }

};

}   //namespace

#endif  //GP_PLANNER_LIMITFACTOR_H