#ifndef GP_PLANNER_GAUSSIANFACTOR_H
#define GP_PLANNER_GAUSSIANFACTOR_H

#pragma once

#include "headers.h"
#include "CostFunction.h"

namespace gp_planner{

class GPFactor : public gtsam::NoiseModelFactor4<gtsam::Vector, gtsam::Vector,
                                                gtsam::Vector, gtsam::Vector>{
private:
    size_t dof_;
    double delta_t_;

    typedef GPFactor This;
    typedef gtsam::NoiseModelFactor4<gtsam::Vector, gtsam::Vector, 
                                    gtsam::Vector, gtsam::Vector> Base;

public:
    GPFactor() {
    } /* Default constructor only for serialization */

    /// Constructor
    /// @param delta_t is the time between the two states
    GPFactor(gtsam::Key poseKey1, gtsam::Key velKey1,
            gtsam::Key poseKey2, gtsam::Key velKey2,
            double delta_t,
            const gtsam::SharedNoiseModel Qc_model)
        : Base(gtsam::noiseModel::Gaussian::Covariance(
                    CalcQ(getQc(Qc_model), delta_t)),
                poseKey1, velKey1, poseKey2, velKey2),
            dof_(Qc_model->dim()),
            delta_t_(delta_t) {}

    ~GPFactor() override {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override{
        return std::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /// factor error function
    gtsam::Vector evaluateError(
        const gtsam::Vector& pose1, const gtsam::Vector& vel1,
        const gtsam::Vector& pose2, const gtsam::Vector& vel2,
        gtsam::OptionalMatrixType H1,
        gtsam::OptionalMatrixType H2,
        gtsam::OptionalMatrixType H3,
        gtsam::OptionalMatrixType H4) const override{

        // gtsam::Vector error = GPCost(pose1, vel1, pose2, vel2, delta_t_, 
        //             H1, H2, H3, H4);
        // cout<<"GPCost: "<<error.transpose()<<endl;
        return GPCost(pose1, vel1, pose2, vel2, delta_t_, 
                    H1, H2, H3, H4);
    }

    /** demensions */
    size_t dim() const { return dof_; }

    /** number of variables attached to this factor */
    size_t size() const { return 4; }

    /** equals specialized to this factor */
    bool equals(const gtsam::NonlinearFactor& expected,
                        double tol = 1e-9) const override{
        const This* e = dynamic_cast<const This*>(&expected);
        return e != nullptr && Base::equals(*e, tol) &&
            fabs(this->delta_t_ - e->delta_t_) < tol;
    }

    /** print contents */
    void print(const std::string& s = "",
                const gtsam::KeyFormatter& keyFormatter =
                    gtsam::DefaultKeyFormatter) const override{
        std::cout << s << "4-way Gaussian Process Factor Linear(" << dof_ << ")"
                << std::endl;
        Base::print("", keyFormatter);
    }

private:
    // /** Serialization function */
    // friend class boost::serialization::access;
    // template <class ARCHIVE>
    // void serialize(ARCHIVE& ar, const unsigned int version) {
    //     ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    //     ar& BOOST_SERIALIZATION_NVP(dof_);
    //     ar& BOOST_SERIALIZATION_NVP(delta_t_);
    // }

};

}   //namespace

#endif  //GP_PLANNER_GAUSSIANFACTOR_H