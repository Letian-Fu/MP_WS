#ifndef GP_PLANNER_PRIORFACTOR_H
#define GP_PLANNER_PRIORFACTOR_H

#pragma once

#include "headers.h"
#include "CostFunction.h"

namespace gp_planner{
class PriorFactorConf: public gtsam::NoiseModelFactorN<gtsam::Vector>{
private:   
    using Base = gtsam::NoiseModelFactorN<gtsam::Vector>;
    gtsam::Vector prior_;
public:
    typedef PriorFactorConf This;
    typedef std::shared_ptr<This> shared_ptr;


    PriorFactorConf() {}

    PriorFactorConf(gtsam::Key key, const gtsam::Vector& prior, const gtsam::SharedNoiseModel& model = nullptr):
        Base(model, key), prior_(prior){}

    PriorFactorConf(gtsam::Key key, const gtsam::Vector& prior, const gtsam::Matrix& covariance):
        Base(gtsam::noiseModel::Gaussian::Covariance(covariance), key), prior_(prior){}

    ~PriorFactorConf() override {} 

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return std::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** print */
    void print(const std::string& s,
       const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "PriorFactorConf on " << keyFormatter(this->key()) << "\n";
      std::cout << "prior value: "<< prior_.transpose() << std::endl;
      if (this->noiseModel_)
        this->noiseModel_->print("  noise model: ");
      else
        std::cout << "no noise model" << std::endl;
    }

    // /** equals */
    bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
      const This* e = dynamic_cast<const This*> (&expected);
      bool flag = true;
      for(size_t i=0;i<prior_.size();i++){
        if(abs(prior_[i]-e->prior_[i])>tol){
            flag = false;
            break;
        }
      }
      return e != nullptr && Base::equals(*e, tol) && flag;
    }

    gtsam::Vector evaluateError(const gtsam::Vector& x, gtsam::OptionalMatrixType H) const override{
      // gtsam::Vector error = PriorCostConf(x, prior_, H);
      // cout<<"PriorConfError: "<<error.transpose()<<endl;
      return PriorCostConf(x, prior_, H);
    }

    /** Serialization function */
    // friend class boost::serialization::access;
    // template<class ARCHIVE>
    // void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    //   // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    //   ar & boost::serialization::make_nvp("NoiseModelFactor1",
    //       boost::serialization::base_object<Base>(*this));
    //   ar & BOOST_SERIALIZATION_NVP(prior_);
    // }
};



class PriorFactorVel: public gtsam::NoiseModelFactorN<gtsam::Vector>{
private:   
    typedef gtsam::NoiseModelFactorN<gtsam::Vector> Base;
    gtsam::Vector prior_;
public:
    typedef PriorFactorVel This;
    typedef std::shared_ptr<This> shared_ptr;

    PriorFactorVel() {}

    PriorFactorVel(gtsam::Key key, const gtsam::Vector& prior, const gtsam::SharedNoiseModel& model = nullptr):
        Base(model, key), prior_(prior){}

    PriorFactorVel(gtsam::Key key, const gtsam::Vector& prior, const gtsam::Matrix& covariance):
        Base(gtsam::noiseModel::Gaussian::Covariance(covariance), key), prior_(prior){}

    ~PriorFactorVel() override {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override {
      return std::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** print */
    void print(const std::string& s,
       const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override {
      std::cout << s << "PriorFactorVel on " << keyFormatter(this->key()) << "\n";
      std::cout << "prior value: "<< prior_.transpose() << std::endl;
      if (this->noiseModel_)
        this->noiseModel_->print("  noise model: ");
      else
        std::cout << "no noise model" << std::endl;
    }

    // /** equals */
    bool equals(const NonlinearFactor& expected, double tol=1e-9) const override {
      const This* e = dynamic_cast<const This*> (&expected);
      bool flag = true;
      for(size_t i=0;i<prior_.size();i++){
        if(abs(prior_[i]-e->prior_[i])>tol){
            flag = false;
            break;
        }
      }
      return e != nullptr && Base::equals(*e, tol) && flag;
    }

    gtsam::Vector evaluateError(const gtsam::Vector& x, gtsam::OptionalMatrixType H) const override{
      // gtsam::Vector error = PriorCostVel(x, prior_, H);
      // cout<<"PriorCostVel: "<<error.transpose()<<endl;
      return PriorCostVel(x, prior_, H);
    }

    /** Serialization function */
    // friend class boost::serialization::access;
    // template<class ARCHIVE>
    // void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    //   // NoiseModelFactor1 instead of NoiseModelFactorN for backward compatibility
    //   ar & boost::serialization::make_nvp("NoiseModelFactor1",
    //       boost::serialization::base_object<Base>(*this));
    //   ar & BOOST_SERIALIZATION_NVP(prior_);
    // }
};

}   //namespace


#endif 