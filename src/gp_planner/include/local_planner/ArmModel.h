#ifndef GP_PLANNER_ARMMODEL_H
#define GP_PLANNER_ARMMODEL_H

#pragma once

#include "headers.h"
#include "ArmKinematics.h"
namespace gp_planner{

/// body sphere class, each one indicate a body sphere
struct BodySphere {
  size_t link_id;        // attched link id, 0 - nr_link-1
  double radius;         // sphere radius
  gtsam::Point3 center;  // sphere center position to the link base
  // constructor
  BodySphere(size_t id, double r, const gtsam::Point3& c)
      : link_id(id), radius(r), center(c) {}
  ~BodySphere() = default;
};

/// vector of body sphere, typedef here to wrap in matlab
using BodySphereVector = std::vector<BodySphere>;
/**
 * Abstract robot model, provide sphere-based robot model.
 * template parameter is forward kinematics type
 */
class ArmModel{
public:
    typedef Arm FKModel;
    typedef typename FKModel::Pose Pose;
    typedef typename FKModel::Velocity Velocity;

private:
    FKModel fk_model_;               // fk
    BodySphereVector body_spheres_;  // body spheres
public:
    /// default contructor for serialization
    ArmModel() {}

    /// Contructor take a fk model
    ArmModel(const FKModel& fk_model, const BodySphereVector& body_spheres)
        : fk_model_(fk_model), body_spheres_(body_spheres) {}

    /// Default destructor
    ~ArmModel();

    /// given pose in configuration space, solve sphere center vector in work
    /// space with optional associated jacobian of pose
    void sphereCenters(const Pose& jp, std::vector<gtsam::Point3>& sph_centers,
                        boost::optional<std::vector<gtsam::Matrix>&> J_point_conf =
                            boost::none) const;

    /// given pose in configuration space, solve a single sphere center in work
    /// space for fast call of a single sphere with optional associated jacobian
    /// of pose
    gtsam::Point3 sphereCenter(
        size_t sph_idx, const Pose& jp,
        boost::optional<gtsam::Matrix&> J_point_conf = boost::none) const;

    /// given pose in configuration space, solve sphere center vector in work
    /// space matlab version, no jacobians output matrix 3 * nr_sphere
    gtsam::Matrix sphereCentersMat(const Pose& jp) const;

    /// accesses from fk_model
    const FKModel& fk_model() const { return fk_model_; }
    size_t dof() const { return fk_model_.dof(); }

    /// accesses
    size_t nr_body_spheres() const { return body_spheres_.size(); }
    size_t sphere_link_id(size_t i) const { return body_spheres_[i].link_id; }
    double sphere_radius(size_t i) const { return body_spheres_[i].radius; }
    const gtsam::Point3& sphere_center_wrt_link(size_t i) const {
        return body_spheres_[i].center;
    }

};

}   // namespace gp_planner

#endif //GP_PLANNER_ARMMODEL_H