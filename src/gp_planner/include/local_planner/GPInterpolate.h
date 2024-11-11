#ifndef GP_PLANNER_GPINTERPOLATE_H
#define GP_PLANNER_GPINTERPOLATE_H

#pragma once

#include "headers.h"

namespace gp_planner{

/* ************************************************************************** */
gtsam::Matrix getQc(const gtsam::SharedNoiseModel Qc_model) {
  gtsam::noiseModel::Gaussian *Gassian_model =
      dynamic_cast<gtsam::noiseModel::Gaussian *>(Qc_model.get());
  return (Gassian_model->R().transpose() * Gassian_model->R()).inverse();
}

/// calculate Q
inline gtsam::Matrix calcQ(const gtsam::Matrix& Qc, double tau) {
  assert(Qc.rows() == Qc.cols());
  return (gtsam::Matrix(2 * Qc.rows(), 2 * Qc.rows())
              << 1.0 / 3 * pow(tau, 3.0) * Qc,
          1.0 / 2 * pow(tau, 2.0) * Qc, 1.0 / 2 * pow(tau, 2.0) * Qc, tau * Qc)
      .finished();
}

/// calculate Q_inv
inline gtsam::Matrix calcQ_inv(const gtsam::Matrix& Qc, double tau) {
  assert(Qc.rows() == Qc.cols());
  const gtsam::Matrix Qc_inv = Qc.inverse();
  return (gtsam::Matrix(2 * Qc.rows(), 2 * Qc.rows())
              << 12.0 * pow(tau, -3.0) * Qc_inv,
          (-6.0) * pow(tau, -2.0) * Qc_inv, (-6.0) * pow(tau, -2.0) * Qc_inv,
          4.0 * pow(tau, -1.0) * Qc_inv)
      .finished();
}

/// calculate Phi
inline gtsam::Matrix calcPhi(size_t dof, double tau) {
  return (gtsam::Matrix(2 * dof, 2 * dof) << gtsam::Matrix::Identity(dof, dof),
          tau * gtsam::Matrix::Identity(dof, dof),
          gtsam::Matrix::Zero(dof, dof), gtsam::Matrix::Identity(dof, dof))
      .finished();
}

/// calculate Lambda
inline gtsam::Matrix calcLambda(const gtsam::Matrix& Qc, double delta_t,
                                const double tau) {
  assert(Qc.rows() == Qc.cols());
  return calcPhi(Qc.rows(), tau) -
         calcQ(Qc, tau) * (calcPhi(Qc.rows(), delta_t - tau).transpose()) *
             calcQ_inv(Qc, delta_t) * calcPhi(Qc.rows(), delta_t);
}

/// calculate Psi
inline gtsam::Matrix calcPsi(const gtsam::Matrix& Qc, double delta_t,
                             double tau) {
  assert(Qc.rows() == Qc.cols());
  return calcQ(Qc, tau) * (calcPhi(Qc.rows(), delta_t - tau).transpose()) *
         calcQ_inv(Qc, delta_t);
}

/// interpolate pose with Jacobians
gtsam::Vector interpolatePose(
    const gtsam::Vector& pose1, const gtsam::Vector& vel1,
    const gtsam::Vector& pose2, const gtsam::Vector& vel2,
    const gtsam::Matrix& Lambda, const gtsam::Matrix& Psi,
    gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1 = std::nullopt,
    gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2 = std::nullopt,
    gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H3 = std::nullopt,
    gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H4 = std::nullopt)  {
size_t dof = pose1.size();
// state vector
gtsam::Vector x1 = (gtsam::Vector(2 * dof) << pose1, vel1).finished();
gtsam::Vector x2 = (gtsam::Vector(2 * dof) << pose2, vel2).finished();

// jacobians
if (H1) *H1 = Lambda.block(0, 0, dof, dof);
if (H2) *H2 = Lambda.block(0, dof, dof, dof);
if (H3) *H3 = Psi.block(0, 0, dof, dof);
if (H4) *H4 = Psi.block(0, dof, dof, dof);

// interpolate pose (just calculate upper part of the interpolated state
// vector to save time)
return Lambda.block(0, 0, dof, 2 * dof) * x1 +
        Psi.block(0, 0, dof, 2 * dof) * x2;
}

/// update jacobian based on interpolated jacobians
void updatePoseJacobians(
    const gtsam::Matrix& Hpose, const gtsam::Matrix& Hint1,
    const gtsam::Matrix& Hint2, const gtsam::Matrix& Hint3,
    const gtsam::Matrix& Hint4, gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2, gtsam::OptionalMatrixType H3,
    gtsam::OptionalMatrixType H4) {
    if (H1) *H1 = Hpose * Hint1;
    if (H2) *H2 = Hpose * Hint2;
    if (H3) *H3 = Hpose * Hint3;
    if (H4) *H4 = Hpose * Hint4;
}

/// interpolate velocity with Jacobians
gtsam::Vector interpolateVelocity(
    const gtsam::Vector& pose1, const gtsam::Vector& vel1,
    const gtsam::Vector& pose2, const gtsam::Vector& vel2,
    const gtsam::Matrix& Lambda, const gtsam::Matrix& Psi,
    gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1 = std::nullopt,
    gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2 = std::nullopt,
    gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H3 = std::nullopt,
    gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H4 = std::nullopt) {
size_t dof = pose1.size();
// state vector
gtsam::Vector x1 = (gtsam::Vector(2 * dof) << pose1, vel1).finished();
gtsam::Vector x2 = (gtsam::Vector(2 * dof) << pose2, vel2).finished();

// jacobians
if (H1) *H1 = Lambda.block(dof, 0, dof, dof);
if (H2) *H2 = Lambda.block(dof, dof, dof, dof);
if (H3) *H3 = Psi.block(dof, 0, dof, dof);
if (H4) *H4 = Psi.block(dof, dof, dof, dof);

// interpolate pose (just calculate lower part of the interpolated state
// vector to save time)
return Lambda.block(dof, 0, dof, 2 * dof) * x1 +
        Psi.block(dof, 0, dof, 2 * dof) * x2;
}

}   //namespace


#endif  //GP_PLANNER_GPINTERPOLATE_H