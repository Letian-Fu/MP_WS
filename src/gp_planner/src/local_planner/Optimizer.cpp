#include "Optimizer.h"

namespace gp_planner{

gtsam::Values optimize(const gtsam::NonlinearFactorGraph& graph,
                       const gtsam::Values& init_values,
                       const OptimizerSetting& setting,
                       bool iter_no_increase) {
  std::shared_ptr<gtsam::NonlinearOptimizer> opt;
  std::shared_ptr<gtsam::NonlinearOptimizerParams> params;
  // init the params/opt and type specific settings
  if (setting.opt_type == OptimizerSetting::Dogleg) {
    params =
        std::shared_ptr<gtsam::NonlinearOptimizerParams>(new gtsam::DoglegParams());
    // trust region ranage, 0.2 rad or meter, no whitenning, not sure make sense
    // or not
    dynamic_cast<gtsam::DoglegParams*>(params.get())->setDeltaInitial(0.2);
  } else if (setting.opt_type == OptimizerSetting::LM) {
    params = std::shared_ptr<gtsam::NonlinearOptimizerParams>(
        new gtsam::LevenbergMarquardtParams());
    dynamic_cast<gtsam::LevenbergMarquardtParams*>(params.get())
        ->setlambdaInitial(100.0);

  } else if (setting.opt_type == OptimizerSetting::GaussNewton) {
    params = std::shared_ptr<gtsam::NonlinearOptimizerParams>(
        new gtsam::GaussNewtonParams());
  }
  // common settings
  params->setMaxIterations(setting.max_iter);
  params->setRelativeErrorTol(setting.rel_thresh);
  if (setting.opt_verbosity >= OptimizerSetting::Error)
    params->setVerbosity("ERROR");
  // optimizer
  if (setting.opt_type == OptimizerSetting::Dogleg) {
    opt = std::shared_ptr<gtsam::NonlinearOptimizer>(new gtsam::DoglegOptimizer(
        graph, init_values, *(dynamic_cast<gtsam::DoglegParams*>(params.get()))));
  } else if (setting.opt_type == OptimizerSetting::LM) {
    opt = std::shared_ptr<gtsam::NonlinearOptimizer>(
        new gtsam::LevenbergMarquardtOptimizer(
            graph, init_values,
            *(dynamic_cast<gtsam::LevenbergMarquardtParams*>(params.get()))));
  } else if (setting.opt_type == OptimizerSetting::GaussNewton) {
    opt = std::shared_ptr<gtsam::NonlinearOptimizer>(new gtsam::GaussNewtonOptimizer(
        graph, init_values, *(dynamic_cast<gtsam::GaussNewtonParams*>(params.get()))));
  }
  double currentError = opt->error();
  // check if we're already close enough
  if (currentError <= params->errorTol) {
    if (params->verbosity >= gtsam::NonlinearOptimizerParams::ERROR)
      cout << "Exiting, as error = " << currentError << " < "
           << params->errorTol << endl;
    return opt->values();
  }

  // Maybe show output
  if (params->verbosity >= gtsam::NonlinearOptimizerParams::ERROR)
    cout << "Initial error: " << currentError << endl;

  // Return if we already have too many iterations
  if (opt->iterations() >= params->maxIterations) {
    if (params->verbosity >= gtsam::NonlinearOptimizerParams::TERMINATION)
      cout << "iterations: " << opt->iterations() << " > "
           << params->maxIterations << endl;
    return opt->values();
  }
  gtsam::Values last_values;
  // Iterative loop
  do {
    // iteration
    currentError = opt->error();
    // copy last values in case error increase
    if (iter_no_increase) last_values = opt->values();
    opt->iterate();
    // Maybe show output
    if (params->verbosity >= gtsam::NonlinearOptimizerParams::ERROR)
      cout << "newError: " << opt->error() << endl;

  } while (opt->iterations() < params->maxIterations &&
           !checkConvergence(params->relativeErrorTol, params->absoluteErrorTol,
                             params->errorTol, currentError, opt->error(),
                             params->verbosity));

  // Printing if verbose
  if (params->verbosity >= gtsam::NonlinearOptimizerParams::TERMINATION) {
    cout << "iterations: " << opt->iterations() << " > "
         << params->maxIterations << endl;
    if (opt->iterations() >= params->maxIterations)
      cout << "Terminating because reached maximum iterations" << endl;
  }

  // check whether values increase
  // if increase use last copied values
  if (opt->error() > currentError) {
    if (iter_no_increase) {
      if (params->verbosity >= gtsam::NonlinearOptimizerParams::ERROR)
        cout << "Error increase, use last copied values" << endl;
      return last_values;
    } else {
      return opt->values();
    }
  } else {
    return opt->values();
  }
}

/* ************************************************************************** */
gtsam::Values initArmTrajStraightLine(const gtsam::Vector& init_conf,
                                      const gtsam::Vector& end_conf,
                                      size_t total_step) {
  gtsam::Values init_values;
  // init pose
  for (size_t i = 0; i <= total_step; i++) {
    gtsam::Vector conf;
    if (i == 0)
      conf = init_conf;
    else if (i == total_step)
      conf = end_conf;
    else
      conf =
          static_cast<double>(i) / static_cast<double>(total_step) * end_conf +
          (1.0 - static_cast<double>(i) / static_cast<double>(total_step)) * init_conf;
    init_values.insert(gtsam::Symbol('x', i), conf);
  }
  // init vel as avg vel
  gtsam::Vector avg_vel = (end_conf - init_conf) / static_cast<double>(total_step);
  for (size_t i = 0; i <= total_step; i++)
    init_values.insert(gtsam::Symbol('v', i), avg_vel);

  return init_values;
}

}   //namespace