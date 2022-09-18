#include "rmpcpp_planner/policies/simple_ESDF.h"

#include "rmpcpp_planner/policies/misc.cuh"

/** Specialized evaluations */
/** Evaluate the policy at Position, according to policy obstacle in RMP paper.
 * @param pos Position
 * @return Policy Value
 */
template <>
typename rmpcpp::SimpleEsdfPolicy<rmpcpp::Space<3>>::PValue
rmpcpp::SimpleEsdfPolicy<rmpcpp::Space<3>>::evaluateAt(const PState& state) {
  Vector pos = state.pos_;
  Vector vel = state.vel_;
  std::vector<nvblox::EsdfVoxel> voxels;
  std::vector<bool> succ = {false};

  /** Get voxel at the position of the state */
  layer->getVoxels({pos.cast<float>()}, &voxels, &succ);
  if (!succ[0]) {
    throw(std::runtime_error("Unable to evaluate simple ESDF policy\n"));
  }

  auto distance = sqrt(voxels[0].squared_distance_vox);

  Vector direction = voxels[0].parent_direction.cast<double>() * distance;

  Vector obst = pos + direction;

  Vector delta_d = -direction / direction.norm();

  Vector f_rep =
      alpha_rep(distance, parameters.eta_rep, parameters.v_rep) * delta_d;
  Vector f_damp = -alpha_damp(distance, parameters.eta_damp, parameters.v_damp,
                              parameters.epsilon_damp) *
                  std::max(0.0, double(-vel.transpose() * delta_d)) *
                  (delta_d * delta_d.transpose()) * vel;
  Vector f = f_rep + f_damp;

  Vector f_norm = softnorm(f, parameters.c_softmax_obstacle);
  Matrix A = w(distance, parameters.r) * f_norm * f_norm.transpose();
  return {f, A};
}

/** 2D Specialization
 */

template <>
typename rmpcpp::SimpleEsdfPolicy<rmpcpp::Space<2>>::PValue
rmpcpp::SimpleEsdfPolicy<rmpcpp::Space<2>>::evaluateAt(const PState& state) {
  Vector pos = state.pos_;
  Vector vel = state.vel_;
  Eigen::Vector3d pos3d = {pos[0], pos[1], 0.0};  // convert to 3d
  std::vector<nvblox::EsdfVoxel> voxels;
  std::vector<bool> succ = {false};

  layer->getVoxels({pos3d.cast<float>()}, &voxels, &succ);
  if (!succ[0]) {
    throw(std::runtime_error("Unable to evaluate simple ESDF policy\n"));
  }

  auto distance = (double)voxels[0].squared_distance_vox;
  Vector direction = voxels[0].parent_direction.cast<double>()({0, 1}) *
                     distance;  // convert back to 2d
  Vector obst = pos + direction;

  Vector delta_d = -direction / direction.norm();

  Vector f_rep =
      alpha_rep(distance, parameters.eta_rep, parameters.v_rep, 0.0) * delta_d;
  Vector f_damp = alpha_damp(distance, parameters.eta_damp, parameters.v_damp,
                             parameters.epsilon_damp) *
                  std::max(0.0, double(-vel.transpose() * delta_d)) *
                  (delta_d * delta_d.transpose()) * vel;
  Vector f = f_rep + f_damp;

  Vector f_norm = softnorm(f, parameters.c_softmax_obstacle);
  Matrix A = w(distance, parameters.r) * f_norm * f_norm.transpose();
  return {f, A};
  return {Vector::Zero(), Matrix::Identity()};
}
