#include "rmpcpp_planner/core/world_rmp.h"

#include "nvblox/core/layer.h"
#include "rmpcpp/core/policy_value.h"
#include "rmpcpp/core/space.h"
#include "rmpcpp_planner/policies/raycasting_CUDA.h"
#include "rmpcpp_planner/policies/simple_ESDF.h"

/**
 * Get a vector of pointers to the policies that the world defines
 * @tparam Space
 * @return Vector of policies
 */
template <class Space>
std::vector<std::shared_ptr<rmpcpp::PolicyBase<Space>>>
rmpcpp::NVBloxWorldRMP<Space>::getPolicies() {
  std::vector<std::shared_ptr<rmpcpp::PolicyBase<Space>>> policies = {};
  /** Target Policy */
  policies.push_back(this->target_policy_);

  /** 'World Policy' */
  switch (this->parameters_.policy_type) {
    case SIMPLE_ESDF:
      world_policy_ = std::make_shared<rmpcpp::SimpleEsdfPolicy<Space>>(
          this->parameters_.worldPolicyParameters, this->esdf_layer_.get());
      break;
    case RAYCASTING_CUDA:
      world_policy_ = std::make_shared<rmpcpp::RaycastingCudaPolicy<Space>>(
          this->parameters_.worldPolicyParameters, this->tsdf_layer_.get(), this);
      break;
  }
  policies.push_back(world_policy_);
  return policies;
}

template std::vector<std::shared_ptr<rmpcpp::PolicyBase<rmpcpp::Space<2>>>>
rmpcpp::NVBloxWorldRMP<rmpcpp::Space<2>>::getPolicies();
template std::vector<std::shared_ptr<rmpcpp::PolicyBase<rmpcpp::Space<3>>>>
rmpcpp::NVBloxWorldRMP<rmpcpp::Space<3>>::getPolicies();

/**
 * Set a new goal in the world
 * @tparam Space
 * @param new_goal
 */
template <class Space>
void rmpcpp::NVBloxWorldRMP<Space>::setGoal(const Vector &new_goal) {
  World<Space>::setGoal(new_goal);
  this->goal = new_goal;
  this->target_policy_ = std::make_shared<rmpcpp::SimpleTargetPolicy<Space>>(
      new_goal, Eigen::Matrix<double, Space::dim, Space::dim>::Identity(),
      this->parameters_.alpha_target, this->parameters_.beta_target,
      this->parameters_.c_softmax_target);
}

template void rmpcpp::NVBloxWorldRMP<rmpcpp::Space<2>>::setGoal(
    const Vector &new_goal);
template void rmpcpp::NVBloxWorldRMP<rmpcpp::Space<3>>::setGoal(
    const Vector &new_goal);
