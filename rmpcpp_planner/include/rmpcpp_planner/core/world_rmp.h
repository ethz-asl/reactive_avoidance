#ifndef RMPCPP_PLANNER_WORLD_RMP_H
#define RMPCPP_PLANNER_WORLD_RMP_H

#include "rmpcpp/core/policy_base.h"
#include "rmpcpp/policies/simple_target_policy.h"
#include "rmpcpp_planner/core/world.h"
#include "rmpcpp_planner/policies/world_policy_base.h"

namespace rmpcpp {

template <class Space>
class NVBloxWorldRMP : public NVBloxWorld<Space> {
 public:
  using Vector = typename NVBloxWorld<Space>::Vector;

  explicit NVBloxWorldRMP(const ParametersRMP& parameters)
      : NVBloxWorld<Space>(parameters.truncation_distance_vox),
        parameters(parameters){};

  std::vector<std::shared_ptr<PolicyBase<Space>>> getPolicies();

  void setGoal(const Vector& new_goal) override;

 private:
  const ParametersRMP& parameters;
  std::shared_ptr<rmpcpp::WorldPolicyBase<Space>> world_policy;

  std::shared_ptr<rmpcpp::SimpleTargetPolicy<Space>> target_policy =
      std::make_shared<rmpcpp::SimpleTargetPolicy<Space>>(this->goal);
};

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_WORLD_RMP_H
