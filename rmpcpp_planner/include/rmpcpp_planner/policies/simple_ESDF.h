#ifndef RMPCPP_PLANNER_SIMPLE_ESDF_H
#define RMPCPP_PLANNER_SIMPLE_ESDF_H

#include "nvblox/core/common_names.h"
#include "rmpcpp/core/policy_base.h"
#include "rmpcpp_planner/core/parameters.h"
#include "rmpcpp_planner/policies/world_policy_base.h"

namespace rmpcpp {

/*
 * Implements a policy that follows the obstacle gradient
 * in an ESDF.
 */
template <class Space>
class SimpleEsdfPolicy : public rmpcpp::WorldPolicyBase<Space> {
 public:
  using Vector = typename WorldPolicyBase<Space>::Vector;
  using Matrix = typename WorldPolicyBase<Space>::Matrix;
  using PValue = typename WorldPolicyBase<Space>::PValue;
  using PState = typename WorldPolicyBase<Space>::PState;

  SimpleEsdfPolicy(WorldPolicyParameters* parameters, nvblox::EsdfLayer* layer)
      : layer_(layer),
        parameters_(*dynamic_cast<EsdfPolicyParameters*>(parameters)){};
  ~SimpleEsdfPolicy() {}

  virtual PValue evaluateAt(const PState& state);

 private:
  const nvblox::EsdfLayer* layer_;
  const EsdfPolicyParameters parameters_;
};

}  // namespace rmpcpp
#endif  // RMPCPP_PLANNER_SIMPLE_ESDF_H
