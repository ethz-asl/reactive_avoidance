#ifndef RMPCPP_PLANNER_WORLD_POLICY_BASE_H
#define RMPCPP_PLANNER_WORLD_POLICY_BASE_H

#include "rmpcpp/core/policy_base.h"

namespace rmpcpp {

/**
 * Baseclass for the world obstacle policy. These policies are more elaborate,
 * e.g. casting rays to form a single policy.
 * @tparam Space
 */
template <class Space>
class WorldPolicyBase : public rmpcpp::PolicyBase<Space> {
 public:
  using Vector = typename PolicyBase<Space>::Vector;
  using Matrix = typename PolicyBase<Space>::Matrix;
  using PValue = typename PolicyBase<Space>::PValue;
  using PState = typename PolicyBase<Space>::PState;

  virtual ~WorldPolicyBase() = default;

 protected:
};
}  // namespace rmpcpp
#endif  // RMPCPP_PLANNER_WORLD_POLICY_BASE_H
