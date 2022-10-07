
#ifndef RMPCPP_PLANNER_RAYCASTING_CUDA_H
#define RMPCPP_PLANNER_RAYCASTING_CUDA_H

#include "nvblox/core/common_names.h"
#include "rmpcpp/core/policy_base.h"
#include "rmpcpp_planner/core/parameters.h"
#include "rmpcpp_planner/core/world_rmp.h"
#include "rmpcpp_planner/policies/world_policy_base.h"

namespace rmpcpp {

/*
 * Implements a nvblox- map based raycasting
 * obstacle avoidance policy
 */
template <class Space>
class RaycastingCudaPolicy : public rmpcpp::WorldPolicyBase<Space> {
 public:
  using Vector = typename WorldPolicyBase<Space>::Vector;
  using Matrix = typename WorldPolicyBase<Space>::Matrix;
  using PValue = typename WorldPolicyBase<Space>::PValue;
  using PState = typename WorldPolicyBase<Space>::PState;

  RaycastingCudaPolicy(WorldPolicyParameters *parameters,
                       nvblox::TsdfLayer *layer, NVBloxWorldRMP<Space> *world);

  ~RaycastingCudaPolicy() {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
    cudaFree(metric_sum_);
    cudaFree(metric_x_force_sum_);
  }

  virtual PValue evaluateAt(const PState &state);
  virtual void startEvaluateAsync(const PState &state);
  virtual void abortEvaluateAsync();

 private:
  void cudaStartEval(const PState &state);

  const nvblox::TsdfLayer *layer_;
  const RaycastingCudaPolicyParameters parameters_;
  const NVBloxWorldRMP<Space> *world_ = nullptr;
  PState last_evaluated_state_;

  bool async_eval_started_ = false;
  cudaStream_t stream_;
  Eigen::Matrix3f *metric_sum_device_ = nullptr;
  Eigen::Vector3f *metric_x_force_sum_device_ = nullptr;
  Eigen::Matrix3f *metric_sum_ = nullptr;
  Eigen::Vector3f *metric_x_force_sum_ = nullptr;
};

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_RAYCASTING_CUDA_H
