
#ifndef RMPCPP_PLANNER_RAYCASTING_CUDA_H
#define RMPCPP_PLANNER_RAYCASTING_CUDA_H

#include "nvblox/core/common_names.h"
#include "rmpcpp/core/policy_base.h"
#include "rmpcpp_planner/core/parameters.h"
#include "rmpcpp_planner/core/world_rmp.h"
#include "rmpcpp_planner/policies/world_policy_base.h"

namespace rmpcpp {

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
    cudaStreamSynchronize(stream);
    cudaStreamDestroy(stream);
    cudaFree(metric_sum);
    cudaFree(metric_x_force_sum);
  }

  virtual PValue evaluateAt(const PState &state);
  virtual void startEvaluateAsync(const PState &state);
  virtual void abortEvaluateAsync();

 private:
  const nvblox::TsdfLayer *layer;
  const RaycastingCudaPolicyParameters parameters;
  const NVBloxWorldRMP<Space> *world = nullptr;

  void cudaStartEval(const PState &state);
  PState last_evaluated_state;

  bool async_eval_started = false;
  cudaStream_t stream;
  Eigen::Matrix3f *metric_sum_device = nullptr;
  Eigen::Vector3f *metric_x_force_sum_device = nullptr;
  Eigen::Matrix3f *metric_sum = nullptr;
  Eigen::Vector3f *metric_x_force_sum = nullptr;
};

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_RAYCASTING_CUDA_H
