
#ifndef RMPCPP_PLANNER_LIDARRAY_CUDA_H
#define RMPCPP_PLANNER_LIDARRAY_CUDA_H

#include "nvblox/core/common_names.h"
#include "rmpcpp/core/policy_base.h"
#include "rmpcpp_planner/core/parameters.h"
#include "rmpcpp_planner/core/world_rmp.h"
#include "rmpcpp_planner/policies/world_policy_base.h"

namespace rmpcpp {

typedef Eigen::Matrix<float, 8, 1>
    LidarRayDebugData;  // for debug data per point
typedef Eigen::Matrix<float, 3, 4>
    LidarPolicyDebugData;  // for overall, blockreduced debug data.

/*
 * Implements a lidar-based obstacle avoidance policy
 */
template <class Space>
class LidarRayCudaPolicy : public rmpcpp::WorldPolicyBase<Space> {
 public:
  typedef struct LidarData {
    const uint8_t *data{nullptr};
    size_t size{0};
    size_t stride{0};
    size_t n_points{0};
  };

  using Vector = typename WorldPolicyBase<Space>::Vector;
  using Matrix = typename WorldPolicyBase<Space>::Matrix;
  using PValue = typename WorldPolicyBase<Space>::PValue;
  using PState = typename WorldPolicyBase<Space>::PState;

  LidarRayCudaPolicy(RaycastingCudaPolicyParameters parameters);

  ~LidarRayCudaPolicy() {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
    cudaFree(metric_sum_);
    cudaFree(metric_x_force_sum_);

    cudaFree(policy_debug_data_device_);
    cudaFree(policy_debug_data_);
    cudaFree(metric_sum_device_);
    cudaFree(metric_x_force_sum_device_);

    if (output_results_) {
      cudaFree(output_cloud_);
    }
  }

  void updateLidarData(const LidarData &lidar_data);

  inline void getPolicyDebugData(LidarPolicyDebugData *data) {
    *data = *policy_debug_data_;
  }

  inline size_t getDebugData(LidarRayDebugData *data, size_t max_size) {
    if (!output_results_) return false;

    size_t size_to_copy = std::min(max_size, (unsigned long)parameters_.N_sqrt *
                                                 parameters_.N_sqrt) *
                          sizeof(LidarRayDebugData);

    cudaMemcpy(data, output_cloud_, size_to_copy, cudaMemcpyDeviceToHost);
    return size_to_copy;
  }

  inline void updateParams(RaycastingCudaPolicyParameters params) {
    parameters_ = params;
  }
  virtual PValue evaluateAt(const PState &state);
  virtual void startEvaluateAsync(const PState &state);
  virtual void abortEvaluateAsync();

 private:
  void cudaStartEval(const PState &state);

  RaycastingCudaPolicyParameters parameters_;
  PState last_evaluated_state_;

  bool async_eval_started_ = false;
  cudaStream_t stream_;

  size_t lidar_data_n_points_ = 0;
  uint8_t *lidar_data_device_ = nullptr;

  Eigen::Matrix3f *metric_sum_device_ = nullptr;
  Eigen::Vector3f *metric_x_force_sum_device_ = nullptr;
  Eigen::Matrix3f *metric_sum_ = nullptr;
  Eigen::Vector3f *metric_x_force_sum_ = nullptr;
  LidarPolicyDebugData *policy_debug_data_ = nullptr;
  LidarPolicyDebugData *policy_debug_data_device_ = nullptr;

  bool output_results_ = true;
  LidarRayDebugData *output_cloud_ = nullptr;  // x,y,z, |f|,|A|,tbd
};

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_LIDARRAY_CUDA_H
