
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
    cudaStreamSynchronize(stream);
    cudaStreamDestroy(stream);
    cudaFree(metric_sum);
    cudaFree(metric_x_force_sum);

    cudaFree(policy_debug_data_device);
    cudaFree(policy_debug_data);
    cudaFree(metric_sum_device);
    cudaFree(metric_x_force_sum_device);

    if (output_results) {
      cudaFree(output_cloud);
    }
  }

  void updateLidarData(const LidarData &lidar_data);

  inline void getPolicyDebugData(LidarPolicyDebugData *data) {
    *data = *policy_debug_data;
  }

  inline size_t getDebugData(LidarRayDebugData *data, size_t max_size) {
    if (!output_results) return false;

    size_t size_to_copy = std::min(max_size, (unsigned long)parameters.N_sqrt *
                                                 parameters.N_sqrt) *
                          sizeof(LidarRayDebugData);

    cudaMemcpy(data, output_cloud, size_to_copy, cudaMemcpyDeviceToHost);
    return size_to_copy;
  }

  inline void updateParams(RaycastingCudaPolicyParameters params) {
    parameters = params;
  }
  virtual PValue evaluateAt(const PState &state);
  virtual void startEvaluateAsync(const PState &state);
  virtual void abortEvaluateAsync();

 private:
  RaycastingCudaPolicyParameters parameters;
  void cudaStartEval(const PState &state);
  PState last_evaluated_state;

  bool async_eval_started = false;
  cudaStream_t stream;

  size_t lidar_data_n_points = 0;
  uint8_t *lidar_data_device = nullptr;

  Eigen::Matrix3f *metric_sum_device = nullptr;
  Eigen::Vector3f *metric_x_force_sum_device = nullptr;
  Eigen::Matrix3f *metric_sum = nullptr;
  Eigen::Vector3f *metric_x_force_sum = nullptr;
  LidarPolicyDebugData *policy_debug_data = nullptr;
  LidarPolicyDebugData *policy_debug_data_device = nullptr;

  bool output_results = true;
  LidarRayDebugData *output_cloud;  // x,y,z, |f|,|A|,tbd
};

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_LIDARRAY_CUDA_H
