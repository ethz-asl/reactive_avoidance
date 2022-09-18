#include <Eigen/QR>
#include <cmath>
#include <iostream>

#include "nvblox/gpu_hash/cuda/gpu_hash_interface.cuh"
#include "nvblox/utils/timing.h"
#include "rmpcpp_planner/policies/lidarray_CUDA.h"
#include "rmpcpp_planner/policies/misc.cuh"

#define BLOCKSIZE 16

typedef struct LidarPointOuster {  //     Start     End     Size
  float x;                         //        0       3         4
  float y;                         //        4       7         4
  float z;                         //        8       11        4
  uint32_t unkown_0;               //        12      15        4
  float intensity;                 //        16      19        4
  uint32_t time;                   //        20      23        4
  uint16_t reflectivity;           //        24      25        2
  uint8_t ring;                    //        26      26        1
  uint8_t unknown_1;               //        27      27        1
  uint16_t ambient;                //        28      29        2
  uint16_t unknown_2;              //        30      31        2
  uint32_t range;                  //        32      35        4
  uint32_t unknown_3;              //        36      39        4
  uint32_t unknown_4;              //        40      43        4
  uint32_t unknown_5;              //        44      47        4
  __device__ inline Eigen::Vector3f ray() {
    return {x, y, z};
    // tried doing this with Eigen::Map<> but always somehow got a warp
    // memory alignment problem. so for now leaving the copying.
  }
  __device__ inline float ray_length() { return range / 1000.0; }
} __attribute__((
    packed));  // important! we want same memory alignment as in the message

typedef struct LidarPointSim {  //     Start     End     Size
  float x;                      //        0       3         4
  float y;                      //        4       7         4
  float z;                      //        8       11        4
  float intensity;              //        12      15        4
  uint16_t ring;                //        16      17        2
  uint32_t time;                //        18      21        4

  __device__ inline Eigen::Vector3f ray() { return {x, y, z}; }
  __device__ inline float ray_length() { return ray().norm(); }
} __attribute__((packed));

typedef LidarPointOuster LidarPoint;

__global__ void raycastKernel(
    const Eigen::Vector3f vel, const uint8_t* lidar_data,
    size_t lidar_data_n_points, Eigen::Matrix3f* metric_sum,
    Eigen::Vector3f* metric_x_force_sum, float maximum_ray_length,
    const RaycastingCudaPolicyParameters parameters, bool output_debug,
    rmpcpp::LidarRayDebugData* output_values,
    rmpcpp::LidarPolicyDebugData* policy_debug_data) {
  /** Shared memory to sum the components of an RMP */
  using Vector = Eigen::Vector3f;
  using Matrix = Eigen::Matrix3f;

  /** Thread ids */
  const unsigned int idx = threadIdx.x + blockIdx.x * blockDim.x;
  const unsigned int dimx = gridDim.x * blockDim.x;
  const unsigned int idy = threadIdx.y + blockIdx.y * blockDim.y;
  const unsigned int dimy = gridDim.y * blockDim.y;
  const unsigned int id = idx + idy * dimx;

  LidarPoint* test = (LidarPoint*)lidar_data;

  float ray_length = test[id].ray_length();
  // ray_length -= 0.45;
  output_values[id].topRows<3>(0) = test[id].ray();
  Eigen::Vector3f ray = test[id].ray().normalized();

  Matrix A_obst, A;
  Vector metric_x_force;

  rmpcpp::LidarPolicyDebugData policy_data;

  if (ray_length >= maximum_ray_length || id >= lidar_data_n_points ||
      ray_length <= 0.5 || test[id].reflectivity <= 25) {
    /** No obstacle hit: return */
    A = Matrix::Zero();
    metric_x_force = Vector::Zero();
    policy_data = rmpcpp::LidarPolicyDebugData::Zero();

    output_values[id](3) = 0.0;
    output_values[id](4) = 0.0;
    output_values[id](5) = 0.0;
    output_values[id](6) = 0.0;
    output_values[id](7) = 0.0;
  } else {
    /** Simple RMP obstacle policy */
    Vector f_rep = alpha_rep(ray_length, parameters.eta_rep, parameters.v_rep,
                             parameters.lin_rep) *
                   -ray;
    Vector f_damp = -alpha_damp(ray_length, parameters.eta_damp,
                                parameters.v_damp, parameters.epsilon_damp) *
                    max(0.0, float(-vel.transpose() * -ray)) *
                    (-ray * -ray.transpose()) * vel;
    Vector f_obst = f_rep + f_damp;
    Vector f_norm = softnorm(f_obst, parameters.c_softmax_obstacle);

    if (parameters.metric) {
      A_obst = w(ray_length, parameters.r) * f_norm * f_norm.transpose();
    } else {
      A_obst = w(ray_length, parameters.r) * Matrix::Identity();
    }

    A = A_obst;
    Vector f = f_obst;
    metric_x_force = A * f;

    policy_data.col(0) = f_rep;
    policy_data.col(1) = f_norm;
    policy_data.col(2) = A * f_rep;
    policy_data.col(3) = A * f_norm;

    output_values[id](3) = f.norm();
    output_values[id](4) = A.norm();
    output_values[id](5) = metric_x_force.norm();
    output_values[id](6) = f_damp.norm();
    output_values[id](7) = f_rep.norm();
  }

  const int blockId = blockIdx.x + blockIdx.y * gridDim.x;

  /** Reduction within CUDA block: Start with metric reduction */
  using BlockReduceMatrix =
      typename cub::BlockReduce<Matrix, BLOCKSIZE, cub::BLOCK_REDUCE_RAKING,
                                BLOCKSIZE>;
  __shared__ typename BlockReduceMatrix::TempStorage temp_storage_matrix;
  Matrix sum_matrices0 = BlockReduceMatrix(temp_storage_matrix)
                             .Sum(A);  // Sum calculated on thread 0

  /** Metric x force reduction */
  using BlockReduceVector =
      typename cub::BlockReduce<Vector, BLOCKSIZE, cub::BLOCK_REDUCE_RAKING,
                                BLOCKSIZE>;
  __shared__ typename BlockReduceVector::TempStorage temp_storage_vector;
  Vector sum_vectors0 =
      BlockReduceVector(temp_storage_vector).Sum(metric_x_force);

  /** Reduction within CUDA block: Start with metric reduction */
  using BlockReduceDebugData =
      typename cub::BlockReduce<rmpcpp::LidarPolicyDebugData, BLOCKSIZE,
                                cub::BLOCK_REDUCE_RAKING, BLOCKSIZE>;
  __shared__ typename BlockReduceDebugData::TempStorage temp_storage_debug_data;
  rmpcpp::LidarPolicyDebugData sum_debugdata =
      BlockReduceDebugData(temp_storage_debug_data)
          .Sum(policy_data);  // Sum calculated on thread 0

  __syncthreads();
  if (threadIdx.x == 0 && threadIdx.y == 0) {
    metric_x_force_sum[blockId] = sum_vectors0;
    metric_sum[blockId] = sum_matrices0;
    policy_debug_data[blockId] = sum_debugdata;
  }
}
/********************************************************************/

template <class Space>
rmpcpp::LidarRayCudaPolicy<Space>::LidarRayCudaPolicy(
    RaycastingCudaPolicyParameters params)
    : parameters(*dynamic_cast<RaycastingCudaPolicyParameters*>(&params)) {
  cudaStreamCreate(&stream);
  const int blockdim = parameters.N_sqrt / BLOCKSIZE;
  /** Somehow trying to malloc device memory here and only deleting it at the
   * end in the destructor, instead of redoing it every integration step does
   * not work. So we only do this for the host memory, which does work
   */
  cudaDeviceSetLimit(cudaLimitPrintfFifoSize, 1024 * 1024 * 80);

  cudaMallocHost(&metric_sum, sizeof(Eigen::Matrix3f) * blockdim * blockdim);
  cudaMallocHost(&metric_x_force_sum,
                 sizeof(Eigen::Vector3f) * blockdim * blockdim);
  cudaMallocHost(&policy_debug_data,
                 sizeof(rmpcpp::LidarPolicyDebugData) * blockdim * blockdim);

  cudaMalloc(&policy_debug_data_device,
             sizeof(rmpcpp::LidarPolicyDebugData) * blockdim * blockdim);

  cudaMalloc((void**)&metric_sum_device,
             sizeof(Eigen::Matrix3f) * blockdim * blockdim);
  cudaMalloc((void**)&metric_x_force_sum_device,
             sizeof(Eigen::Vector3f) * blockdim * blockdim);
  cudaMalloc(&lidar_data_device,
             sizeof(LidarPoint) * (parameters.N_sqrt * parameters.N_sqrt + 1));

  // magic memory alignment? we'll see.
  // to be cleaned up.
  long residual = (long)lidar_data_device % sizeof(LidarPoint);
  lidar_data_device += sizeof(LidarPoint) - residual;

  std::cout << "start " << (long)lidar_data_device << std::endl;
  std::cout << "length "
            << (long)sizeof(LidarPoint) * parameters.N_sqrt * parameters.N_sqrt
            << std::endl;
  std::cout << "end "
            << (long)lidar_data_device +
                   sizeof(LidarPoint) * parameters.N_sqrt * parameters.N_sqrt
            << std::endl;

  if (output_results) {
    cudaMalloc(&output_cloud, sizeof(LidarRayDebugData) *
                                  (parameters.N_sqrt * parameters.N_sqrt + 1));

    // manual alignment
    output_results += sizeof(LidarRayDebugData) -
                      ((long)output_results % sizeof(LidarRayDebugData));
  }
}
template rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<2>>::LidarRayCudaPolicy(
    RaycastingCudaPolicyParameters parameters);
template rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<3>>::LidarRayCudaPolicy(
    RaycastingCudaPolicyParameters parameters);

template <class Space>
void rmpcpp::LidarRayCudaPolicy<Space>::cudaStartEval(const PState& state) {
  throw std::logic_error("Not implemented");
};

/**
 * Start evaluation. Only implemented for 3d worlds
 * @param state
 */
template <>
void rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<3>>::cudaStartEval(
    const PState& state) {
  /** State vectors */
  Eigen::Vector3f pos = state.pos_.cast<float>();
  Eigen::Vector3f vel = state.vel_.cast<float>();

  const int blockdim = parameters.N_sqrt / BLOCKSIZE;

  constexpr dim3 kThreadsPerThreadBlock(BLOCKSIZE, BLOCKSIZE, 1);
  const dim3 num_blocks(blockdim, blockdim, 1);

  raycastKernel<<<num_blocks, kThreadsPerThreadBlock, 0, stream>>>(
      vel, lidar_data_device, lidar_data_n_points, metric_sum_device,
      metric_x_force_sum_device, parameters.r * 10, parameters, true,
      output_cloud, policy_debug_data_device);
}

template <>
void rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<3>>::updateLidarData(
    const LidarData& lidar_data) {
  if (lidar_data.n_points > parameters.N_sqrt * parameters.N_sqrt) {
    std::cout << "WARNING POINT CLOUD TRUNCATION - got point cloud with "
              << lidar_data.n_points << " points" << std::endl;
  }

  if (sizeof(LidarPoint) != lidar_data.stride) {
    std::cout << "Point Stride mismatch, ignoring data." << std::endl;
    lidar_data_n_points = 0;
    return;
  }

  size_t data_to_copy =
      std::min(lidar_data.size, (unsigned long)parameters.N_sqrt *
                                    parameters.N_sqrt * sizeof(LidarPoint));
  lidar_data_n_points =
      std::min(lidar_data.n_points,
               (unsigned long)parameters.N_sqrt * parameters.N_sqrt);

  // copy in new data.
  cudaMemcpy(lidar_data_device, (void*)lidar_data.data, data_to_copy,
             cudaMemcpyHostToDevice);
}

/**
 * Blocking call to evaluate at state.
 * @param state
 * @return
 */
template <>
rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<3>>::PValue
rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<3>>::evaluateAt(const PState& state) {
  const int blockdim = parameters.N_sqrt / BLOCKSIZE;

  if (!async_eval_started) {
    cudaStartEval(state);
  }
  /** If an asynchronous eval was started, no check is done whether the state is
   * the same. (As for now this should never happen)*/
  cudaStreamSynchronize(stream);
  async_eval_started = false;

  cudaMemcpy(metric_sum, metric_sum_device,
             sizeof(Eigen::Matrix3f) * blockdim * blockdim,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(metric_x_force_sum, metric_x_force_sum_device,
             sizeof(Eigen::Vector3f) * blockdim * blockdim,
             cudaMemcpyDeviceToHost);

  cudaMemcpy(policy_debug_data, policy_debug_data_device,
             sizeof(rmpcpp::LidarPolicyDebugData) * blockdim * blockdim,
             cudaMemcpyDeviceToHost);

  Eigen::Matrix3f sum = Eigen::Matrix3f::Zero();
  Eigen::Vector3f sumv = Eigen::Vector3f::Zero();
  rmpcpp::LidarPolicyDebugData debug_data;

  for (int i = 0; i < blockdim * blockdim; i++) {
    sum += metric_sum[i];
    sumv += metric_x_force_sum[i];

    debug_data += policy_debug_data[i];
  }
  if (sum.isZero(0.001)) {  // Check if not all values are 0, leading to
                            // unstable inverse
    return {Vector::Zero(), Matrix::Zero()};
  }

  Eigen::Matrix3d sumd = sum.cast<double>();
  Eigen::Matrix3d sumd_inverse =
      sumd.completeOrthogonalDecomposition().pseudoInverse();

  Eigen::Vector3d f = sumd_inverse * sumv.cast<double>();
  last_evaluated_state.pos_ = state.pos_;
  last_evaluated_state.vel_ = state.vel_;

  // recover debug policies
  debug_data.col(0) /=
      (parameters.N_sqrt * parameters.N_sqrt);  // no metric, just average
  debug_data.col(1) /=
      (parameters.N_sqrt * parameters.N_sqrt);  // no metric, just average
  debug_data.col(2) = sumd_inverse.cast<float>() * debug_data.col(2);
  debug_data.col(3) = sumd_inverse.cast<float>() *
                      debug_data.col(3);  // multiply by inverse metric

  return {f, sumd};
}

/**
 * Not implemented for 2d
 * @param state
 * @return
 */
template <>
rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<2>>::PValue
rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<2>>::evaluateAt(const PState& state) {
  std::cout << "LIDARRAYCUDAPOLICY 2 called" << std::endl;
  throw std::logic_error("Not implemented");
}

/**
 * Starts asynchronous evaluation (so returns before it is done)
 * @tparam Space
 * @param state
 */
template <class Space>
void rmpcpp::LidarRayCudaPolicy<Space>::startEvaluateAsync(
    const PState& state) {
  cudaStartEval(state);
  async_eval_started = true;
}
template void rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<2>>::startEvaluateAsync(
    const PState& state);
template void rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<3>>::startEvaluateAsync(
    const PState& state);

/**
 * Abort asynchronous evaluation
 * @tparam Space
 */
template <class Space>
void rmpcpp::LidarRayCudaPolicy<Space>::abortEvaluateAsync() {
  cudaStreamSynchronize(stream);
  cudaFree(metric_sum_device);
  cudaFree(metric_x_force_sum_device);
  async_eval_started = false;
}
template void
rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<2>>::abortEvaluateAsync();
template void
rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<3>>::abortEvaluateAsync();
