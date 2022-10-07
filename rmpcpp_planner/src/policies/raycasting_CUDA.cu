#include <Eigen/QR>
#include <cmath>

#include "nvblox/gpu_hash/cuda/gpu_hash_interface.cuh"
#include "nvblox/gpu_hash/cuda/gpu_indexing.cuh"
#include "nvblox/ray_tracing/sphere_tracer.h"
#include "nvblox/utils/timing.h"
#include "rmpcpp_planner/policies/misc.cuh"
#include "rmpcpp_planner/policies/raycasting_CUDA.h"

#define BLOCKSIZE 8

/********************************************************************
 ****** Ray tracing kernel code (parts adapted from nvblox)
 ********************************************************************/

/** Adapted from NVBLOX */
__device__ inline bool isTsdfVoxelValid(const nvblox::TsdfVoxel &voxel) {
  constexpr float kMinWeight = 1e-4;
  return voxel.weight > kMinWeight;
}
/**
 * Thrusts a ray on a GPU, adapted from NVBLOX
 */
__device__ thrust::pair<float, bool> cast(
    const nvblox::Ray &ray,                                          // NOLINT
    nvblox::Index3DDeviceHashMapType<nvblox::TsdfBlock> block_hash,  // NOLINT
    float truncation_distance_m,                                     // NOLINT
    float block_size_m,                                              // NOLINT
    int maximum_steps,                                               // NOLINT
    float maximum_ray_length_m,                                      // NOLINT
    float surface_distance_epsilon_m) {
  // Approach: Step along the ray until we find the surface, or fail to
  bool last_distance_positive = false;
  // t captures the parameter scaling along ray.direction. We assume
  // that the ray is normalized which such that t has units meters.
  float t = 0.0f;
  for (int i = 0; (i < maximum_steps) && (t < maximum_ray_length_m); i++) {
    // Current point to sample
    const Eigen::Vector3f p_L = ray.origin + t * ray.direction;

    // Evaluate the distance at this point
    float step;
    nvblox::TsdfVoxel *voxel_ptr;

    // Can't get distance, let's see what to do...
    if (!nvblox::getVoxelAtPosition(block_hash, p_L, block_size_m,
                                    &voxel_ptr) ||
        !isTsdfVoxelValid(*voxel_ptr)) {
      // 1) We weren't in observed space before this, let's step through this
      // (unobserved) st
      // uff and hope to hit something allocated.
      if (!last_distance_positive) {
        // step forward by the truncation distance
        step = truncation_distance_m;
        last_distance_positive = false;
      }
      // 2) We were in observed space, now we've left it... let's kill this
      // ray, it's risky to continue.
      // Note(alexmillane): The "risk" here is that we've somehow passed
      // through the truncation band. This occurs occasionally. The risk
      // of continuing is that we can then see through an object. It's safer
      // to stop here and hope for better luck in the next frame.
      else {
        return {t, false};
      }
    }
    // We got a valid distance
    else {
      // Distance negative (or close to it)!
      // We're gonna terminate, let's determine how.
      if (voxel_ptr->distance < surface_distance_epsilon_m) {
        // 1) We found a zero crossing. Terminate successfully.
        if (last_distance_positive) {
          // We "refine" the distance by back stepping the (now negative)
          // distance value
          t += voxel_ptr->distance;
          // Output - Success!
          return {t, true};
        }
        // 2) We just went from unobserved to negative. We're observing
        // something from behind, terminate.
        else {
          return {t, false};
        }
      }
      // Distance positive!
      else {
        // Step by this amount
        step = voxel_ptr->distance;
        last_distance_positive = true;
      }
    }

    // Step further along the ray
    t += step;
  }
  // Ran out of number of steps or distance... Fail
  return {t, false};
}
inline __device__ thrust::pair<float, float> get_angles(const float u,
                                                        const float v) {
  /** Convert uniform sample idx/dimx and idy/dimy to uniform sample on sphere
   */
  float phi = acos(1 - 2 * u);
  float theta = 2.0f * M_PI * v;
  return {phi, theta};
}

__global__ void raycastKernel(
    const Eigen::Vector3f origin, const Eigen::Vector3f vel,
    Eigen::Matrix3f *metric_sum, Eigen::Vector3f *metric_x_force_sum,
    nvblox::Index3DDeviceHashMapType<nvblox::TsdfBlock> block_hash,
    float truncation_distance, float block_size, int maximum_steps,
    float maximum_ray_length, float surface_distance_eps,
    const RaycastingCudaPolicyParameters parameters
#if OUTPUT_RAYS
    ,
    Eigen::Vector3f *ray_endpoints
#endif
) {
  /** Shared memory to sum the components of an RMP */
  using Vector = Eigen::Vector3f;
  using Matrix = Eigen::Matrix3f;

  /** Thread ids */
  const unsigned int idx = threadIdx.x + blockIdx.x * blockDim.x;
  const unsigned int dimx = gridDim.x * blockDim.x;
  const unsigned int idy = threadIdx.y + blockIdx.y * blockDim.y;
  const unsigned int dimy = gridDim.y * blockDim.y;
  const unsigned int id = idx + idy * dimx;

  /** Generate halton sequence and get angles*/
  const float u = halton_seq(id, 2);
  const float v = halton_seq(id, 3);
  thrust::pair<float, float> angles = get_angles(u, v);
  float phi = angles.first;
  float theta = angles.second;

  /** Convert to direction */
  float x = sin(phi) * cos(theta);
  float y = sin(phi) * sin(theta);
  float z = cos(phi);
  nvblox::Ray ray;
  ray.direction = {x, y, z};
  ray.origin = origin;

  thrust::pair<float, bool> result =
      cast(ray, block_hash, truncation_distance, block_size, maximum_steps,
           maximum_ray_length, surface_distance_eps);

  Matrix A;
  Vector metric_x_force;

  if (result.first >= maximum_ray_length) {
    /** No obstacle hit: return */
    A = Matrix::Zero();
    metric_x_force = Vector::Zero();

  } else {
    /** Calculate resulting RMP for this obstacle */
    float distance = result.first;

    Vector direction = ray.direction;

    /** Unit vector pointing away from the obstacle */
    Vector delta_d =
        -direction / direction.norm();  // Direction should be normalized so the
                                        // norm step is maybe redundant

    /** Simple RMP obstacle policy */
    Vector f_rep =
        alpha_rep(distance, parameters.eta_rep, parameters.v_rep, 0.0) *
        delta_d;
    Vector f_damp = -alpha_damp(distance, parameters.eta_damp,
                                parameters.v_damp, parameters.epsilon_damp) *
                    max(0.0, float(-vel.transpose() * delta_d)) *
                    (delta_d * delta_d.transpose()) * vel;
    Vector f = f_rep + f_damp;
    Vector f_norm = softnorm(f, parameters.c_softmax_obstacle);

    if (parameters.metric) {
      A = w(distance, parameters.r) * f_norm * f_norm.transpose();
    } else {
      A = w(distance, parameters.r) * Matrix::Identity();
    }
    A = A / float(dimx * dimy);  // scale with number of rays
    metric_x_force = A * f;
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

  __syncthreads();
  if (threadIdx.x == 0 && threadIdx.y == 0) {
    metric_x_force_sum[blockId] = sum_vectors0;
    metric_sum[blockId] = sum_matrices0;
  }
}
/********************************************************************/

template <class Space>
rmpcpp::RaycastingCudaPolicy<Space>::RaycastingCudaPolicy(
    WorldPolicyParameters *params, nvblox::TsdfLayer *layer,
    NVBloxWorldRMP<Space> *world)
    : layer_(layer),
      parameters_(*dynamic_cast<RaycastingCudaPolicyParameters *>(params)),
      world_(world) {
  cudaStreamCreate(&stream_);
  const int blockdim = parameters_.N_sqrt / BLOCKSIZE;
  /** Somehow trying to malloc device memory here and only deleting it at the
   * end in the destructor, instead of redoing it every integration step does
   * not work. So we only do this for the host memory, which does work
   */
  cudaMallocHost(&metric_sum_, sizeof(Eigen::Matrix3f) * blockdim * blockdim);
  cudaMallocHost(&metric_x_force_sum_,
                 sizeof(Eigen::Vector3f) * blockdim * blockdim);
}
template rmpcpp::RaycastingCudaPolicy<rmpcpp::Space<2>>::RaycastingCudaPolicy(
    WorldPolicyParameters *parameters, nvblox::TsdfLayer *layer,
    NVBloxWorldRMP<rmpcpp::Space<2>> *world);
template rmpcpp::RaycastingCudaPolicy<rmpcpp::Space<3>>::RaycastingCudaPolicy(
    WorldPolicyParameters *parameters, nvblox::TsdfLayer *layer,
    NVBloxWorldRMP<rmpcpp::Space<3>> *world);

template <class Space>
void rmpcpp::RaycastingCudaPolicy<Space>::cudaStartEval(const PState &state) {
  throw std::logic_error("Not implemented");
};

/**
 * Start evaluation. Only implemented for 3d worlds
 * @param state
 */
template <>
void rmpcpp::RaycastingCudaPolicy<rmpcpp::Space<3>>::cudaStartEval(
    const PState &state) {
  /** State vectors */
  Eigen::Vector3f pos = state.pos_.cast<float>();
  Eigen::Vector3f vel = state.vel_.cast<float>();

  nvblox::GPULayerView<nvblox::TsdfBlock> gpu_layer_view =
      layer_->getGpuLayerView();
  const float surface_distance_epsilon =
      parameters_.surface_distance_epsilon_vox * layer_->voxel_size();

  const int blockdim = parameters_.N_sqrt / BLOCKSIZE;
#if OUTPUT_RAYS
  cudaMalloc((void **)&ray_endpoints_device,
             sizeof(Eigen::Vector3f) * parameters.N_sqrt * parameters.N_sqrt);
#endif
  cudaMalloc((void **)&metric_sum_device_,
             sizeof(Eigen::Matrix3f) * blockdim * blockdim);
  cudaMalloc((void **)&metric_x_force_sum_device_,
             sizeof(Eigen::Vector3f) * blockdim * blockdim);
  constexpr dim3 kThreadsPerThreadBlock(BLOCKSIZE, BLOCKSIZE, 1);
  const dim3 num_blocks(blockdim, blockdim, 1);
  raycastKernel<<<num_blocks, kThreadsPerThreadBlock, 0, stream_>>>(
      pos, vel, metric_sum_device_, metric_x_force_sum_device_,
      gpu_layer_view.getHash().impl_,
      parameters_.truncation_distance_vox * layer_->voxel_size(),
      gpu_layer_view.block_size(), parameters_.max_steps, parameters_.r,
      surface_distance_epsilon, parameters_
#if OUTPUT_RAYS
      ,
      ray_endpoints_device
#endif
  );
}

/**
 * Blocking call to evaluate at state.
 * @param state
 * @return
 */
template <>
rmpcpp::RaycastingCudaPolicy<rmpcpp::Space<3>>::PValue
rmpcpp::RaycastingCudaPolicy<rmpcpp::Space<3>>::evaluateAt(
    const PState &state) {
  if (!async_eval_started_) {
    cudaStartEval(state);
  }
  /** If an asynchronous eval was started, no check is done whether the state is
   * the same. (As for now this should never happen)*/
  cudaStreamSynchronize(stream_);
  async_eval_started_ = false;

  const int blockdim = parameters_.N_sqrt / BLOCKSIZE;

  cudaMemcpy(metric_sum_, metric_sum_device_,
             sizeof(Eigen::Matrix3f) * blockdim * blockdim,
             cudaMemcpyDeviceToHost);
  cudaMemcpy(metric_x_force_sum_, metric_x_force_sum_device_,
             sizeof(Eigen::Vector3f) * blockdim * blockdim,
             cudaMemcpyDeviceToHost);

  cudaFree(metric_sum_device_);
  cudaFree(metric_x_force_sum_device_);

  Eigen::Matrix3f sum = Eigen::Matrix3f::Zero();
  Eigen::Vector3f sumv = Eigen::Vector3f::Zero();
  for (int i = 0; i < blockdim * blockdim; i++) {
    sum += metric_sum_[i];
    sumv += metric_x_force_sum_[i];
  }
  if (sum.isZero(0.001)) {  // Check if not all values are 0, leading to
                            // unstable inverse
    return {Vector::Zero(), Matrix::Zero()};
  }

  Eigen::Matrix3d sumd = sum.cast<double>();
  Eigen::Matrix3d sumd_inverse =
      sumd.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::Vector3d sumvd_scaled = sumv.cast<double>();

  Eigen::Vector3d f = sumd_inverse * sumvd_scaled;
  last_evaluated_state_.pos_ = state.pos_;
  last_evaluated_state_.vel_ = state.vel_;

  return {f, sumd};
}

/**
 * Not implemented for 2d
 * @param state
 * @return
 */
template <>
rmpcpp::RaycastingCudaPolicy<rmpcpp::Space<2>>::PValue
rmpcpp::RaycastingCudaPolicy<rmpcpp::Space<2>>::evaluateAt(
    const PState &state) {
  throw std::logic_error("Not implemented");
}

/**
 * Starts asynchronous evaluation (so returns before it is done)
 * @tparam Space
 * @param state
 */
template <class Space>
void rmpcpp::RaycastingCudaPolicy<Space>::startEvaluateAsync(
    const PState &state) {
  cudaStartEval(state);
  async_eval_started_ = true;
}
template void rmpcpp::RaycastingCudaPolicy<
    rmpcpp::Space<2>>::startEvaluateAsync(const PState &state);
template void rmpcpp::RaycastingCudaPolicy<
    rmpcpp::Space<3>>::startEvaluateAsync(const PState &state);

/**
 * Abort asynchronous evaluation
 * @tparam Space
 */
template <class Space>
void rmpcpp::RaycastingCudaPolicy<Space>::abortEvaluateAsync() {
  cudaStreamSynchronize(stream_);
  cudaFree(metric_sum_device_);
  cudaFree(metric_x_force_sum_device_);
  async_eval_started_ = false;
}
template void
rmpcpp::RaycastingCudaPolicy<rmpcpp::Space<2>>::abortEvaluateAsync();
template void
rmpcpp::RaycastingCudaPolicy<rmpcpp::Space<3>>::abortEvaluateAsync();
