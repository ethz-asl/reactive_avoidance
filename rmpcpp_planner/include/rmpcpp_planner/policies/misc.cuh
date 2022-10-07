#ifndef RMPCPP_PLANNER_MISC_CUH
#define RMPCPP_PLANNER_MISC_CUH

#include <cmath>

#include "Eigen/Dense"

/**
 * Generates a value between 0 and 1 according to the halton sequence
 * @param index ID of the number
 * @param base Base for the halton sequence
 * @return
 */
inline __device__ __host__ float halton_seq(int index, int base) {
  float f = 1, r = 0;
  while (index > 0) {
    f = f / base;
    r = r + f * (index % base);
    index = index / base;
  }
  return r;
}

template <typename Vector>
inline __device__ __host__ Vector softnorm(const Vector& v, const float c) {
  float norm = v.norm();
  float h = norm + c * log(1.0f + exp(-2.0f * c * norm));

  return v / h;
}

inline float __device__ __host__ alpha_freespace(const float d,
                                                 const float eta_fsp) {
  return eta_fsp * 1.0 / (1.0 + exp(-(2 * d - 6)));
}

/**
 * RMP Obstacle policy repulsive term activation function as defined in the RMP
 * paper
 * @param d Distance to obstacle
 * @param eta_rep
 * @param v_rep
 * @return
 */
inline float __device__ __host__ alpha_rep(const float d, const float eta_rep,
                                           const float v_rep,
                                           const float linear = 0.0) {
  return eta_rep * (exp(-d / v_rep)) + (linear * 1 / d);
}

/**
 * RMP Obstacle policy damping term activation function as defined in the RMP
 * paper
 * @param d Distance to obstacle
 * @param eta_damp
 * @param v_damp
 * @param epsilon
 * @return
 */
inline float __device__ __host__ alpha_damp(const float d, const float eta_damp,
                                            const float v_damp,
                                            const float epsilon) {
  return eta_damp / (d / v_damp + epsilon);
}

/**
 * RMP Obstacle policy metric weighing term as defined in the RMP paper
 * @param d Distance to obstacle
 * @param r
 * @return
 */
inline float __device__ __host__ w(const float d, const float r) {
  if (d > r) {
    return 0.0f;
  }
  return (1.0f / (r * r)) * d * d - (2.0f / r) * d + 1.0f;
}

#endif  // RMPCPP_PLANNER_MISC_CUH
