#include "rmpcpp_planner/core/world.h"

#include "nvblox/core/layer.h"
#include "nvblox/ray_tracing/sphere_tracer.h"
#include "rmpcpp/core/policy_value.h"
#include "rmpcpp/core/space.h"
#include "rmpcpp_planner/policies/raycasting_CUDA.h"
#include "rmpcpp_planner/policies/simple_ESDF.h"

/** Specialized get voxel functions */
/**
 * Get the voxel at a specific location in the layer
 * @tparam VoxelType
 * @param pos Position
 * @param layer
 * @param succ
 * @return
 */
template <>
template <class VoxelType>
VoxelType rmpcpp::NVBloxWorld<rmpcpp::Space<2>>::getVoxel(
    const Vector& pos, const nvblox::VoxelBlockLayer<VoxelType>* layer,
    bool* succ) {
  Eigen::Matrix<double, 3, 1> matrix = {pos[0], pos[1], 0.0};

  std::vector<VoxelType> voxels;
  std::vector<bool> succv = {false};
  layer->getVoxels({matrix.cast<float>()}, &voxels, &succv);
  if (!succv[0]) {
    *succ = false;
    return VoxelType();
  }
  *succ = true;
  return voxels[0];
}

template <>
template <class VoxelType>
VoxelType rmpcpp::NVBloxWorld<rmpcpp::Space<3>>::getVoxel(
    const Vector& pos, const nvblox::VoxelBlockLayer<VoxelType>* layer,
    bool* succ) {
  std::vector<VoxelType> voxels;
  std::vector<bool> succv = {false};
  layer->getVoxels({pos.cast<float>()}, &voxels, &succv);
  if (!succv[0]) {
    *succ = false;
    return VoxelType();
  }
  *succ = true;
  return voxels[0];
}

/**
 * CHeck for collision
 * @tparam Space
 * @param pos
 * @param layer
 * @return True if collided
 */
template <class Space>
bool rmpcpp::NVBloxWorld<Space>::collision(const Vector& pos,
                                           nvblox::TsdfLayer* layer) {
  bool succ = false;
  nvblox::TsdfVoxel voxel = getVoxel<nvblox::TsdfVoxel>(pos, layer, &succ);
  if (!succ) {
    return true;  // Treat unallocated space as collision?
  }
  return voxel.distance < 0.0f;
}

template bool rmpcpp::NVBloxWorld<rmpcpp::Space<2>>::collision(
    const Vector& pos, nvblox::TsdfLayer* layer);
template bool rmpcpp::NVBloxWorld<rmpcpp::Space<3>>::collision(
    const Vector& pos, nvblox::TsdfLayer* layer);

/**
 * Distance to nearest obstacle. Uses ESDF!
 * @tparam Space
 * @param pos
 * @return
 */
template <class Space>
double rmpcpp::NVBloxWorld<Space>::distanceToObstacle(
    const Vector& pos, nvblox::EsdfLayer* layer) {
  bool succ = false;
  nvblox::EsdfVoxel voxel = getVoxel<nvblox::EsdfVoxel>(pos, layer, &succ);
  if (!succ) {
    return 0.0;
  }
  if (voxel.is_inside) {
    return -sqrt(voxel.squared_distance_vox);
  }
  return sqrt(voxel.squared_distance_vox);
}
template double rmpcpp::NVBloxWorld<rmpcpp::Space<2>>::distanceToObstacle(
    const Vector& pos, nvblox::EsdfLayer* layer);
template double rmpcpp::NVBloxWorld<rmpcpp::Space<3>>::distanceToObstacle(
    const Vector& pos, nvblox::EsdfLayer* layer);

/** Check motion between 2 subsequent states by casting a ray between them. The
 * assumption is that state 1 is valid. */
/** Only implemented for 3d below */
template <class Space>
bool rmpcpp::NVBloxWorld<Space>::checkMotion(const Vector& s1,
                                             const Vector& s2) const {
  throw std::runtime_error("Not implemented");
}
template bool rmpcpp::NVBloxWorld<rmpcpp::Space<2>>::checkMotion(
    const Vector& s1, const Vector& s2) const;

template <>
bool rmpcpp::NVBloxWorld<rmpcpp::Space<3>>::checkMotion(
    const Vector& s1, const Vector& s2) const {
  if (collision(s2, this->tsdf_layer.get())) {
    return false;
  }
  if ((s2 - s1).norm() <
      0.0001) {  // Raycasting is inconsistent if they're almost on top of each
                 // other. Assume its okay
    return true;
  }
  nvblox::Ray ray;
  ray.origin = s1.cast<float>();
  ray.direction = ((s2 - s1) / (s2 - s1).norm()).cast<float>();

  nvblox::SphereTracer st;
  st.params().maximum_ray_length_m = float((s2 - s1).norm());
  float t;
  st.castOnGPU(ray, *tsdf_layer.get(), truncation_distance, &t);

  /** We don't care about the returned bool of castOnGPU, just the distance */
  if (t < (s2 - s1).norm()) {
    return false;
  }
  return true;
}

/**
 * Get the gradient to obstacle, used for CHOMP
 * @tparam Space
 * @param pos
 * @return
 */
template <class Space>
typename rmpcpp::NVBloxWorld<Space>::Vector
rmpcpp::NVBloxWorld<Space>::gradientToObstacle(const Vector& pos) {
  bool succ;
  nvblox::EsdfVoxel voxel =
      getVoxel<nvblox::EsdfVoxel>(pos, esdf_layer.get(), &succ);
  if (!succ) {
    return Vector::Ones();
  }
  Vector dir = voxel.parent_direction.cast<double>();
  return dir / dir.norm();
}

template typename rmpcpp::NVBloxWorld<rmpcpp::Space<3>>::Vector
rmpcpp::NVBloxWorld<rmpcpp::Space<3>>::gradientToObstacle(const Vector& pos);

template <>
typename rmpcpp::NVBloxWorld<rmpcpp::Space<2>>::Vector
rmpcpp::NVBloxWorld<rmpcpp::Space<2>>::gradientToObstacle(const Vector& pos) {
  bool succ;
  nvblox::EsdfVoxel voxel =
      getVoxel<nvblox::EsdfVoxel>(pos, esdf_layer.get(), &succ);
  if (!succ) {
    return Vector::Zero();
  }
  Vector dir = voxel.parent_direction({0, 1})
                   .cast<double>();  // ignore 3rd coordinate in 2d
  return dir / dir.norm();
}
