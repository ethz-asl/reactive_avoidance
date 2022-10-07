#ifndef RMPCPP_PLANNER_WORLDGEN_H
#define RMPCPP_PLANNER_WORLDGEN_H

#include <random>
#include <vector>

#include "nvblox/core/common_names.h"
#include "nvblox/core/types.h"
#include "nvblox/core/voxels.h"
#include "nvblox/integrators/esdf_integrator.h"
#include "nvblox/mesh/mesh_integrator.h"
#include "nvblox/primitives/scene.h"
#include "rmpcpp/core/space.h"
#include "testing/settings.h"

namespace rmpcpp {

/**
 * Random world generator
 * @tparam Space
 */
class WorldGen {
 public:
  WorldGen() = default;
  explicit WorldGen(const struct WorldGenSettings &new_settings);

  void generateRandomWorld(const int &n, const float &r = 1.0,
                           const float &r_std = 0.2);

  void reset();
  void seed(int seed) { settings_.seed = seed; };

  void exportToPly(const std::string &path);
  nvblox::TsdfLayer::Ptr getTsdfLayer() { return tsdf_layer_; };
  nvblox::EsdfLayer::Ptr getEsdfLayer() { return esdf_layer_; };

  Eigen::Vector3d getStart() { return startpos_; };
  Eigen::Vector3d getGoal() { return goal_; };

  WorldType getWorldType() { return settings_.world_type; };

  inline std::pair<Eigen::Vector3d, Eigen::Vector3d> getLimits() {
    return {settings_.world_limits.first.cast<double>(),
            settings_.world_limits.second.cast<double>()};
  };

  double getDensity();

 private:
  std::unique_ptr<nvblox::primitives::Sphere> getRandomSphere(const float &r,
                                                              const float &std);
  std::unique_ptr<nvblox::primitives::Cube> getRandomCube(const float &r,
                                                          const float &std);
  Eigen::Vector3d getRandomLocation();

  struct WorldGenSettings settings_;
  nvblox::primitives::Scene scene_;
  nvblox::TsdfLayer::Ptr tsdf_layer_;

  /** Generated from tsdf using the esdf integrator */
  nvblox::EsdfLayer::Ptr esdf_layer_;
  nvblox::EsdfIntegrator esdf_integrator_;

  /** Used to export to ply file for visualization. Generated from tsdf using
   * the mesh integrator */
  nvblox::MeshIntegrator mesh_integrator_;
  nvblox::MeshLayer::Ptr mesh_layer_;

  std::default_random_engine generator_;

  Eigen::Vector3d startpos_ = {1.0, 1.0, 1.0};
  Eigen::Vector3d goal_ = {9.0, 9.0, 9.0};
};

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_WORLDGEN_H
