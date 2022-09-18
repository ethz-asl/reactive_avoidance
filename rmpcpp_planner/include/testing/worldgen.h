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
  void seed(int seed) { settings.seed = seed; };

  void exportToPly(const std::string &path);
  nvblox::TsdfLayer::Ptr getTsdfLayer() { return tsdf_layer; };
  nvblox::EsdfLayer::Ptr getEsdfLayer() { return esdf_layer; };

  Eigen::Vector3d getStart() { return startpos; };
  Eigen::Vector3d getGoal() { return goal; };

  WorldType getWorldType() { return settings.world_type; };

  std::pair<Eigen::Vector3d, Eigen::Vector3d> getLimits() {
    return {settings.world_limits.first.cast<double>(),
            settings.world_limits.second.cast<double>()};
  };

  double getDensity();

 private:
  struct WorldGenSettings settings;
  nvblox::primitives::Scene scene;
  nvblox::TsdfLayer::Ptr tsdf_layer;

  /** Generated from tsdf using the esdf integrator */
  nvblox::EsdfLayer::Ptr esdf_layer;
  nvblox::EsdfIntegrator esdf_integrator;

  /** Used to export to ply file for visualization. Generated from tsdf using
   * the mesh integrator */
  nvblox::MeshIntegrator mesh_integrator;
  nvblox::MeshLayer::Ptr mesh_layer;

  std::default_random_engine generator;

  std::unique_ptr<nvblox::primitives::Sphere> getRandomSphere(const float &r,
                                                              const float &std);
  std::unique_ptr<nvblox::primitives::Cube> getRandomCube(const float &r,
                                                          const float &std);

  Eigen::Vector3d getRandomLocation();
  Eigen::Vector3d getRandomLocationFreeSpace();

  Eigen::Vector3d startpos = {1.0, 1.0, 1.0};
  Eigen::Vector3d goal = {9.0, 9.0, 9.0};
  std::string lastpath;
};

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_WORLDGEN_H
