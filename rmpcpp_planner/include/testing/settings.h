#ifndef RMPCPP_PLANNER_SETTINGS_H
#define RMPCPP_PLANNER_SETTINGS_H
#include <Eigen/Dense>

#include "nvblox/core/common_names.h"
namespace rmpcpp {

enum PlannerType { RMP };

enum WorldType { SPHERES_ONLY_WORLD, SPHERES_BOX_WORLD, CUSTOM_WORLD };

typedef struct TestSettings {
  int obstacles = 20;
  int seed = -1;
  int n_runs = 1;
  std::string data_path;
  std::string world_save_path;
  std::string world_load_path;
  int stats_only = 0;
  double voxel_truncation_distance = 4.0;
  WorldType world_type = SPHERES_ONLY_WORLD;
  PlannerType planner_type = RMP;
} TestSettings;

/** General (3d) settings */
struct WorldGenSettings {
  explicit WorldGenSettings(TestSettings settings) {
    this->seed = settings.seed;
    this->voxel_truncation_distance = settings.voxel_truncation_distance;
    this->world_type = settings.world_type;
  };

  int seed = 0;

  WorldType world_type = SPHERES_ONLY_WORLD;

  std::pair<Eigen::Vector3f, Eigen::Vector3f> world_limits = {
      {0.0f, 0.0f, 0.0f}, {10.4f, 10.4f, 10.4f}};

  float voxel_size = 0.2f;
  float block_size = nvblox::VoxelBlock<bool>::kVoxelsPerSide * voxel_size;
  float voxel_truncation_distance = 1.0f;
};

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_SETTINGS_H
