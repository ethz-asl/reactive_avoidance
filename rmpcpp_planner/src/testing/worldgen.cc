#include "testing/worldgen.h"

#include <random>

#include "nvblox/core/bounding_boxes.h"
#include "nvblox/core/common_names.h"
#include "nvblox/core/types.h"
#include "nvblox/core/voxels.h"
#include "nvblox/io/mesh_io.h"
#include "nvblox/primitives/primitives.h"
#include "nvblox/primitives/scene.h"
#include "rmpcpp_planner/core/world.h"

#define MARGIN 1

/**
 * NVBlox World Generator
 * @tparam Space
 * @param new_settings Settings
 */
rmpcpp::WorldGen::WorldGen(const struct WorldGenSettings& new_settings)
    : settings_(new_settings) {
  reset();

  /** Set world bounds (axis aligned bounding box) */
  scene_.aabb() = Eigen::AlignedBox3f(settings_.world_limits.first,
                                     settings_.world_limits.second);
}

/**
 * Reset the world
 * @tparam Space
 */
void rmpcpp::WorldGen::reset() {
  scene_.clear();

  /** Reseed random engine such that maps are reproducible. Settings.seed is
   * incremented by the tester class
   * TODO: This is quite ugly maybe, fix this.*/
  if (settings_.seed != -1) {
    generator_.seed(settings_.seed);
  }

  tsdf_layer_.reset(
      new nvblox::TsdfLayer(settings_.voxel_size, nvblox::MemoryType::kUnified));
  esdf_layer_.reset(
      new nvblox::EsdfLayer(settings_.voxel_size, nvblox::MemoryType::kUnified));
  mesh_layer_.reset(new nvblox::MeshLayer(tsdf_layer_->block_size(),
                                         nvblox::MemoryType::kUnified));
}

/**
 * Generate a random world with spheres, or spheres and boxes, with normally
 * distributed radii
 * @tparam Space
 * @param n Number of spheres
 * @param r Mean of the radius
 * @param r_std Standard deviation of the radius
 */
void rmpcpp::WorldGen::generateRandomWorld(const int& n, const float& r,
                                           const float& r_std) {
  reset();
  std::vector<Eigen::Vector3d> forbidden_locations = {startpos_, goal_};

  int valid = 0;
  std::uniform_real_distribution<float> distr(0, 1);
  while (valid < n) {
    std::unique_ptr<nvblox::primitives::Primitive> primitive;

    switch (settings_.world_type) {
      case SPHERES_ONLY_WORLD:
        primitive = getRandomSphere(r, r_std);
        break;

      case SPHERES_BOX_WORLD:

        if (distr(generator_) > 0.5) {
          primitive = getRandomSphere(r, r_std);
        } else {
          primitive = getRandomCube(2 * r, r_std);
        }
        break;

      default:
        // Should not happen.
        std::cout << "ERROR - world-type not implemented" << std::endl;
        break;
    }

    /** Check if it is too close to forbidden loc */
    bool space = true;
    for (Eigen::Vector3d& floc : forbidden_locations) {
      if (primitive->getDistanceToPoint(floc.cast<float>()) < MARGIN) {
        space = false;
        break;
      }
    }
    if (space) {
      valid++;
      scene_.addPrimitive(std::move(primitive));
    }
  }
  /** Generate floor and walls */
  float x_min = settings_.world_limits.first[0];
  float x_max = settings_.world_limits.second[0];
  float y_min = settings_.world_limits.first[1];
  float y_max = settings_.world_limits.second[1];
  float z_min = settings_.world_limits.first[2];
  float z_max = settings_.world_limits.second[2];

  scene_.addGroundLevel(z_min + settings_.voxel_size);
  scene_.addCeiling(z_max - settings_.voxel_size);
  scene_.addPlaneBoundaries(
      x_min + settings_.voxel_size, x_max - settings_.voxel_size,
      y_min + settings_.voxel_size, y_max - settings_.voxel_size);

  /** Generate TSDF */
  scene_.generateSdfFromScene(
      settings_.voxel_truncation_distance * settings_.voxel_size,
      tsdf_layer_.get());

  /** Generate ESDF from TSDF */
  std::vector<Eigen::Vector3i> block_indices =
      tsdf_layer_->getAllBlockIndices();
  esdf_integrator_.integrateBlocksOnGPU(*tsdf_layer_, block_indices,
                                       esdf_layer_.get());
}

std::unique_ptr<nvblox::primitives::Sphere> rmpcpp::WorldGen::getRandomSphere(
    const float& r, const float& std) {
  /** Normal distribution for the radius */
  std::normal_distribution<float> radii(r, std);
  /** Generate radius */
  float radius = radii(generator_);

  return std::make_unique<nvblox::primitives::Sphere>(
      getRandomLocation().cast<float>(), radius);
}

std::unique_ptr<nvblox::primitives::Cube> rmpcpp::WorldGen::getRandomCube(
    const float& r, const float& std) {
  /** Normal distribution for the length of each side*/
  std::normal_distribution<float> lengths_distr(r, std);

  /** Generate side lengths*/
  Eigen::Vector3f lengths;
  for (int j = 0; j < 3; j++) {
    lengths[j] = lengths_distr(generator_);
  }

  return std::make_unique<nvblox::primitives::Cube>(
      getRandomLocation().cast<float>(), lengths);
}

Eigen::Vector3d rmpcpp::WorldGen::getRandomLocation() {
  /** Generate uniform distributions for every dimension */
  std::vector<std::uniform_real_distribution<double>> coord_distr;
  for (int i = 0; i < 3; i++) {
    coord_distr.emplace_back(settings_.world_limits.first[i],
                             settings_.world_limits.second[i]);
  }

  /** Generate location */
  Eigen::Vector3d location;
  for (int j = 0; j < 3; j++) {
    location[j] = coord_distr[j](generator_);
  }

  return location;
}

/**
 * Export the world to a ply file, by first generating a mesh and then exporting
 * @tparam Space
 * @param path Path to save the file
 */
void rmpcpp::WorldGen::exportToPly(const std::string& path) {
  mesh_integrator_.integrateMeshFromDistanceField(*tsdf_layer_, mesh_layer_.get(),
                                                 nvblox::DeviceType::kGPU);
  nvblox::io::outputMeshLayerToPly(*mesh_layer_, path);
}

/**
 * Get the density of this world
 * @return
 */
double rmpcpp::WorldGen::getDensity() {
  unsigned occupied = 0;
  unsigned total = 0;

  for (auto& blockIndex : esdf_layer_->getAllBlockIndices()) {
    nvblox::EsdfBlock::Ptr block = esdf_layer_->getBlockAtIndex(blockIndex);

    unsigned vps = block->kVoxelsPerSide;
    unsigned numvoxels = vps * vps * vps;
    /** This is maybe slightly unreadable, but we're casting our 3d array to a
     * 1d array of the same length, so we can use a 1d range based for loop. */
    for (nvblox::EsdfVoxel& voxel :
         reinterpret_cast<nvblox::EsdfVoxel(&)[numvoxels]>(block->voxels)) {
      if (voxel.is_inside) {
        occupied++;
      }
      total++;
    }
  }
  return double(occupied) / total;
}
