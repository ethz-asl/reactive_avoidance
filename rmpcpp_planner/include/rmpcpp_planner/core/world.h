#ifndef RMPCPP_PLANNER_WORLD_H
#define RMPCPP_PLANNER_WORLD_H

/** RMPCPP_PLANNER */
#include "parameters.h"

/** RMPCPP */
#include "rmpcpp/core/space.h"

/** NVBLOX */
#include <Eigen/Dense>

#include "nvblox/core/common_names.h"
#include "nvblox/core/layer.h"

namespace rmpcpp {

/***
 * Defines the general world in which the robot moves
 * @tparam Space Space in which the world is defined (from rmpcpp/core/space)
 */
template <class Space>
class World {
 protected:
  using Vector = Eigen::Matrix<double, Space::dim, 1>;

 public:
  World() = default;
  virtual ~World() = default;

  Vector* getGoal() { return &goal; };
  virtual void setGoal(const Vector& new_goal) { goal = new_goal; };
  virtual bool collision(const Vector& pos) = 0;

 protected:
  Vector goal = Vector::Zero();
};

/**
 * World that interfaces with NVBlox. TODO: Figure out if keeping this templated
 * makes sense, as 2d in nvblox does not work well due to the blocks
 */
template <class Space>
class NVBloxWorld : public World<Space> {
 public:
  using Vector = typename World<Space>::Vector;

  virtual ~NVBloxWorld() = default;
  NVBloxWorld() = delete;
  NVBloxWorld(const float truncation_distance)
      : truncation_distance_(truncation_distance){};

  bool collision(const Vector& pos) override {
    return collision(pos, tsdf_layer_.get());
  };
  static bool collision(const Vector& pos, nvblox::TsdfLayer* layer);
  double distanceToObstacle(const Vector& pos) {
    return distanceToObstacle(pos, esdf_layer_.get());
  };
  static double distanceToObstacle(const Vector& pos, nvblox::EsdfLayer* layer);
  Vector gradientToObstacle(const Vector& pos);

  void setTsdfLayer(const nvblox::TsdfLayer::Ptr newlayer) {
    tsdf_layer_ = newlayer;
  };
  void setEsdfLayer(const nvblox::EsdfLayer::Ptr newlayer) {
    esdf_layer_ = newlayer;
  };
  nvblox::EsdfLayer* getEsdfLayer() { return esdf_layer_.get(); };
  nvblox::TsdfLayer* getTsdfLayer() { return tsdf_layer_.get(); };

  bool checkMotion(const Vector& s1, const Vector& s2) const;

  template <class VoxelType>
  static VoxelType getVoxel(const Vector& pos,
                            const nvblox::VoxelBlockLayer<VoxelType>* layer,
                            bool* succ);

  double getDensity();

 protected:
  std::shared_ptr<nvblox::EsdfLayer> esdf_layer_;
  std::shared_ptr<nvblox::TsdfLayer> tsdf_layer_;

  const float truncation_distance_;
};

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_WORLD_H
