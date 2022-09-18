#ifndef RMPCPP_PLANNER_PLANNER_BASE_H
#define RMPCPP_PLANNER_PLANNER_BASE_H

#include <numeric>

#include "Eigen/Dense"
#include "nvblox/core/layer.h"
#include "rmpcpp/core/state.h"
#include "world.h"

namespace rmpcpp {

/**
 * Base class for a planner that uses nvblox as a world.
 * @tparam Space
 */
template <class Space>
class PlannerBase {
 public:
  using Vector = Eigen::Matrix<double, Space::dim, 1>;

  PlannerBase() = default;
  virtual ~PlannerBase() = default;
  explicit PlannerBase(std::unique_ptr<rmpcpp::NVBloxWorld<Space>> world) {
    this->world = std::move(world);
  };

  /** Pure virtual */
  virtual void plan(const rmpcpp::State<Space::dim>& start) = 0;
  virtual double getShortestLengthToGoal() = 0;
  virtual int getShortestLengthToGoalDiscrete() = 0;
  virtual double getSmoothnessToGoal() = 0;

  virtual void setGoal(const Vector& new_goal) {
    this->world->setGoal(new_goal);
  };
  void setTsdf(const nvblox::TsdfLayer::Ptr& tsdflayer) {
    this->world->setTsdfLayer(tsdflayer);
  };
  void setEsdf(const nvblox::EsdfLayer::Ptr& esdflayer) {
    this->world->setEsdfLayer(esdflayer);
  };

  bool reachedGoal() { return goal_reached; };
  int totalSteps() { return total_steps; };

 protected:
  std::unique_ptr<rmpcpp::NVBloxWorld<Space>> world;

  bool goal_reached = false;
  int total_steps = 0;  // Integration steps

  /**
   * Get the length from a trajectory vector
   * @param trajectory n-dimensional position vector
   * @return length
   */
  double lengthFromTrajectory(const std::vector<Vector>& trajectory) {
    using AccType =
        typename std::pair<Vector, double>;  // Custom accumulator type: (last
                                             // position, current length)
    AccType acc = {trajectory[0], 0.0};
    /** Accumulate length starting from the starting position and length = 0 */
    return std::accumulate(
               trajectory.cbegin(), trajectory.cend(), acc,
               [](AccType& acc, const Vector& v) {  // Lambda accumulator
                 double d_length = (v - acc.first).norm();
                 return AccType(v, acc.second + d_length);
               })
        .second;
  }
};

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_PLANNER_BASE_H
