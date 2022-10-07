#ifndef RMPCPP_PLANNER_PLANNER_RMP_H
#define RMPCPP_PLANNER_PLANNER_RMP_H

#include <queue>

#include "parameters.h"
#include "planner_base.h"
#include "rmpcpp/core/space.h"
#include "trajectory_rmp.h"
#include "world_rmp.h"

namespace rmpcpp {
/**
 * The planner class is the top-level entity that handles all planning
 * @tparam Space Space in which the world is defined (from rmpcpp/core/space)
 */
template <class Space>
class PlannerRMP : public PlannerBase<Space> {
  using Vector = Eigen::Matrix<double, Space::dim, 1>;

 public:
  friend class TrajectoryRMP<Space>;
  const int dim = Space::dim;
  PlannerRMP(const ParametersRMP& parameters);
  ~PlannerRMP() = default;

  const std::shared_ptr<TrajectoryRMP<Space>> getTrajectory() const {
    return trajectory_;  // only valid if planner has run.
  };

  bool hasTrajectory() const { return trajectory_.operator bool(); }

  rmpcpp::NVBloxWorldRMP<Space>* getWorld() {
    return dynamic_cast<rmpcpp::NVBloxWorldRMP<Space>*>(this->world_.get());
  };

  void plan(const rmpcpp::State<Space::dim>& start,
            const Vector& goal) override;

 private:
  void integrate();
  
  const ParametersRMP& parameters_;
  std::shared_ptr<TrajectoryRMP<Space>> trajectory_;
};

// explicit instantation
template class rmpcpp::PlannerRMP<rmpcpp::Space<3>>;

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_PLANNER_RMP_H
