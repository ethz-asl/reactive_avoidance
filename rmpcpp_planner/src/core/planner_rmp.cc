
#include "rmpcpp_planner/core/planner_rmp.h"

#include "rmpcpp_planner/core/trajectory_rmp.h"

#define N 1e2

/**
 * Constructor
 * @tparam Space
 * @param parameters
 */
template <class Space>
rmpcpp::PlannerRMP<Space>::PlannerRMP(const ParametersRMP &parameters)
    : PlannerBase<Space>(
          std::make_unique<rmpcpp::NVBloxWorldRMP<Space>>(parameters)),
      parameters(parameters),
      active_trajectories(CmpTrajPtrs<Space>(parameters.sort_type,
                                             parameters.length_type,
                                             parameters.search_heuristic)) {}

template rmpcpp::PlannerRMP<rmpcpp::Space<2>>::PlannerRMP(
    const ParametersRMP &parameters);
template rmpcpp::PlannerRMP<rmpcpp::Space<3>>::PlannerRMP(
    const ParametersRMP &parameters);

/**
 * Create empty trajectory
 * @tparam Space
 * @param start
 */
template <class Space>
void rmpcpp::PlannerRMP<Space>::createTrajectory(
    const rmpcpp::State<Space::dim> &start) {
  std::unique_ptr<TrajectoryRMP<Space>> trajectory(
      new TrajectoryRMP<Space>(this, start));
  active_trajectories.push(trajectory.get());
  trajectories.push_back(std::move(trajectory));
}
template void rmpcpp::PlannerRMP<rmpcpp::Space<2>>::createTrajectory(
    const rmpcpp::State<2> &start);
template void rmpcpp::PlannerRMP<rmpcpp::Space<3>>::createTrajectory(
    const rmpcpp::State<3> &start);

/**
 * Integrate all active trajectories in steps of N. Uses a priority queue to
 * determine the next trajectory to integrate
 * @tparam Space
 */
template <class Space>
void rmpcpp::PlannerRMP<Space>::integrate() {
  while (!this->active_trajectories.empty()) {
    TrajectoryRMP<Space> *trajectory = active_trajectories.top();
    active_trajectories.pop();

    this->total_steps += trajectory->integrate(N);

    /** Add trajectory back to priority queue if it is still active */
    if (trajectory->isActive()) {
      active_trajectories.push(trajectory);
    }
    if (this->goal_reached && parameters.terminate_upon_goal_reached) {
      break;
    }
  }
}
template void rmpcpp::PlannerRMP<rmpcpp::Space<2>>::integrate();
template void rmpcpp::PlannerRMP<rmpcpp::Space<3>>::integrate();

/**
 * Register that a trajectory has reached the goal
 * @tparam Space
 * @param trajectory Trajectory that reached the goal
 */
template <class Space>
void rmpcpp::PlannerRMP<Space>::registerGoalReached(
    TrajectoryRMP<Space> *trajectory) {
  this->goal_reached = true;
  reached_goal_trajectories.push_back(trajectory);
}

template void rmpcpp::PlannerRMP<rmpcpp::Space<2>>::registerGoalReached(
    TrajectoryRMP<Space<2>> *trajectory);
template void rmpcpp::PlannerRMP<rmpcpp::Space<3>>::registerGoalReached(
    TrajectoryRMP<Space<3>> *trajectory);

/**
 * Start planning run
 * @tparam Space
 * @param start
 */
template <class Space>
void rmpcpp::PlannerRMP<Space>::plan(const rmpcpp::State<Space::dim> &start) {
  createTrajectory(start);
  integrate();
}

template void rmpcpp::PlannerRMP<rmpcpp::Space<2>>::plan(
    const rmpcpp::State<Space<2>::dim> &start);
template void rmpcpp::PlannerRMP<rmpcpp::Space<3>>::plan(
    const rmpcpp::State<Space<3>::dim> &start);

/**
 * Get the shortest length to the goal
 * @tparam Space
 * @return Shortest length
 */
template <class Space>
double rmpcpp::PlannerRMP<Space>::getShortestLengthToGoal() {
  /** Find the shortest trajectory */
  double shortest = std::numeric_limits<double>::infinity();
  rmpcpp::TrajectoryRMP<Space> *best = nullptr;
  for (auto &goal_traj : reached_goal_trajectories) {
    shortest = std::min(shortest, goal_traj->totalLength());
  }
  return shortest;
}

/**
 * Get the shortest discrete length to the goal, i.e. how many integration steps
 * for that trajectory
 * @tparam Space
 * @return
 */
template <class Space>
int rmpcpp::PlannerRMP<Space>::getShortestLengthToGoalDiscrete() {
  /** Find the shortest(discrete) trajectory */
  int shortest = std::numeric_limits<int>::max();
  rmpcpp::TrajectoryRMP<Space> *best = nullptr;
  for (auto &goal_traj : reached_goal_trajectories) {
    shortest = std::min(shortest, goal_traj->totalLengthDiscrete());
  }
  return shortest;
}

template <class Space>
double rmpcpp::PlannerRMP<Space>::getSmoothnessToGoal() {
  /** Find the shortest trajectory */
  double shortest = std::numeric_limits<double>::infinity();
  rmpcpp::TrajectoryRMP<Space> *best = nullptr;
  for (auto &goal_traj : reached_goal_trajectories) {
    double length = goal_traj->totalLength();
    if (length < shortest) {
      shortest = length;
      best = goal_traj;
    }
  }
  int length = best->totalLengthDiscrete();

  /** Now get the smoothness, and normalize by length */
  return best->getSmoothness();  // / length; no normalization for aceleration
                                 // based smoothness
}
