#ifndef RMPCPP_PLANNER_PLANNER_RMP_H
#define RMPCPP_PLANNER_PLANNER_RMP_H

#include "world_rmp.h"
#include "parameters.h"
#include "misc.h"

#include "rmpcpp/core/space.h"

#include <queue>
#include "trajectory_rmp.h"

#include "planner_base.h"

namespace rmpcpp{

/** Forward declare trajectory class because of circular dependencies */
template<class Space>
class TrajectoryRMP;

/**
 * The planner class is the top-level entity that handles all planning
 * @tparam Space Space in which the world is defined (from rmpcpp/core/space)
 */
template<class Space>
class PlannerRMP : public PlannerBase<Space>{
    using Vector = Eigen::Matrix<double, Space::dim, 1>;
public:
    friend class TrajectoryRMP<Space>;
    const int dim = Space::dim;
    PlannerRMP(const ParametersRMP &parameters);
    ~PlannerRMP() = default;

    std::vector<std::unique_ptr<TrajectoryRMP<Space>>>* getTrajectories(){return &trajectories;};

    rmpcpp::NVBloxWorldRMP<Space>* getWorld(){
        return dynamic_cast<rmpcpp::NVBloxWorldRMP<Space>*>(this->world.get());
    };

    void registerGoalReached(TrajectoryRMP<Space>* trajectory);
    void registerActiveTrajectory(TrajectoryRMP<Space>* trajectory){
        active_trajectories.push(trajectory);
    };

    void plan(const rmpcpp::State<Space::dim> &start) override;
    double getShortestLengthToGoal() override;
    int getShortestLengthToGoalDiscrete() override;
    double getSmoothnessToGoal() override;

private:
    const ParametersRMP& parameters;

    void createTrajectory(const rmpcpp::State<Space::dim> &start);
    void integrate();


    /** Vector of trajectories pointers. Every trajectory class can hold a tree of trajectories,
     * so generally only 1 root trajectory will be in this vector. */
    std::vector<std::unique_ptr<TrajectoryRMP<Space>>> trajectories;

    /** Priority queue of active trajectories (i.e. ones that can still be integrated) */
    std::priority_queue<TrajectoryRMP<Space>*, std::vector<TrajectoryRMP<Space>*>, CmpTrajPtrs<Space>> active_trajectories;

    /** Vector of leaf node trajectories that reached the goal */
    std::vector<TrajectoryRMP<Space>*> reached_goal_trajectories = {};
};


}


#endif //RMPCPP_PLANNER_PLANNER_RMP_H
