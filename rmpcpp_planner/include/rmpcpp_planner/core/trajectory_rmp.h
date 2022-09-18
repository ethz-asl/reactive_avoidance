#ifndef RMPCPP_PLANNER_TRAJECTORY_RMP_H
#define RMPCPP_PLANNER_TRAJECTORY_RMP_H

#include "rmpcpp/core/space.h"
#include "rmpcpp/core/policy_value.h"
#include "rmpcpp/core/policy_base.h"
#include "rmpcpp/eval/trapezoidal_integrator.h"
#include "rmpcpp/geometry/linear_geometry.h"

#include "planner_rmp.h"

#include "parameters.h"

#include <memory>


namespace rmpcpp {

template<class Space>
struct DataRMP{
    using Vector = Eigen::Matrix<double, Space::dim, 1>;
    static std::string dataformat;
    Vector position;
    Vector velocity;
    Vector acceleration;

    double this_cumulative_length = 0.0; // Cumulative length of this trajectory class up until this point (EXCLUDES THE LENGTH OF THE PARENT)
    double velocity_magnitude = sqrt(3); // Because we initialize filter with [1.1.1] todo: fix this hacky approach
    double filtered_velocity_magnitude = sqrt(3);
    float stuck_param = 0.0;
    float stuck_param_filtered = 0.0;

    static std::string getHeaderFormat();
    std::string format();
};

/** Forward declare planner class because of circular dependencies */
template<class Space>
class PlannerRMP;

template<class Space>
class TrajectoryRMP {
    using Vector = Eigen::Matrix<double, Space::dim, 1>;
public:

    /** Initializes with no parent. Root of a trajectory tree */
    TrajectoryRMP(PlannerRMP<Space>* planner, const rmpcpp::State<Space::dim> &start);

    /** Initialize with parent */
    TrajectoryRMP(const ParametersRMP &parameters, TrajectoryRMP<Space>* parent);

    int integrate(const int N);

    double distanceToGoal(){return space.norm(space.minus(datavector.back().position, *goal));};

    int totalLengthDiscrete() {return stats.start_length_discrete + stats.this_length_discrete;};
    double totalLength() {return stats.start_length + datavector[stats.this_length_discrete].this_cumulative_length;};
    bool isActive() {return stats.active;};
    std::vector<std::unique_ptr<TrajectoryRMP<Space>>>* getChildren(){return &children;};

    /** These are public just to make exporting to file a bit easier using an external class */
    struct Stats{
        int start_length_discrete = 0;  // Length of all accumulated parents up to the root

        /** Length of this specific trajectory class. In case a branch has happened, this indicates the branching
         * point, which may be different than the length p,v,a etc. vectors. */
        int this_length_discrete = 0;
        double start_length = 0.0;

        /** Depth of this trajectory in the trajectory tree */
        int depth = 0;

        bool active = true; // Set to false if it has children, has reached the goal, is stuck etc.
        bool goal_reached = false;
        bool collided = false;

    } stats;

    DataRMP<Space> operator[](int i) const {return datavector[i];};
    DataRMP<Space>& operator[](int i) {return datavector[i];};

    /** Get the smoothness of this trajectory, going up the tree */
    double getSmoothness() const;
private:
    PlannerRMP<Space>* planner;
    const ParametersRMP& parameters;

    /** Child trajectories which make up the tree structure
     * Note that every child picks off where the parent stopped. So if a trajectory has children, it will always be
     * inactive. */
    std::vector<std::unique_ptr<TrajectoryRMP<Space>>> children = {};

    std::vector<DataRMP<Space>> datavector;

    const TrajectoryRMP* parent = nullptr; // Can only be set upon construction

    std::vector<std::shared_ptr<PolicyBase<Space>>> policies; // Policy list for the vector fields

    TrapezoidalIntegrator<PolicyBase<Space>, LinearGeometry<Space::dim>> integrator;

    bool exitCondition();

    void appendData();

    Space space;

    const Vector* goal;

};



}

#endif //RMPCPP_PLANNER_TRAJECTORY_RMP_H
