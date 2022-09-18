#include "rmpcpp_planner/core/trajectory_rmp.h"
#include <random>


/** Formatting of the data struct for data exporting */
template<class Space>
std::string rmpcpp::DataRMP<Space>::getHeaderFormat() {
    return "x y z vx vy vz ax ay az v_mag ";
}
template std::string rmpcpp::DataRMP<rmpcpp::Space<3>>::getHeaderFormat();

template<>
std::string rmpcpp::DataRMP<rmpcpp::Space<2>>::getHeaderFormat() {
    return "x y vx vy ax ay v_mag ";
}

template<class Space>
std::string rmpcpp::DataRMP<Space>::format() {
    Eigen::IOFormat format(Eigen::FullPrecision, Eigen::DontAlignCols, " ", " ", "", "", " ", "");
    std::stringstream str;
    str << position.format(format) << velocity.format(format) << acceleration.format(format)
        << " " << velocity.norm();
    return str.str();
}
template std::string rmpcpp::DataRMP<rmpcpp::Space<2>>::format();
template std::string rmpcpp::DataRMP<rmpcpp::Space<3>>::format();
/**************************************************/


/**
 * Create a root trajectory with a starting point
 * @tparam Space
 * @param parameters
 * @param start Starting state
 * @param policies World policies
 */
template<class Space>
rmpcpp::TrajectoryRMP<Space>::TrajectoryRMP(PlannerRMP<Space>* planner, const rmpcpp::State<Space::dim> &start)
                                      : planner(planner), parameters(planner->parameters), policies(planner->getWorld()->getPolicies()),
                                        goal(planner->getWorld()->getGoal()){

    integrator.resetTo(start.pos_, start.vel_);

    datavector.push_back(DataRMP<Space>());

    /** Save initial position (and default 0 acceleration) */
    datavector.back().position = start.pos_;
    datavector.back().velocity = start.vel_;
    datavector.back().acceleration = Vector::Zero();
}


template rmpcpp::TrajectoryRMP<rmpcpp::Space<2>>::TrajectoryRMP(PlannerRMP<Space<2>>* planner, const rmpcpp::State<Space<2>::dim> &start);
template rmpcpp::TrajectoryRMP<rmpcpp::Space<3>>::TrajectoryRMP(PlannerRMP<Space<3>>* planner, const rmpcpp::State<Space<3>::dim> &start);

/**
 * Create a trajectory from a parent trajectory
 * @tparam Space
 * @param parent Parent trajectory in the tree structure
 * @param parameters
 */
template<class Space>
rmpcpp::TrajectoryRMP<Space>::TrajectoryRMP(const ParametersRMP &parameters, TrajectoryRMP<Space>* parent)
        : planner(parent->planner), parameters(parameters),
        parent(parent), policies(parent->policies), goal(planner->getWorld()->getGoal()){
    stats.start_length_discrete = parent->totalLengthDiscrete();
    stats.start_length = parent->totalLength();
    stats.depth = parent->stats.depth + 1;

    /** Set starting point */
    int parent_length = parent->stats.this_length_discrete;
    Vector startpos = parent->datavector[parent_length].position;
    Vector startvel = parent->datavector[parent_length].velocity;
    Vector startacc = parent->datavector[parent_length].acceleration;
    integrator.resetTo(startpos, startvel);

    datavector.push_back(DataRMP<Space>());
    datavector.back().position = startpos;
    datavector.back().velocity = startvel;
    datavector.back().acceleration = startacc;
}
template rmpcpp::TrajectoryRMP<rmpcpp::Space<2>>::TrajectoryRMP(const ParametersRMP &parameters, TrajectoryRMP<Space<2>> *parent);
template rmpcpp::TrajectoryRMP<rmpcpp::Space<3>>::TrajectoryRMP(const ParametersRMP &parameters, TrajectoryRMP<Space<3>> *parent);

/**
 * Integrates the trajectory for a maximum N steps. Aborts on exit condition/local minima detected
 * @tparam Space
 * @param N Maximum number of integration steps
 * @return Number of steps integrated (< N in case of abort)
 */
template<class Space>
int rmpcpp::TrajectoryRMP<Space>::integrate(const int N) {
    LinearGeometry<Space::dim> geometry; // TODO: Put this in a proper place.
    int i = 0;
    for(; i < N; i++){
        /** Start with asynchronous evaluation */
        Vector pos = integrator.getPos(); Vector vel = integrator.getVel();
        for(auto &policy : policies){
            policy->startEvaluateAsync({pos, vel});
        }

        /** Do other checks and bookkeeping */
        if(exitCondition()){
            stats.active = false;
            for(auto &policy : policies){
                policy->abortEvaluateAsync();
            }
            break;
        }

        /** Convert shared pointers to normal pointers for integration step */
        std::vector<PolicyBase<Space>*> policiesRaw;
        policiesRaw.reserve(policies.size());
        std::transform(policies.cbegin(), policies.cend(), std::back_inserter(policiesRaw),
            [](auto& ptr) { return ptr.get(); });

        /** Blocking call to integration step */
        integrator.forwardIntegrate(policiesRaw, geometry, parameters.dt);
    }


    return i; // Return number of integrated steps
}
template int rmpcpp::TrajectoryRMP<rmpcpp::Space<2>>::integrate(const int N);
template int rmpcpp::TrajectoryRMP<rmpcpp::Space<3>>::integrate(const int N);

/**
 * Append some data to the statistics vector after integration step
 * @tparam Space

 */
template<class Space>
void rmpcpp::TrajectoryRMP<Space>::appendData() {
    datavector.push_back(DataRMP<Space>());

    datavector.back().position = integrator.getPos();
    datavector.back().velocity = integrator.getVel();
    datavector.back().acceleration = integrator.getAcc();

    stats.this_length_discrete++;
    double d_length = space.norm(space.minus(datavector.end()[-1].position, datavector.end()[-2].position));
    if(datavector.empty()){
        datavector.back().this_cumulative_length = d_length;
    }
    else{
        datavector.back().this_cumulative_length = d_length + datavector.end()[-2].this_cumulative_length;
    }
}
template void rmpcpp::TrajectoryRMP<rmpcpp::Space<2>>::appendData();
template void rmpcpp::TrajectoryRMP<rmpcpp::Space<3>>::appendData();

/**
 * Check for exit conditions
 * @tparam Space
 * @return True if exit condition has happened
 */
template<class Space>
bool rmpcpp::TrajectoryRMP<Space>::exitCondition() {

    /** Collision check */
    if(datavector.size() > 1 && !planner->world.get()->checkMotion(datavector.end()[-2].position, datavector.end()[-1].position)){
        stats.collided = true;
        return true;
    }

    /** Check if the goal has been reached */
    if(space.norm(space.minus(datavector.back().position, *goal)) < 0.05){
        stats.goal_reached = true;
        planner->registerGoalReached(this);
        return true;
    }

    /** Max length check */
    if(totalLengthDiscrete() > parameters.max_length){
        return true;
    }

    return false;
}
/**
 * Get the smoothness of this trajectory, going up the tree to the start. Note that the smoothness is NOT NORMALIZED YET
 * @tparam Space
 * @return NOT NORMALIZED Smoothness
 */
template<class Space>
double rmpcpp::TrajectoryRMP<Space>::getSmoothness() const{
    double smoothness = 0;
    for(int i = 2; i < stats.this_length_discrete; i++){
        Vector A = datavector[i - 2].position;
        Vector B = datavector[i - 1].position;
        Vector C = datavector[i].position;
        A = B - A;
        B = C - B;
        smoothness += 1 - 1 / M_PI * atan2((A.cross(B)).norm(), A.dot(B));
    }

    /** Right now we neglect the step inbetween parent and child, though this is probably negligible */
    if(parent){
        smoothness += parent->getSmoothness();
    }
    return smoothness;
}

template double rmpcpp::TrajectoryRMP<rmpcpp::Space<3>>::getSmoothness() const;

/** Cross product does not work for 2d vectors. */
template<>
double rmpcpp::TrajectoryRMP<rmpcpp::Space<2>>::getSmoothness() const{
    throw std::runtime_error("Not implemented");
}