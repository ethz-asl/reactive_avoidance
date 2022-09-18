#ifndef RMPCPP_PLANNER_MISC_H
#define RMPCPP_PLANNER_MISC_H

#include "trajectory_rmp.h"
#include "parameters.h"

namespace rmpcpp {

template<class Space>
class TrajectoryRMP;

/** Compare trajectory pointers according to the trajectories' length
 * SMALLEST_FIRST means that the smallest element should be popped first from priority queue, so return comparison with
 * >, as std::priority queue orders with largest elements first, so comparator signs are reversed */
template<class Space>
struct CmpTrajPtrs{
    CmpTrajPtrs() = default;
    CmpTrajPtrs(SortType sort_type, LengthType length_type, Heuristic heuristic)
    : heuristic(heuristic), sort_type(sort_type), length_type(length_type){};

    inline bool operator()(TrajectoryRMP<Space>* left, TrajectoryRMP<Space>* right){
        if(resolveLength(left) == resolveLength(right)){
            return true; // does not matter in this version.
        }
        switch (sort_type) {
            case SMALLEST_FIRST:
                return resolveLength(left) > resolveLength(right);
            case LARGEST_FIRST:
                return resolveLength(left) < resolveLength(right);
            default:
                throw std::runtime_error("Undefined sort type.");
        }
    }

private:
    const Heuristic heuristic = NO_HEURISTIC;
    const SortType sort_type = SMALLEST_FIRST;
    const LengthType length_type = ACTUAL_LENGTH;

    inline double resolveLength(TrajectoryRMP<Space>* trajectory){
        double heuristic_val = 0.0;
        switch (heuristic) {
            case NO_HEURISTIC:
                break;
            case DISTANCE_TO_GOAL:
                heuristic_val = trajectory->distanceToGoal();
                break;
            default:
                throw std::runtime_error("Undefined heuristic type.");
        }

        switch (length_type) {
            case ACTUAL_LENGTH:
                return double(trajectory->totalLength()) + heuristic_val;
            case DISCRETE_LENGTH:
                return double(trajectory->totalLengthDiscrete()) + heuristic_val;
            default:
                throw std::runtime_error("Undefined length function.");
        }
    }
};


}

#endif //RMPCPP_PLANNER_MISC_H
