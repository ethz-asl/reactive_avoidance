
#include "rmpcpp_planner/core/planner_rmp.h"

#include <rmpcpp/core/policy_base.h>
#include <rmpcpp/eval/trapezoidal_integrator.h>
#include <rmpcpp/geometry/linear_geometry.h>

#include "rmpcpp_planner/core/trajectory_rmp.h"

/**
 * Constructor
 * @tparam Space
 * @param parameters
 */
template <class Space>
rmpcpp::PlannerRMP<Space>::PlannerRMP(const ParametersRMP &parameters)
    : PlannerBase<Space>(
          std::make_unique<rmpcpp::NVBloxWorldRMP<Space>>(parameters)),
      parameters_(parameters) {}

/**

 * @tparam Space
 */
template <class Space>
void rmpcpp::PlannerRMP<Space>::integrate() {
  if (!trajectory_) {
    return;
  }  // ignore if trajectory is not initalized.

  LinearGeometry<Space::dim> geometry;
  TrapezoidalIntegrator<PolicyBase<Space>, LinearGeometry<Space::dim>>
      integrator;

  // start from end of current trajectory (which should always be initialized
  // when this function is called)
  integrator.resetTo(trajectory_->current().position,
                     trajectory_->current().velocity);

  // reset state
  size_t num_steps = 0;

  while (!this->collided_ && !this->goal_reached_ && !this->diverged_) {
    // evaluate policies
    auto policies = this->getWorld()->getPolicies();
    /** Convert shared pointers to normal pointers for integration step */
    std::vector<PolicyBase<Space> *> policiesRaw;
    policiesRaw.reserve(policies.size());
    std::transform(policies.cbegin(), policies.cend(),
                   std::back_inserter(policiesRaw),
                   [](auto &ptr) { return ptr.get(); });

    // integrate
    integrator.forwardIntegrate(policiesRaw, geometry, parameters_.dt);

    // get new positions
    Vector position, velocity, acceleration;
    integrator.getState(position, velocity, acceleration);

    // update exit conditions
    /** Collision check */
    if (!this->getWorld()->checkMotion(trajectory_->current().position,
                                       position)) {
      this->collided_ = true;
    }

    if ((position - *this->getWorld()->getGoal()).norm() <
        this->goal_tolerance_) {
      this->goal_reached_ = true;
    }

    if (num_steps > parameters_.max_length) {
      this->diverged_ = true;
    }

    num_steps++;
    // store results
    trajectory_->addPoint(position, velocity, acceleration);
  }
}

/**
 * Start planning run
 * @tparam Space
 * @param start
 */
template <class Space>
void rmpcpp::PlannerRMP<Space>::plan(const rmpcpp::State<Space::dim> &start,
                                     const Vector &goal) {
  // Reset states
  this->collided_ = false;
  this->goal_reached_ = false;
  this->diverged_ = false;
  trajectory_ = std::make_unique<TrajectoryRMP<Space>>(start.pos_, start.vel_);

  // as policies live in the world, we have to set the goal there
  this->getWorld()->setGoal(goal);

  // run integrator
  integrate();
}
