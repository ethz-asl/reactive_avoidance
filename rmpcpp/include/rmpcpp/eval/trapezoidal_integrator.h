/**
 * This file is part of RMPCPP
 *
 * Copyright (C) 2020 Michael Pantic <mpantic at ethz dot ch>
 *
 * RMPCPP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RMPCPP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RMPCPP. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RMPCPP_EVAL_INTEGRATOR_H
#define RMPCPP_EVAL_INTEGRATOR_H
#include <rmpcpp/core/policy_value.h>
#include <rmpcpp/core/geometry_base.h>

#include <iostream>
#include <vector>

namespace rmpcpp {

/**
 * Simple Integrator to integrate a trajectory through
 * a policy field.
 *
 * Uses Trapezoidal integration with a user definable timestep.
 * Keeps a distance integral.
 *
 * \tparam TGeometry Geometry on which the integrator operates.
 *                   Has to be inherited from GeometryBase.
 */
template <class TPolicy, class TGeometry>
class TrapezoidalIntegrator {
 public:
  static_assert(
      std::is_base_of<GeometryBase<TGeometry::K, TGeometry::D>,
                      TGeometry>::value,
      "Geometry must inherit from GeometryBase with correct dimensionality");

  using Vector_x = typename TGeometry::VectorX;
  using Matrix_x = typename TGeometry::MatrixX;
  using Vector_q = typename TGeometry::VectorQ;
  using Matrix_q = typename TGeometry::MatrixQ;
  using StateQ = typename TGeometry::StateQ;
  using StateX = typename TGeometry::StateX;

  /**
   * Constructor to supply a policycontainer.
   */
  TrapezoidalIntegrator() {}

  /**
   *  Reset integrator to a specific state.
   */
  void resetTo(const Vector_q position,
               const Vector_q velocity = Vector_q::Zero()) {
    current_pos_ = position;
    current_vel_ = velocity;
    distance_ = 0.0;
    done_ = false;
  }

  /**
   * Advance by dt and return position in configuration space.
   */
  Vector_q forwardIntegrate(std::vector<TPolicy*> policies, TGeometry& geometry,
                            float dt) {
    // relative start and end of integration
    const float a = 0.0;
    const float b = dt;

    // get position in manifold
    Vector_x pos_x, x_dot;
    Vector_q acc_b, vel_b, acc_a, vel_a;
    Vector_q dist_increment;
    acc_a = last_acc_;
    vel_a = current_vel_;

    // get current configuration space position and convert to
    //  task space.
    StateQ current_stateQ{current_pos_, current_vel_};
    StateX current_stateX = geometry.convertToX(current_stateQ);

    // evaluate all policy and get new accelerations
    std::vector<typename TPolicy::PValue> evaluated_policies;
    for (TPolicy* policy : policies) {
      evaluated_policies.push_back(geometry.at(current_stateX).pull(*policy));
    }

    PolicyValue pval = TPolicy::PValue::sum(evaluated_policies);
    acc_b = pval.f_;
    last_metric_ = pval.A_;

    // trapezoidal integration of acceleration.
    vel_b = vel_a + ((b - a) * (acc_a + acc_b) / 2.0);

    // trapezoidal integration of velocity
    dist_increment = (b - a) * (vel_a + vel_b) / 2.0;
    current_pos_ += dist_increment;
    distance_ += dist_increment.norm();
    last_acc_ = acc_b;
    current_vel_ = vel_b;

    return current_pos_;
  }
  Vector_q forwardIntegrateFixed(typename TPolicy::PValue policy, TGeometry& geometry,
                            float dt) {
    // relative start and end of integration
    const float a = 0.0;
    const float b = dt;

    // get position in manifold
    Vector_x pos_x, x_dot;
    Vector_q acc_b, vel_b, acc_a, vel_a;
    Vector_q dist_increment;
    acc_a = last_acc_;
    vel_a = current_vel_;

    // get current configuration space position and convert to
    //  task space.
    StateQ current_stateQ{current_pos_, current_vel_};
    StateX current_stateX = geometry.convertToX(current_stateQ);

    // evaluate all policy and get new accelerations
    acc_b = policy.f_;
    last_metric_ = policy.A_;

    // trapezoidal integration of acceleration.
    vel_b = vel_a + ((b - a) * (acc_a + acc_b) / 2.0);

    // trapezoidal integration of velocity
    dist_increment = (b - a) * (vel_a + vel_b) / 2.0;
    current_pos_ += dist_increment;
    distance_ += dist_increment.norm();
    last_acc_ = acc_b;
    current_vel_ = vel_b;

    return current_pos_;
  }

  /**
   * Returns true if we came to a rest.
   */
  bool isDone() { return done_; }

  /**
   * Returns the total distance in configuration space.
   */
  double totalDistance() { return distance_; }

  Vector_q getPos() { return current_pos_; }

  Vector_q getVel() { return current_vel_; }
  Vector_q getAcc() { return last_acc_; }

  Matrix_q getMetric() { return last_metric_; }

  void getState(Vector_q& pos, Vector_q& vel, Vector_q& acc) {
    pos = current_pos_;
    vel = current_vel_;
    acc = last_acc_;
  }

 private:
  bool done_{false};
  double distance_{0.0};
  Vector_q current_pos_{Vector_q::Zero()};
  Vector_q current_vel_{Vector_q::Zero()};
  Vector_q last_acc_{Vector_q::Zero()};
  Matrix_q last_metric_{Matrix_q::Zero()};
};

}  // namespace rmpcpp

#endif  // RMPCPP_EVAL_INTEGRATOR_H
