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

#ifndef RMPCPP_GEOMETRY_CYLINDRICAL_GEOMETRY_H_
#define RMPCPP_GEOMETRY_CYLINDRICAL_GEOMETRY_H_

#include <rmpcpp/core/geometry_base.h>

namespace rmpcpp {

/**
 * Example of a cylindrical geometry that maps to a plane.
 * Here, the task space is the unit sphere, and the configuration space is R^3
 *
 * X Task space coordinates are: theta, rho, z
 * Q Configuration space coordinates are: x,y,z
 */
class CylindricalGeometry : public GeometryBase<3, 3> {
  // type alias for readability.
  using base = GeometryBase<3, 3>;

 protected:
  /**
   * Return jacobian.
   */
  virtual typename base::J_phi J(const typename base::StateX &state) const {
    base::J_phi mtx_j(base::J_phi::Identity());

    base::VectorQ q;

    mtx_j(0, 0) = cos(state.pos_.y());
    mtx_j(0, 1) = sin(state.pos_.y());
    mtx_j(1, 0) = -state.pos_.x() * sin(state.pos_.y());
    mtx_j(1, 1) = state.pos_.x() * cos(state.pos_.y());
    // Rest is taken care of by the identity initializer.
    return mtx_j;
  }

 public:
  virtual StateX convertToX(const StateQ &state_q) const {
    // Standard formula for cylindrical coordinates
    StateX state_x;

    // rho
    state_x.pos_.x() = state_q.pos_.topRows<2>().norm();  // sqrt(x^2+y^2)

    // theta
    state_x.pos_.y() = atan2(state_q.pos_.y(), state_q.pos_.x());

    // z
    state_x.pos_.z() = state_q.pos_.z();

    // J(state_x) only works because we know J only uses position...
    state_x.vel_ = J(state_x) * state_q.vel_;

    return state_x;
  }

  virtual StateQ convertToQ(const StateX &state_x) const {
    // Standard formula for cylindrical coordinates
    StateQ state_q;

    // rho
    state_q.pos_.x() = state_x.pos_.x() * cos(state_x.pos_.y());

    // theta
    state_q.pos_.y() = state_x.pos_.x() * sin(state_x.pos_.y());

    // z
    state_q.pos_.z() = state_x.pos_.z();

    return state_q;
  }
};
}  // namespace rmpcpp
#endif  // RMPCPP_GEOMETRY_CYLINDRICAL_GEOMETRY_H_
