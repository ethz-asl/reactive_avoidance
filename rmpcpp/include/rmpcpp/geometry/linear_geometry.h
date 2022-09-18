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

#ifndef RMPCPP_GEOMETRY_LINEAR_GEOMETRY_H_
#define RMPCPP_GEOMETRY_LINEAR_GEOMETRY_H_
#include <rmpcpp/core/geometry_base.h>
namespace rmpcpp {

/**
 * Example of a simple linear geometry, where both
 * task and configuration space are some regular
 * euclidean space R^d.
 * \tparam d Dimensionality of geometry.
 */
template <int d>
class LinearGeometry : public GeometryBase<d, d> {
 public:
  // type alias for readability.
  using base = GeometryBase<d, d>;
  using StateX = typename base::StateX;
  using StateQ = typename base::StateQ;

  /**
   * Return jacobian. As the spaces are equal, this
   * is always identity.
   */
  virtual typename base::J_phi J(const StateX &state) const {
    return base::J_phi::Identity();
  }

  /**
   * Convert position.
   * As the spaces are equal, they are the same too.
   */
  virtual StateX convertToX(const StateQ &state_q) const { return state_q; }

  /**
   * Convert position.
   * As the spaces are equal, they are the same too.
   */
  virtual StateQ convertToQ(const StateX &state_x) const { return state_x; }
};
}  // namespace rmpcpp
#endif  // RMPCPP_GEOMETRY_LINEAR_GEOMETRY_H_
