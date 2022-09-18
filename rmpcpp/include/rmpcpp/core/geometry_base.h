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

#ifndef RMPCPP_CORE_GEOMETRY_BASE_H_
#define RMPCPP_CORE_GEOMETRY_BASE_H_

#include <rmpcpp/core/policy_base.h>
#include <rmpcpp/core/policy_value.h>
#include <rmpcpp/core/state.h>

#include <Eigen/Dense>

namespace rmpcpp {

/** Abstract Base class that fully defines a geometry mapping between two spaces
 * Q and X. It mostly serves to pull everything together and ensure the correct
 * sizes of all used vectors.
 *
 * Q is generally used as configuration space, and X as task space.
 *
 *  \tparam k Dimensionality of Task Space \f$ \mathcal{X} \f$
 *  \tparam d Dimensionality of Config Space \f$ \mathcal{Q} \f$
 *
 */
template <int k, int d>
class GeometryBase {
 public:
  /// Type alias for a Vector belonging to task Space X
  using MatrixX = Eigen::Matrix<double, k, k>;
  using VectorX = Eigen::Matrix<double, k, 1>;
  using PolicyX = PolicyValue<k>;
  using StateX = State<k>;

  /// Type alias for a Vector belonging to Config Space Q
  using MatrixQ = Eigen::Matrix<double, d, d>;
  using VectorQ = Eigen::Matrix<double, d, 1>;
  using PolicyQ = PolicyValue<d>;
  using StateQ = State<d>;

  /// Type for J as defined in Eq 1 in [1]
  using J_phi = Eigen::Matrix<double, k, d>;

  /*
   * Class that represents a fully parametrized geometry
   * that allows pushing and polling.
   * Internal class, as it is built by GeometryBase only
   */
  class ParametrizedGeometry {
    /// Position/Velocity at which thhis geometry is valid
    StateX state_;

    /// Corresponding Jacobian at this state
    J_phi J_;

   public:
    ParametrizedGeometry(J_phi J, StateX state) : J_(J), state_(state) {}

    /**
     * Evaluates a given policy and then pulls it.
     * @param policy_noneval Non-evaluated policy to pull
     */
    template <class NormSpace>
    PolicyQ pull(PolicyBase<NormSpace> &policy_noneval) {
      PolicyX policy = policy_noneval.evaluateAt(state_);
      MatrixQ A = J_.transpose() * policy.A_ * J_;
      VectorQ f = PolicyX::pinv(A) * J_.transpose() * policy.A_ * policy.f_;
      return {f, A};
    }

    /**
     * Takes an evaluated policy and pulls it
     * @param policy Evaluated policy
     */
    PolicyQ pull(const PolicyX policy) {
      MatrixQ A = J_.transpose() * policy.A_ * J_;
      VectorQ f = PolicyX::pinv(A) * J_.transpose() * policy.A_ * policy.f_;
      return {f, A};
    }

    /**
     * Takes an evaluated policy and pushes it
     * @param policy Evaluated Policy
     */
    PolicyX push(const PolicyQ policy) {
      // todo(mpantic): Check if this correct. I think its currently not used.
      auto J_pinv = J_.completeOrthogonalDecomposition().pseudoInverse();
      MatrixX A = J_pinv.transpose() * policy.A_ * J_pinv;
      VectorX f = J_ * policy.f_;
      return {f, A};
    }
  };

  /// Static accessor for dimension K of X
  const static int K;

  /// Static accessor for dimension D of Q
  const static int D;

  /**
   * Creats a fully parametrized geometry for the given state
   * @param state State to parametrize for
   */
  ParametrizedGeometry at(StateX state) {
    return ParametrizedGeometry(J(state), state);
  }

  /**
   * To be implemented in derived classes.
   * Performs coordinate conversion from Q to X.
   * Note: Not guaranteed to be implemented correctly for all geometries.
   */
  virtual StateX convertToX(const StateQ &state_q) const = 0;

  /**
   * To be implemented in derived classes.
   * Performs coordinate conversion from X to Q.
   * Note: Not guaranteed to be implemented correctly for all geometries.
   */
  virtual StateQ convertToQ(const StateX &state_x) const = 0;

  /**
   * To be implemented in derived classes.
   * Calculates Jacobian at position x/x_dot.
   */
  virtual J_phi J(const StateX &state) const = 0;
};

template <int k, int d>
const int GeometryBase<k, d>::K = k;

template <int k, int d>
const int GeometryBase<k, d>::D = d;

}  // namespace rmpcpp

#endif  // RMPCPP_CORE_GEOMETRY_BASE_H_
