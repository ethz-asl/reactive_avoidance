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

#ifndef RMPCPP_CORE_POLICY_BASE_H_
#define RMPCPP_CORE_POLICY_BASE_H_

#include <rmpcpp/core/policy_value.h>
#include <rmpcpp/core/space.h>
#include <rmpcpp/core/state.h>

#include <Eigen/Dense>
#include <Eigen/QR>
#include <iostream>
#include <memory>

// Macro to mark unused variables s.t. no unused warning appears.
#define ACK_UNUSED(expr) \
  do {                   \
    (void)(expr);        \
  } while (0)

namespace rmpcpp {

/**
 * Holds a plain basic n-dimensional Riemannian Motion Policy
 * as defined in [1], Chapter IV.
 *
 * Implements all mathematical base operations on RMPs.
 *
 * \tparam NormSpace Space in which the policy is active (defines dimensionality
 * and distance norm)
 */
template <class NormSpace>
class PolicyBase {
 public:
  const static int n;
  using Matrix = Eigen::Matrix<double, NormSpace::dim, NormSpace::dim>;
  using Vector = Eigen::Matrix<double, NormSpace::dim, 1>;
  using PValue = PolicyValue<NormSpace::dim>;
  using PState = State<NormSpace::dim>;

  // to be implemented in derivatives.
  virtual PValue evaluateAt(const PState &state) = 0;
  /** Asynchronous start of evaluation. If implemented will make a subsequent (blocking) call to evaluateAt
   * (with the same state) faster. */
  virtual void startEvaluateAsync(const PState &state){}; // As a default derivatives don't have to implement this
  virtual void abortEvaluateAsync(){};

  virtual ~PolicyBase() = default;
  /**
   * Setter for metric A
   * @param A Metric
   */
  inline void setA(Matrix A) { A_static_ = A; }

 protected:
  PolicyBase() {}

  NormSpace space_;
  Matrix A_static_;
};

template <class NormSpace>
const int PolicyBase<NormSpace>::n = NormSpace::dim;

}  // namespace rmpcpp

#endif  // RMPCPP_CORE_POLICY_BASE_H_
