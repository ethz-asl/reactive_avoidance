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

#ifndef RMPCPP_POLICIES_SIMPLE_TARGET_POLICY_H_
#define RMPCPP_POLICIES_SIMPLE_TARGET_POLICY_H_

#include <rmpcpp/core/policy_base.h>

namespace rmpcpp {

/**
 * Defines a simple n dimensional target policy, as described in [1].
 * \tparam n Dimensionality of geometry
 */
template <class NormSpace>
class SimpleTargetPolicy : public PolicyBase<NormSpace> {

 public:
  using Vector = typename PolicyBase<NormSpace>::Vector;
  using Matrix = typename PolicyBase<NormSpace>::Matrix;
  using PValue = typename PolicyBase<NormSpace>::PValue;
  using PState = typename PolicyBase<NormSpace>::PState;

  /**
   * Sets up the policy.
   * target is the target to move to.
   * A is the metric to be used.
   * alpha, beta and c are tuning parameters.
   */
  SimpleTargetPolicy(Vector target, Matrix A, double alpha, double beta,
                     double c)
      : target_(target), alpha_(alpha), beta_(beta), c_(c) {
    this->A_static_ = A;
  }

  SimpleTargetPolicy(Vector target) : target_(target) {}

  virtual PValue evaluateAt(const PState &state) {
    Vector f = alpha_ * s(this->space_.minus(target_, state.pos_)) -
               beta_ * state.vel_;
    return {f, this->A_static_};
  }

  void updateParams(double alpha, double beta, double c){
   alpha_ = alpha;
   beta_ = beta;
   c_ = c;
  }

 protected:
  /**
   *  Normalization helper function.
   */
  inline Vector s(Vector x) { return x / h(this->space_.norm(x)); }

  /**
   * Softmax helper function
   */
  inline double h(const double z) {
    return (z + c_ * log(1 + exp(-2 * c_ * z)));
  }

  double alpha_{1.0}, beta_{8.0}, c_{0.005};
  Vector target_;
};

}  // namespace rmpcpp

#endif  // RMPCPP_POLICIES_SIMPLE_TARGET_POLICY_H_
