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

#ifndef RMPCPP_CORE_POLICY_VALUE_H_
#define RMPCPP_CORE_POLICY_VALUE_H_

#include <Eigen/Dense>

namespace rmpcpp {
/**
 * Evaluated policy consisting of concrete f and A
 * @tparam n Dimensionality
 */
template <int n>
class PolicyValue {
 public:
  using Matrix = Eigen::Matrix<double, n, n>;
  using Vector = Eigen::Matrix<double, n, 1>;

  PolicyValue(const Vector &f, const Matrix &A) : A_(A), f_(f) {}

  /**
   * Implemetation of addition operation.
   * Defined in Eq. 8 in [1]
   */
  PolicyValue operator+(PolicyValue &other) {
    Matrix A_combined = this->A_ + other.A_;
    Vector f_combined = pinv(A_combined) * (this->A_ * this->f_ + other.A_ * other.f_);

    return PolicyValue(f_combined, A_combined);
  }

  /**
   * Sum operator as defined in eq. 9 in [1].
   */
  static PolicyValue sum(const std::vector<PolicyValue> RMPBases) {
    Matrix sum_ai = Matrix::Zero();
    Vector sum_ai_fi = Vector::Zero();

    // sum up terms
    for (const auto &RMPBase : RMPBases) {
      sum_ai += RMPBase.A_;
      sum_ai_fi += RMPBase.A_ * RMPBase.f_;
    }

    auto f_summed = pinv(sum_ai) * sum_ai_fi;
    return PolicyValue(f_summed, sum_ai);
  }

  /// Convenience method for pseudo-inverse
  template <int i, int j>
  static inline Eigen::Matrix<double, i, j> pinv(const Eigen::Matrix<double, i, j> &M) {
    return (M.completeOrthogonalDecomposition().pseudoInverse());
  }

  const Matrix A_;
  const Vector f_;
};

}  // namespace rmpcpp

#endif  // RMPCPP_CORE_POLICY_VALUE_H_