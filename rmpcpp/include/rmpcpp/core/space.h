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
#ifndef RMPCPP_INCLUDE_RMPCPP_CORE_SPACE_H_
#define RMPCPP_INCLUDE_RMPCPP_CORE_SPACE_H_
#include <Eigen/Dense>
namespace rmpcpp {

/**
 * Represents a mathematical space with a corresponding norm and distance
 * @tparam n Dimensionality
 */
template <int n>
class Space {
 public:
  using Vector = Eigen::Matrix<double, n, 1>;

  virtual Vector minus(Vector v1, Vector v2) { return v1 - v2; }

  virtual double norm(Vector v) { return v.norm(); }

  const static int dim;
};

template <int n>
const int Space<n>::dim = n;

/**
 * Represents a cylindrical space with angular wrap around in the y-axis
 */
class CylindricalSpace : public Space<3> {
 public:
  virtual Vector minus(Vector v1, Vector v2) {
    Vector dist = v1 - v2;

    if (dist.y() > M_PI) {
      dist.y() -= 2.0 * M_PI;
    }
    if (dist.y() < -M_PI) {
      dist.y() += 2.0 * M_PI;
    }
    return dist;
  }
};

}  // namespace rmpcpp
#endif  // RMPCPP_INCLUDE_RMPCPP_CORE_SPACE_H_
