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

#include <rmpcpp_ros/arrow_renderer.h>
namespace rmpcpp {

void ArrowRenderer::addArrow(const Eigen::Vector3d& start,
                             const Eigen::Vector3d& end,
                             const Eigen::Vector3d& viewing_direction,
                             const size_t index, std_msgs::ColorRGBA color,
                             visualization_msgs::Marker* marker) {
  auto rev_direction = start - end;
  auto perp_direction = rev_direction.cross(viewing_direction);

  // check if arrays have right size
  if (marker->points.size() < index + 6) {
    marker->points.resize(index + 6);
  }
  if (marker->colors.size() < index + 6) {
    marker->colors.resize(index + 6);
  }

  // add main body
  marker->points[index] = ptFromEigen(start);
  marker->points[index + 1] = ptFromEigen(end);

  // add sides
  marker->points[index + 2] = ptFromEigen(end);
  marker->points[index + 3] = ptFromEigen(end + rev_direction * tip_length_ +
                                          perp_direction * tip_width_);
  marker->points[index + 4] = ptFromEigen(end);
  marker->points[index + 5] = ptFromEigen(end + rev_direction * tip_length_ -
                                          perp_direction * tip_width_);

  // assign color
  for (size_t i = index; i < index + 6; ++i) {
    marker->colors[i] = color;
  }
}
}  // namespace rmpcpp