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

#ifndef RMPCPP_ROS_ARROW_RENDERER_H_
#define RMPCPP_ROS_ARROW_RENDERER_H_
#include <rmpcpp_ros/color_map.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

namespace rmpcpp {

/*
 * Helper Class that renders an Arrow as MarkerMessage
 * for RVIZ visualization.
 */
class ArrowRenderer {
 public:
  ArrowRenderer() = default;

  static inline geometry_msgs::Point ptFromEigen(const Eigen::Vector3d& vec) {
    auto pt = geometry_msgs::Point();
    pt.x = vec.x();
    pt.y = vec.y();
    pt.z = vec.z();
    return pt;
  }

  inline void setDefaultViewingDirection(const Eigen::Vector3d& direction) {
    viewing_direction_ = direction;
  }

  /*
   * Helper defintion for shorter call of addArrow.
   */
  inline void addArrow(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                       const size_t index, std_msgs::ColorRGBA color,
                       visualization_msgs::Marker* marker) {
    addArrow(start, end, viewing_direction_, index, color, marker);
  }

  /*
   * Adds an arrow to the marker message.
   * Needs 6 entries in points array.
   *
   * @param start               Start coordinate of the arrow
   * @param end                 End coordinate of arrow
   * @param viewing_direction   Normal of head of arrow
   * @param index               Index position in marker message points.
   * @param marker              The marker to populate
   */
  void addArrow(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                const Eigen::Vector3d& viewing_direction, const size_t index,
                std_msgs::ColorRGBA color, visualization_msgs::Marker* marker);

  float tip_length_{0.3};  // relative length of tip.
  float tip_width_{0.2};   // relative width of tip.
  Eigen::Vector3d viewing_direction_{0.0, 0.0,
                                     1.0};  // default normal for arrow head.
};

}  // namespace rmpcpp

#endif  // RMPCPP_ROS_ARROW_RENDERER_H_
