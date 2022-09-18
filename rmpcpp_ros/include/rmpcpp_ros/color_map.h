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

#ifndef RMPCPP_ROS_COLOR_MAP_H_
#define RMPCPP_ROS_COLOR_MAP_H_
#include <std_msgs/ColorRGBA.h>

#include <Eigen/Dense>
#include <map>
#include <utility>

namespace rmpcpp {

/*
 * Helper class to create piecewise color maps
 */
class ColorMap {
 public:
  /*
   * A color map contains double values and corresponding colors.
   * Colors are represented as 4D eigen arrays.
   */
  typedef std::map<double, Eigen::Vector4d> ColorMapDefinition;

  /*
   * Initializes with a default color map
   * where  0 = red,
   *        50 = white
   *        100 = blue
   */
  ColorMap()
      : color_map_({{0, {1.0, 0.0, 0.0, 1.0}},
                    {80, {1.0, 1.0, 1.0, 1.0}},
                    {100.0, {0.0, 0.0, 1.0, 1.0}}}) {}

  /*
   *  Constructor for custom colormap.
   */
  explicit ColorMap(ColorMapDefinition map) : color_map_(std::move(map)) {}

  /*
   * Initializes with one color colormap
   * @param color The color for all valeus.
   */
  explicit ColorMap(const Eigen::Vector4d& color) {
    color_map_.clear();
    color_map_[0.0] = color;
  }

  void scale(double min, double max);

  /*
   * Return color for this value.
   */
  Eigen::Vector4d operator()(double value);

  /*
   * Return color as ROS colorRGBA for this value.
   */
  std_msgs::ColorRGBA getColorRGBA(double value);

 private:
  ColorMapDefinition color_map_;
};
}  // namespace rmpcpp
#endif  // RMPCPP_ROS_COLOR_MAP_H_
