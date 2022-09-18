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

#include <rmpcpp_ros/color_map.h>

#include <iostream>

namespace rmpcpp {

void ColorMap::scale(const double new_min, const double new_max) {
  // nothing to scale for small color maps
  if (color_map_.size() < 2) {
    return;
  }
  // otherwise get current min/max
  double curr_min = (--color_map_.rend())->first;
  double curr_max = (color_map_.rbegin())->first;

  // invalid scaling, don't do anything.
  if (new_max == new_min || curr_max == curr_min) {
    return;
  }

  double multiplier = (new_max - new_min) / (curr_max - curr_min);

  // copy old map and clear, as keys are immutable
  ColorMapDefinition old_map(color_map_);
  color_map_.clear();

  for (auto item : old_map) {
    double new_key = new_min + item.first * multiplier;
    std::cout << new_key << std::endl;
    color_map_[new_key] = item.second;
  }
}

Eigen::Vector4d ColorMap::operator()(const double value) {
  if (color_map_.empty()) {
    return Eigen::Vector4d{0, 0, 0, 1.0};
  } else if (color_map_.size() == 1) {
    return color_map_.begin()->second;
  } else {
    // linear interpolation, get adjacent values

    // returns first item that is greater or equal to value.
    auto it = color_map_.lower_bound(value);

    // if we get the first or last item as greater or equal,
    // return the first resp. last color, as there is nothing to interpolate.
    if (it == color_map_.begin()) {
      return it->second;
    }
    if (it == color_map_.end()) {
      return (--it)->second;
    }

    // get two adjacent values and their color for interpolation
    double b = it->first;
    auto colorB = it->second;
    it--;
    double a = it->first;
    auto colorA = it->second;

    // linear interpolation between a and b
    double d = (value - a) / (b - a);
    return (1 - d) * colorA + d * colorB;
  }
}

std_msgs::ColorRGBA ColorMap::getColorRGBA(double value) {
  auto color = this->operator()(value);
  std_msgs::ColorRGBA color_rgba;
  color_rgba.r = color[0];
  color_rgba.g = color[1];
  color_rgba.b = color[2];
  color_rgba.a = color[3];
  return color_rgba;
}

}  // namespace rmpcpp