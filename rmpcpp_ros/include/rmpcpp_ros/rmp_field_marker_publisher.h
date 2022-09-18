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

#ifndef RMPCPP_ROS_INCLUDE_RMP_MARKER_PUBLISHER_H_
#define RMPCPP_ROS_INCLUDE_RMP_MARKER_PUBLISHER_H_
#include <rmpcpp/core/state.h>
#include <rmpcpp/util/vector_range.h>
#include <rmpcpp_ros/arrow_renderer.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace rmpcpp {

template <int n>
class RMPFieldMarkerPublisher {
 public:
  /*
   * Function signature for conversion function from
   * vector in space Q to 3D Vector for visualization.
   */
  typedef std::function<Eigen::Vector3d(const State<n>&)> CoordinateConversion;

  /*
   * Enum that holds the different possible length modes for the arrows.
   */
  enum class LengthMode {
    fixed,
    scaledLinear
  };  // To be added later: e.g. scaled log

  /*
   * Enum that holds the different possible coloring modes for the arrows.
   */
  enum class ColorMode {
    xCoord,
    yCoord,
    zCoord,
    norm
  };  // To be added later: e.g. direction

  RMPFieldMarkerPublisher(ros::NodeHandle& node_handle,
                          const std::string& topic) {
    pub_markers_ =
        node_handle.advertise<visualization_msgs::Marker>(topic, 1, true);
  }

  /*
   * Setter for Colormap.
   */
  inline void setColorMap(ColorMap map) { colormap_ = map; }

  /*
   * Getter for colormap.
   * Returns reference, so it can be modified.
   */
  inline ColorMap& getColorMap() { return colormap_; }

  /*
   * Sets the length mode according to the enum.
   */
  inline void setLengthMode(LengthMode mode) { length_mode_ = mode; }

  /*
   * Sets the color mode according to the enum.
   */
  inline void setColorMode(ColorMode mode) { color_mode_ = mode; }

  /*
   * Sets the length factor.
   * Effect depends on length mode.
   */
  inline void setLengthFactor(double factor) { arrow_length_ = factor; }

  /*
   * Sets a custom conversion for the coordinates.
   * As the Vectors in Q Space must be transformed to ordinary 3D
   * vectors for visualization, this might need special functionality
   * depending on the usecase.
   */
  inline void setCustomConversion(CoordinateConversion conversion) {
    conversion_ = conversion;
  }

  /*
   * Default conversation that works if Vector_Q is
   * at least a 2D Eigen Vector.
   */
  static Eigen::Vector3d defaultConversion(const State<n>& input) {
    // In case we deal with something 2D.
    // In case we deal with higher dimensional data, the user has to specify a
    // custom conversion.
    if constexpr (n == 2) {
      return {input.pos_[0], input.pos_[1], 0.0};
    } else {
      return {input.pos_[0], input.pos_[1], input.pos_[2]};
    }
  }

  /*
   * Publishes a visualization of the given policy
   * sampled with the given range.
   */
  template <class TGeometry, class NormSpace>
  void publish(TGeometry* geometry, PolicyBase<NormSpace>* policy,
               VectorRange<typename TGeometry::VectorX>& range,
               const std::string& frame_id = "world",
               const ros::Time& stamp = ros::Time::now()) {
    visualization_msgs::Marker msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.ns = "policy";
    msg.id = 0;
    msg.type = visualization_msgs::Marker::LINE_LIST;
    msg.action = visualization_msgs::Marker::ADD;
    msg.scale.x = 0.01;

    msg.points.resize(range.length() * 6);
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.colors.resize(range.length() * 6);
    msg.color.a = 1.0;

    int index = 0;
    for (const auto position : range) {
      typename TGeometry::StateX state_x(position);
      typename TGeometry::StateQ state_q;

      state_q = geometry->convertToQ(state_x);

      // Compute acceleration
      typename TGeometry::PolicyQ pol_q = geometry->at(state_x).pull(*policy);

      // set up vector
      typename TGeometry::VectorQ start = state_q.pos_;
      typename TGeometry::VectorQ end = state_q.pos_;

      switch (length_mode_) {
        case LengthMode::scaledLinear:
          end += pol_q.f_ * arrow_length_;
          break;

        case LengthMode::fixed:
        default:
          end += pol_q.f_.normalized() * arrow_length_;
          break;
      }

      double color_value;
      switch (color_mode_) {
        case ColorMode::xCoord:
          color_value = state_q.pos_.x();
          break;
        case ColorMode::yCoord:
          color_value = state_q.pos_.y();
          break;
        case ColorMode::zCoord:
          color_value = state_q.pos_.z();
          break;

        default:
        case ColorMode::norm:
          color_value = pol_q.f_.norm();
          break;
      }

      // color by color_value
      auto color = colormap_.getColorRGBA(color_value);

      renderer_.addArrow(start, end, index, color, &msg);
      index += 6;
    }

    pub_markers_.publish(msg);
  }

 private:
  CoordinateConversion conversion_{std::bind(
      &RMPFieldMarkerPublisher<n>::defaultConversion, std::placeholders::_1)};

  double arrow_length_{1.0};
  LengthMode length_mode_{LengthMode::fixed};
  ColorMode color_mode_{ColorMode::norm};
  ColorMap colormap_;
  ArrowRenderer renderer_;
  ros::Publisher pub_markers_;
};

}  // namespace rmpcpp
#endif  // RMPCPP_ROS_INCLUDE_RMP_MARKER_PUBLISHER_H_
