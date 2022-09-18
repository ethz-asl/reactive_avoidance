//
// Created by mpantic on 06.09.22.
//

#ifndef RMPCPP_ROTATEDGEOMETRY3D_H
#define RMPCPP_ROTATEDGEOMETRY3D_H
#include <rmpcpp/core/geometry_base.h>
namespace rmpcpp {
class RotatedGeometry3d : public GeometryBase<3, 3> {
 public:
  // type alias for readability.
  using base = GeometryBase<3, 3>;
  using StateX = typename base::StateX;
  using StateQ = typename base::StateQ;

  inline void setRotation(const Eigen::Matrix3d &rotation) {
    R_x_q_ = rotation;
  }

  /**
   * Return jacobian. simply the rotation matrix;
   */
  inline virtual typename base::J_phi J(const StateX &state) const { return R_x_q_; }

  inline virtual StateX convertToX(const StateQ &state_q) const {
    return {R_x_q_ * state_q.pos_, R_x_q_ * state_q.vel_};
  }

  inline virtual StateQ convertToQ(const StateX &state_x) const {
    return {R_x_q_.transpose() * state_x.pos_,
            R_x_q_.transpose() * state_x.vel_};
  }

 private:
  Eigen::Matrix3d R_x_q_{Eigen::Matrix3d::Identity()};
};

}  // namespace rmpcpp

#endif  // RMPCPP_ROTATEDGEOMETRY3D_H
