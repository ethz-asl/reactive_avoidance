#ifndef RMPCPP_PLANNER_TRAJECTORY_RMP_H
#define RMPCPP_PLANNER_TRAJECTORY_RMP_H

#include <Eigen/Dense>
#include <memory>

#include "rmpcpp/core/space.h"

namespace rmpcpp {
/*
 * Struct that holds a discretized and integrated
 *  point in a RMP trajectory.
 */
template <class Space>
struct TrajectoryPointRMP {
  using Vector = Eigen::Matrix<double, Space::dim, 1>;
  Vector position;
  Vector velocity;
  Vector acceleration;
  double cumulative_length = 0.0;  // Cumulative length of this trajectory

  static std::string getHeaderFormat();
  std::string format() const;
};

/*
 * Class that holds a full trajectory (vecetor of points)
 */
template <class Space>
class TrajectoryRMP {
  using Vector = Eigen::Matrix<double, Space::dim, 1>;

 public:
  explicit TrajectoryRMP(const Vector& pos_origin,
                         const Vector& v_start = Vector::Zero(),
                         const Vector& a_start = Vector::Zero());

  const TrajectoryPointRMP<Space>& start() const;
  const TrajectoryPointRMP<Space>& current() const;

  void addPoint(const Vector& p, const Vector& v, const Vector& a);

  int getSegmentCount() const;
  double getSmoothness() const;
  double getLength() const;

  inline const TrajectoryPointRMP<Space> operator[](int i) const {
    return trajectory_data_[i];
  };
  inline TrajectoryPointRMP<Space>& operator[](int i) {
    return trajectory_data_[i];
  };

  void writeToStream(std::ofstream& file) const;

 private:
  std::vector<TrajectoryPointRMP<Space>> trajectory_data_;
};

}  // namespace rmpcpp

// explicit instantation
template class rmpcpp::TrajectoryRMP<rmpcpp::Space<3>>;

#endif  // RMPCPP_PLANNER_TRAJECTORY_RMP_H
