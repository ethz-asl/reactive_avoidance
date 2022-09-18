// Dummy cc file that includes headers to make code completion work.
/*#include <rmpcpp/core/geometry_base.h>

*/
#include <rmpcpp/core/policy_value.h>
#include <rmpcpp/geometry/cylindrical_geometry.h>
#include <rmpcpp/geometry/linear_geometry.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <rmpcpp/eval/trapezoidal_integrator.h>

#include <Eigen/Dense>
#include <iostream>

int main(int argc, char *argv[]) {
  // Define some targets and metrics
  Eigen::Vector3d target{2.0, M_PI_2, 0.0};
  Eigen::Vector3d target2{3.0, 0, 0.0};
  Eigen::Matrix3d metric = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d metric2 = Eigen::Vector3d({1.0, 0.0, 0.0}).asDiagonal();

  // Create two independent policies
  rmpcpp::SimpleTargetPolicy<rmpcpp::CylindricalSpace> targetPolicy(
      target, metric, 1.0, 2.0, 0.05);
  rmpcpp::SimpleTargetPolicy<rmpcpp::CylindricalSpace> targetPolicy2(
      target2, metric2, 10.0, 22.0, 0.05);

  // Create two independent geometries for test
  rmpcpp::LinearGeometry<3> geometry;
  rmpcpp::CylindricalGeometry geometry_B;

  // Define some steps
  rmpcpp::State<3> state_q({1.0, M_PI_4, 0.0});
  rmpcpp::State<3> state_B({10.0, M_PI_4, 0.0});

  auto pullvalue = geometry.at(state_q).pull(targetPolicy);
  auto pullvalue_B = geometry_B.at(state_B).pull(targetPolicy2);

  auto result = pullvalue + pullvalue_B;

  std::cout << result.f_ << std::endl;
  std::cout << result.A_ << std::endl;
  return 0;
}
