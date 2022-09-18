#include <rmpcpp/core/space.h>
#include <rmpcpp/geometry/cylindrical_geometry.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <rmpcpp/util/vector_range.h>
#include <rmpcpp_ros/arrow_renderer.h>
#include <rmpcpp_ros/rmp_field_marker_publisher.h>

#include <iostream>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "dummy");
  ros::NodeHandle nh;

  // Define base objects for RMP
  using Geometry3 = rmpcpp::CylindricalGeometry;
  Geometry3 geometry_3;

  // Define simple target policy and add to container
  rmpcpp::SimpleTargetPolicy<rmpcpp::CylindricalSpace> target_policy{
      {0.5, -0.5 * 3.14159, 0}};
  Geometry3::MatrixX A_target(Geometry3::MatrixX::Zero());
  A_target(0, 0) = 1.0;
  A_target(1, 1) = 1.0;
  target_policy.setA(A_target);

  // Publish on a vector range
  rmpcpp::RMPFieldMarkerPublisher<Geometry3::D> publisher(nh, "field");
  publisher.getColorMap().scale(0.99, 1.0);
  publisher.setLengthFactor(0.05);
  rmpcpp::VectorRange<Geometry3::VectorX> range({0, 0, 0}, {2, 2 * M_PI, 0},
                                                {0.1, 0.1, 0.5});

  publisher.publish(&geometry_3, &target_policy, range);

  ros::spin();
  return 0;
}
