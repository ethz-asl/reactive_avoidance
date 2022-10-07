
#ifndef RMPCPP_PLANNER_LIDAR_CLOUD_NODE_H
#define RMPCPP_PLANNER_LIDAR_CLOUD_NODE_H
#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <rmpcpp/eval/trapezoidal_integrator.h>
#include <rmpcpp/geometry/linear_geometry.h>
#include <rmpcpp/geometry/rotated_geometry_3d.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <rmpcpp_planner/LidarPlannerConfig.h>
#include <rmpcpp_planner/policies/lidarray_CUDA.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>

namespace rmpcpp {

/**
 * Implementation of a Lidar-planning ros node
 *  - subscribes to :
 *      - Lidar Pointcloud
 *      - Odometry ( needs velocity)
 *      - A desired goal location
 *
 *  - publishes:
 *      - pose as next input for robot controller (planning result)
 *      - debug markers that visualize policy results
 *      - debug pointclouds that visualize per-point policy strengths
 *
 */
class LidarCloudNode {
  typedef rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<3>> RayPolicy;
  typedef rmpcpp::SimpleTargetPolicy<rmpcpp::Space<3>> TargetPolicy;

  enum class PointCloudType {
    FullScanUndistorted,
    PartialScan,  // not implemented
    BufferedPartialScans // not implemented
  };

 public:
  LidarCloudNode();

  void configCallback(rmpcpp_planner::LidarPlannerConfig& config,
                      uint32_t level);

  void goalCallback(const geometry_msgs::Point& pt);

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

  void odometryCallback(const nav_msgs::Odometry& odom);

  visualization_msgs::Marker vectorToMarker(Eigen::Vector3d vec,
                                            std::string name, std::string frame,
                                            Eigen::Vector3d pos,
                                            Eigen::Vector4d color);

  visualization_msgs::Marker positionToMarker(std::string name,
                                              std::string frame,
                                              Eigen::Vector3d pos,
                                              Eigen::Vector4d color);

  Eigen::Affine3d T_W_B_;
  Eigen::Vector3d v_B_;
  Eigen::Vector3d v_W_;
  Eigen::Vector3d goal_W_;
  RayPolicy::PValue ray_policy_value_B_;

  rmpcpp::RotatedGeometry3d body_geometry_;
  std::shared_ptr<RayPolicy> ray_policy_;
  std::shared_ptr<TargetPolicy> target_policy_;
  ros::Subscriber sub_lidar_, sub_odometry_, sub_goal_;
  ros::Publisher pub_pose_, pub_debugcloud_, pub_debugmarkers_;
  ros::NodeHandle nh_;

  RaycastingCudaPolicyParameters params;

  rmpcpp::TrapezoidalIntegrator<TargetPolicy, rmpcpp::LinearGeometry<3>>
      integrator_;
  dynamic_reconfigure::Server<rmpcpp_planner::LidarPlannerConfig> server_;
  bool int_reset = false;
};

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_LIDAR_CLOUD_NODE_H
