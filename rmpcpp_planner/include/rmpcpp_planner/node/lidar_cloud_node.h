
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

class LidarCloudNode {
  typedef rmpcpp::LidarRayCudaPolicy<rmpcpp::Space<3>> RayPolicy;
  typedef rmpcpp::SimpleTargetPolicy<rmpcpp::Space<3>> TargetPolicy;

  enum class PointCloudType {
    FullScanUndistorted,
    PartialScan,
    BufferedPartialScans
  };

 public:
  LidarCloudNode()
      : ray_policy_value_B_(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()) {
    params.N_sqrt = 182;
    params.r = 5;
    params.c_softmax_obstacle = 5;
    params.epsilon_damp = 0.001;
    params.eta_damp = 0.2;
    params.v_damp = 20;
    params.eta_rep = 30;
    params.v_rep = 0.1;
    params.metric = true;

    ray_policy_ = std::make_shared<RayPolicy>(params);
    target_policy_ = std::make_shared<TargetPolicy>(
        Eigen::Vector3d(1.0, 1.0, 1.0),
        Eigen::Matrix3d::Identity() * params.metric_goal, params.alpha_goal,
        params.beta_goal, params.gamma_goal);

    server_.setCallback(boost::bind(&LidarCloudNode::callback, this, _1, _2));

    sub_lidar_ =
        nh_.subscribe("points", 1, &LidarCloudNode::pointCloudCallback, this);

    sub_odometry_ =
        nh_.subscribe("odometry", 1, &LidarCloudNode::odometryCallback, this);

    sub_goal_ = nh_.subscribe("goal", 1, &LidarCloudNode::goalCallback, this);

    pub_pose_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
        "trajectory", 1);

    pub_debugcloud_ = nh_.advertise<sensor_msgs::PointCloud2>("debugcloud", 1);

    pub_debugmarkers_ =
        nh_.advertise<visualization_msgs::MarkerArray>("debugmarker", 1);
  }

  void callback(rmpcpp_planner::LidarPlannerConfig& config, uint32_t level) {
    params.alpha_goal = config.alpha_goal;
    params.beta_goal = config.beta_goal;
    params.gamma_goal = config.gamma_goal;

    params.eta_damp = config.eta_damp;
    params.v_damp = config.v_damp;
    params.epsilon_damp = config.epsilon_damp;
    params.metric_goal = config.metric_goal;

    params.eta_rep = config.eta_rep;
    params.v_rep = config.v_rep;
    params.lin_rep = config.lin_rep;
    params.r = config.r;
    params.c_softmax_obstacle = config.c_softmax_obstacle;
    params.metric = config.metric;
    ray_policy_->updateParams(params);
    target_policy_->updateParams(params.alpha_goal, params.beta_goal,
                                 params.gamma_goal);
  }

  void goalCallback(const geometry_msgs::Point& pt) {
    goal_W_ = Eigen::Vector3d(pt.x, pt.y, pt.z);
    target_policy_ = std::make_shared<TargetPolicy>(
        goal_W_, Eigen::Matrix3d::Identity() * params.metric_goal,
        params.alpha_goal, params.beta_goal, params.gamma_goal);
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    const uint pc_size = cloud_msg.get()->width * cloud_msg.get()->height;
    const size_t pc_data_len = cloud_msg.get()->data.size();

    for (auto field : cloud_msg.get()->fields) {
      std::cout << field.name << std::endl;
      std::cout << field.datatype << std::endl;
      std::cout << field.offset << std::endl;
    }

    RayPolicy::LidarData pc_data;
    pc_data.data = cloud_msg.get()->data.data();
    pc_data.stride = cloud_msg.get()->point_step;
    pc_data.n_points = pc_size;
    pc_data.size = pc_data_len;

    ray_policy_->updateLidarData(pc_data);
    auto start = std::chrono::steady_clock::now();

    // in body frame we're always at 0,0,0 ...
    auto state_B = RayPolicy::PState({0, 0, 0}, v_B_);
    auto pol_value = ray_policy_->evaluateAt(state_B);

    new (&ray_policy_value_B_) RayPolicy::PValue(pol_value.f_, pol_value.A_);

    auto end = std::chrono::steady_clock::now();
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Elapsed(us)=" << duration.count() << std::endl;

    // publish debug cloud.
    sensor_msgs::PointCloud2 debug_msg;
    debug_msg.header = cloud_msg->header;
    debug_msg.point_step = sizeof(LidarRayDebugData);
    debug_msg.is_dense = true;
    debug_msg.height = 1;
    debug_msg.width = pc_data.n_points;
    debug_msg.row_step = debug_msg.point_step * pc_data.n_points;

    size_t offset_ctr = 0;
    auto float_field_lambda = [&](std::string name) {
      sensor_msgs::PointField field;
      field.name = name;
      field.offset = offset_ctr;
      field.datatype = sensor_msgs::PointField::FLOAT32;
      field.count = 1;
      offset_ctr += 4;
      return field;
    };

    debug_msg.fields.push_back(float_field_lambda("x"));
    debug_msg.fields.push_back(float_field_lambda("y"));
    debug_msg.fields.push_back(float_field_lambda("z"));
    debug_msg.fields.push_back(float_field_lambda("f"));
    debug_msg.fields.push_back(float_field_lambda("a"));
    debug_msg.fields.push_back(float_field_lambda("fa"));
    debug_msg.fields.push_back(float_field_lambda("damp"));
    debug_msg.fields.push_back(float_field_lambda("rep"));
    debug_msg.data.resize(sizeof(LidarRayDebugData) * pc_data.n_points);
    ray_policy_->getDebugData((LidarRayDebugData*)debug_msg.data.data(),
                              pc_data.n_points);
    pub_debugcloud_.publish(debug_msg);
  }

  void odometryCallback(const nav_msgs::Odometry& odom) {
    tf::poseMsgToEigen(odom.pose.pose, T_W_B_);
    tf::vectorMsgToEigen(odom.twist.twist.linear, v_B_);
    v_W_ = T_W_B_.rotation() * v_B_;

    body_geometry_.setRotation(T_W_B_.rotation().transpose());
    // execute policy at odom rate
    auto state_W = RayPolicy::PState(T_W_B_.translation(), v_W_);
    std::cout << state_W.pos_.transpose() << " " << v_W_.transpose()
              << std::endl;
    auto target_policy_value_W = target_policy_->evaluateAt(state_W);
    auto ray_policy_value_W =
        body_geometry_.at(state_W).pull(ray_policy_value_B_);

    auto full_result = ray_policy_value_W + target_policy_value_W;

    double dt = 0.1;

    rmpcpp::LinearGeometry<3> geometry;

    trajectory_msgs::MultiDOFJointTrajectory msg;
    msg.header.stamp = odom.header.stamp;
    integrator.resetTo(T_W_B_.translation(), v_W_);
    for (int i = 0; i < 2; i++) {
      integrator.forwardIntegrateFixed(full_result, geometry, dt);
      Eigen::Vector3d int_pos = integrator.getPos();
      Eigen::Vector3d int_vel = integrator.getVel();
      Eigen::Vector3d int_acc = integrator.getAcc();

      trajectory_msgs::MultiDOFJointTrajectoryPoint pt;

      geometry_msgs::Twist acc;
      tf::vectorEigenToMsg(int_acc, acc.linear);
      pt.accelerations.push_back(acc);

      geometry_msgs::Twist vel_pt;
      tf::vectorEigenToMsg(int_vel, vel_pt.linear);
      pt.velocities.push_back(vel_pt);

      geometry_msgs::Transform tran;
      tran.rotation.w = 1;
      tran.rotation.x = 0;
      tran.rotation.y = 0;
      tran.rotation.z = 0;
      tf::vectorEigenToMsg(int_pos, tran.translation);
      pt.transforms.push_back(tran);

      pt.time_from_start = ros::Duration(i * dt);
      msg.points.push_back(pt);
    }

    pub_pose_.publish(msg);

    // build marker debug message
    rmpcpp::LidarPolicyDebugData debugdata;
    ray_policy_->getPolicyDebugData(&debugdata);

    // these should be in world frame.
    std::cout << full_result.f_ << std::endl;
    Eigen::Vector3d full_policy = full_result.f_;
    Eigen::Vector3d target_policy = target_policy_value_W.f_;
    Eigen::Vector3d ray_policy = ray_policy_value_W.f_;

    visualization_msgs::MarkerArray msg_markers;

    msg_markers.markers.push_back(
        vectorToMarker(full_policy, "f_full", odom.header.frame_id,
                       T_W_B_.translation(), {1.0, 1.0, 0.0, 1.0}));

    msg_markers.markers.push_back(
        vectorToMarker(target_policy, "f_target", odom.header.frame_id,
                       T_W_B_.translation(), {0.0, 1.0, 0.0, 0.5}));
    msg_markers.markers.push_back(
        vectorToMarker(ray_policy, "f_ray", odom.header.frame_id,
                       T_W_B_.translation(), {0.0, 0.0, 1.0, 0.5}));
    msg_markers.markers.push_back(positionToMarker("pos", odom.header.frame_id,
                                                   T_W_B_.translation(),
                                                   {0.0, 0.0, 1.0, 0.5}));
    msg_markers.markers.push_back(positionToMarker(
        "goal", odom.header.frame_id, goal_W_, {0.0, 1.0, 0.0, 0.5}));
    pub_debugmarkers_.publish(msg_markers);
  }

  visualization_msgs::Marker vectorToMarker(Eigen::Vector3d vec,
                                            std::string name, std::string frame,
                                            Eigen::Vector3d pos,
                                            Eigen::Vector4d color) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = color.w();  // Don't forget to set the alpha!
    marker.color.r = color.x();
    marker.color.g = color.y();
    marker.color.b = color.z();
    marker.points.resize(2);
    Eigen::Vector3d vec_end = pos + vec;
    marker.points[0].x = pos.x();
    marker.points[0].y = pos.y();
    marker.points[0].z = pos.z();
    marker.points[1].x = vec_end.x();
    marker.points[1].y = vec_end.y();
    marker.points[1].z = vec_end.z();
    return marker;
  }

  visualization_msgs::Marker positionToMarker(std::string name,
                                              std::string frame,
                                              Eigen::Vector3d pos,
                                              Eigen::Vector4d color) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = pos.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = color.w();  // Don't forget to set the alpha!
    marker.color.r = color.x();
    marker.color.g = color.y();
    marker.color.b = color.z();
    return marker;
  }

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
      integrator;
  dynamic_reconfigure::Server<rmpcpp_planner::LidarPlannerConfig> server_;
  bool int_reset = false;
};

}  // namespace rmpcpp

#endif  // RMPCPP_PLANNER_LIDAR_CLOUD_NODE_H
