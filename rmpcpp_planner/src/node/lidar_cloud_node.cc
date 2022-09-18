#include <rmpcpp_planner/node/lidar_cloud_node.h>

#include <iostream>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "rmpcpp_lidar_node");
  rmpcpp::LidarCloudNode node;
  ros::spin();
}