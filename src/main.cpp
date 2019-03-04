#include "wall_following/WallFollow.h" 

using wall::WallFollow;

int main(int argc, char** argv) {
  ros::init(argc, argv, "wall_follow");
  ros::NodeHandle n;
  std::string add = "/home/george/slam_ws/src/wall_following/data/config.yaml";
  WallFollow follow(n, add);
  ros::Subscriber map_sub = n.subscribe("map", 100, &WallFollow::GetMapCallback, &follow);
  ros::Subscriber odom_sub = n.subscribe("odom", 100, &WallFollow::GetOdometryCallback, &follow);
  ros::Subscriber scan_sub = n.subscribe("scan", 100, &WallFollow::GetScanCallback, &follow);
  ros::spin();

  return 0;
}
