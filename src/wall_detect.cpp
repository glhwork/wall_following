#include "wall_following/WallDetect.h" 

using wall::WallDetect;

int main(int argc, char** argv) {
  ros::init(argc, argv, "wall_follow");
  ros::NodeHandle n;
  std::string add = "/home/glh/slam_ws/src/wall_following/data/config.yaml";
  WallDetect follow;
  ros::Subscriber map_sub = n.subscribe("map", 100, &WallDetect::GetMapCallback, &follow);
  ros::Subscriber odom_sub = n.subscribe("odom", 100, &WallDetect::GetOdometryCallback, &follow);
  ros::Subscriber scan_sub = n.subscribe("scan", 100, &WallDetect::GetScanCallback, &follow);
  ros::spin();

  return 0;
}
