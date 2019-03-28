#include "wall_following/WallDetect.h" 

using wall::WallDetect;

int main(int argc, char** argv) {
  ros::init(argc, argv, "wall_detect");
  ros::NodeHandle n;
  WallDetect detect(n);

  ros::Timer pose_timer = n.createTimer(ros::Duration(0.5), &WallDetect::FindWallCallback, &detect);
  ros::Subscriber scan_sub = n.subscribe("scan", 100, &WallDetect::GetScanCallback, &detect);
  ros::Subscriber map_sub = n.subscribe("map", 100, &WallDetect::GetMapCallback, &detect);
  ros::spin();

  return 0;
}
