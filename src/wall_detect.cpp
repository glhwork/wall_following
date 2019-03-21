#include "wall_following/WallDetect.h" 

using wall::WallDetect;

int main(int argc, char** argv) {
  ros::init(argc, argv, "wall_detect");
  ros::NodeHandle n;
  WallDetect detect;

  ros::Timer pose_timer = n.createTimer(ros::Duration(0.5), &WallDetect::GetPoseCallback, &detect);
  ros::Subscriber scan_sub = n.subscribe("scan", 100, &WallDetect::GetScanCallback, &detect);
  ros::spin();

  return 0;
}
