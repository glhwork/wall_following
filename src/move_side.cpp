#include "wall_following/MoveSide.h"

using wall::MoveSide;

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_side");
  ros::NodeHandle n;

  MoveSide move(n);
  ros::Timer pose_timer = n.createTimer(ros::Duration(0.5), &MoveSide::GetPoseCallback, &move);
  ros::Subscriber wall_sub = n.subscribe("wall_accu", 100, &MoveSide::GetWallCallback, &move);
  ros::spin();
  return 0;
}