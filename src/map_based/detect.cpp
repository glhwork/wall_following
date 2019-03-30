#include "wall_following/map_based/Detection.h"

using wall::Detection;

int main(int argc, char** argv) {
  ros::init(argc, argv, "detect");
  ros::NodeHandle n;

  Detection detect;
  ros::spin();
  return 0;
}

