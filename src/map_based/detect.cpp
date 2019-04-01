#include "wall_following/map_based/Detection.h"

using wall::Detection;

int main(int argc, char** argv) {
  ros::init(argc, argv, "detect");
  ros::NodeHandle n;

  Detection detect;
  ros::Subscriber map_sub = n.subscribe("map", 100, &Detection::MapCallback, &detect);
  ros::spin();
  return 0;
}

