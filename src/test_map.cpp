#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"


void GetMap(const nav_msgs::OccupancyGrid& map_msg) {
  double reso = map_msg.info.resolution;
  int width = map_msg.info.width;
  int height = map_msg.info.height;
    
    Eigen::Matrix4d map_msg_transform = Eigen::Matrix4d::Identity();
    map_msg_transform(0,3) = map_msg.info.origin.position.x;
    map_msg_transform(1,3) = map_msg.info.origin.position.y;
    map_msg_transform(2,3) = 0.0;
    Eigen::Quaterniond quat_tmp(map_msg.info.origin.orientation.w,
                                map_msg.info.origin.orientation.x,
                                map_msg.info.origin.orientation.y,
                                map_msg.info.origin.orientation.z);
    map_msg_transform.block(0,0, 3,3) = quat_tmp.toRotationMatrix();
    
    std::cout << "size of map is :" << width * height << std::endl;
    std::cout << "pose of map is : " << std::endl
              << map_msg_transform << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_map");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("map", 100, GetMap);
  ros::spin();
  return 0;
}