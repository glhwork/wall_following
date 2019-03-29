#ifndef MOVESIDE_H
#define MOVESIDE_H

#include <iostream>
#include <vector>
#include <string>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "tf/transform_listener.h"

namespace wall {

struct RoboPosi {
  RoboPosi(double a, double b, double c) : x(a), y(b), z(c) {}
  RoboPosi() {}
  double x;
  double y;
  double z;
};  // struct RoboPosi
typedef Eigen::Quaterniond RoboOrie;

struct RoboPose {
  RoboPose(RoboPosi p, RoboOrie o) : position(p), orientation(o) {}
  RoboPose() {}
  RoboPosi position;
  RoboOrie orientation;
};  // struct RoboPose

class MoveSide {
 public:
  MoveSide(ros::NodeHandle n);
  MoveSide() {}
  ~MoveSide() {}
  void GetWallCallback(const nav_msgs::Path& wall);
  void GetPoseCallback(const ros::TimerEvent&);
  void ParamInit();
  void Setup(ros::NodeHandle n);
  RoboPosi FirstGoal(const RoboPosi& inter_point, 
                     const double k_pose, 
                     const double b_pose);
  double FindYaw(const nav_msgs::Path& wall);
  double FindDist(double x1, double y1, double x2, double y2);
  void PubGoal(const geometry_msgs::PoseStamped& pose); 

 private:
  double to_wall_dis;
  RoboPose global_pose;
  bool get_pose;
  bool send_first_goal;

  std::string base_frame_id;
  std::string laser_frame_id;
  std::string map_frame_id;
  
  ros::NodeHandle nh;
  ros::NodeHandle goal_n;
  ros::Publisher first_goal_pub;
  ros::Publisher first_wall_pub;
  ros::Publisher goal_marker_pub;

  nav_msgs::Path show_wall;
	
};  // class MoveSide

}  // namespace wall

#endif
