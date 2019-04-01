#ifndef DETECTION_H
#define DETECTION_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"

#define PI 3.1415926

namespace wall {

struct Point3d {
  Point3d(double x, double y, double z) : x(x), y(y), z(z) {}
  Point3d() : x(0.0), y(0.0), z(0.0) {}
  double x;
  double y;
  double z;
};  // struct Point2d
typedef std::vector<Point3d> Point3dVec;

struct Point2d {
  Point2d(double x, double y) : x(x), y(y) {}
  Point2d() : x(0.0), y(0.0) {}
  double x;
  double y;
};  // struct Point2d
typedef std::vector<Point2d> Point2dVec;

struct PointPolar {
  PointPolar(double r, double theta) : r(r), theta(theta) {}
  PointPolar() : r(0.0), theta(0.0) {}
  double r;
  double theta;
};  // struct PointPolar
typedef std::vector<PointPolar> PointPolarVec;

enum {
  OCCUPIED = 0,
  PASSABLE = 100,
  UNKNOWN = -1,
};  // enum used to judge the passing ability of map point

class Detection {
 public:
  Detection();
  ~Detection() {}
  // initializer of ros publishers
  void Setup();
  // get parameters from launch file
  void ParamInit();
  // callback functions to obtain map msgs and pose from tf tree
  void MapCallback(const nav_msgs::OccupancyGrid& map_msg);
  void PoseCallback(const ros::TimerEvent&);
  // transform map from Cartesian to Polar and sort points w.r.t theta
  void PolarMap();

 private:
  /* PARAMETERS */ 
  // the map difference limit to judge whether updating the map
  int update_map_dif;  
  // frame id
  std::string map_frame_id;
  std::string base_frame_id;
  // minimum limit used to judge when to skip some map points
  double angle_interval;
  
  /* GLOBAL VARIABLES */
  // used to judge whether a map is received
  bool get_map;
  // the map size of the received map msg
  int map_size; 
  // map data in Cartesian coordinates
  Point2dVec map_2d;
  // map data in Polar coordinates
  PointPolarVec map_polar;


  // node handle to get paramters
  ros::NodeHandle nh;


	
};






}  // namespace wall

#endif