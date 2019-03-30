#ifndef DETECTION_H
#define DETECTION_H

#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

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
};
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


	
};






}  // namespace wall

#endif