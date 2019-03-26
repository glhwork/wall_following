#ifndef WALLDETECT_H
#define WALLDETECT_H

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include "yaml-cpp/yaml.h"

#include "ros/ros.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "sensor_msgs/LaserScan.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "tf/transform_listener.h"

#include "geometry_msgs/PoseStamped.h"


#define PI 3.1415926

// #define OCCUPIED 0
// #define PASSABLE 100
// #define UNKNOWN -1

namespace wall {

enum {
  OCCUPIED = 0,
  PASSABLE = 100,
  UNKNOWN = -1,
};

struct XYMap {
  XYMap(double _x, double _y) : x(_x), y(_y) {}
  XYMap() {}
  double x;
  double y;
};  // struct XYMap
typedef std::vector<XYMap> XYMapVec;

struct LineParam {
  LineParam(double _k, double _b, bool _is_line) 
    : k(_k), b(_b), is_line(_is_line) {}
  LineParam() : is_line(false) {}
  double k;
  double b;
  bool is_line;
};  // struct LineParam

class WallDetect {
 public:
  WallDetect(ros::NodeHandle n);
  WallDetect() {}
  ~WallDetect() {}
  void GetMapCallback(const nav_msgs::OccupancyGrid& map_msg);
  void FindWallCallback(const ros::TimerEvent&);
  void GetScanCallback(const sensor_msgs::LaserScan& scan);
  void ParamInit();
  void Setup(ros::NodeHandle n);
  void Loop();
  std::vector<XYMapVec> LineCut(const XYMapVec& vec);
  std::vector<LineParam> LinearFit(const std::vector<XYMapVec>& cut);
  LineParam GetLine(const std::vector<LineParam>& param_vec,
                    const Eigen::Vector2d& position);
  LineParam WallCalibrate(LineParam line);
  void PubStart(const Eigen::Matrix4d& base);
  void PubWall(const LineParam& line);
  void PubAccuWall(const XYMapVec& vec, const LineParam& line);
  void PubPosition(const Eigen::Vector2d& position);
  void PubLimit(const Eigen::Vector2d& position);
  void PubMap();
 private:
  XYMapVec map_2d;
  XYMapVec laser_points;
  int map_size;
  bool get_map;
  bool get_laser;
  double limit;
  double angle_limit;
  int least_n;
  std::string laser_frame_id;
  std::string base_frame_id;
  std::string map_frame_id;
  int update_map_dif;
  double to_line_limit;

  // attention! if multiple nodes initial node handles with ("~")
  ros::NodeHandle nh;
  ros::NodeHandle n;
  ros::Publisher wall_line_pub;
  ros::Publisher wall_accu_pub;
  ros::Publisher limit_pub;
  ros::Publisher map_marker_pub;
  ros::Publisher posi_marker_pub;
  ros::Publisher start_pub;

  std::vector<double> angle_vec;
  std::vector<double> value_vec;
};  // class WallDetect

}  // namespace wall

#endif
