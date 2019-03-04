#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <Eigen/Dense>


#include "yaml-cpp/yaml.h"

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#define PI 3.1415926

namespace wall {

struct XYMap {
  XYMap(double _x, double _y) : x(_x), y(_y) {}
  XYMap() {}
  double x;
  double y;
  // PointState state;
};
typedef std::vector<XYMap> XYMapVec;

struct LineParam {
  LineParam(double _k, double _b, bool _is_line) 
    : k(_k), 
      b(_b), 
      is_line(_is_line) {}
  LineParam() : is_line(false) {}
  double k;
  double b;
  bool is_line;
};

class WallFollow {
 public:
  WallFollow(ros::NodeHandle n, const std::string add);
  ~WallFollow() {}
  void GetMapCallback(const nav_msgs::OccupancyGrid& map);
  void GetOdometryCallback(const nav_msgs::Odometry& pose);
  void GetScanCallback(const sensor_msgs::LaserScan& scan);
  std::vector<XYMapVec> LineCut(const XYMapVec& vec);
  std::vector<LineParam> LinearFit(const std::vector<XYMapVec>& cut);
  LineParam GetLine(const std::vector<LineParam>& right,
                    const std::vector<LineParam>& left,
                    const Eigen::Vector2d& position);
  void PubStart(const Eigen::Matrix4d& base);
  void PubLine(const LineParam& line);
  void PubCircle(const Eigen::Matrix4d& base_point);
  void PubMap();
 private:
  XYMapVec xy_map;
  XYMapVec laser_points;
  bool map_flag;
  bool laser_flag;
  double limit;
  double angle_limit;
  size_t least_n;
  Eigen::Matrix4d map_transform;

  ros::Publisher wall_path_pub;
  ros::Publisher circle_pub;
  ros::Publisher marker_pub;
  ros::Publisher left_marker_pub;
  ros::Publisher right_marker_pub;
  ros::Publisher laser_marker_pub;
  ros::Publisher start_pub;
};

}