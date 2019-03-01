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

using Eigen::ComputeThinU;
using Eigen::ComputeThinV;

// enum PointState {
//   OCCUPIED,
//   FREE,
//   UNKNOWN;
// };

struct XYMap {
  XYMap(double _x, double _y) : x(_x), y(_y) {}
  XYMap() {}
  double x;
  double y;
  // PointState state;
};
typedef std::vector<XYMap> XYMapVec;

class WallFollow {
 public:
  WallFollow(ros::NodeHandle n, const std::string add);
  ~WallFollow() {}
  void GetMapCallback(const nav_msgs::OccupancyGrid& map);
  void GetOdometryCallback(const nav_msgs::Odometry& pose);
  void GetScanCallback(const sensor_msgs::LaserScan& scan);
  XYMapVec LineCut(const XYMapVec& vec);
  Eigen::Vector2d LinearFitting(const XYMapVec& local_map);
  Eigen::Vector2d GetLine(const Eigen::Vector2d& right,
                          const Eigen::Vector2d& left,
                          const Eigen::Vector2d& position);
  void PubLine(const Eigen::VectorXd& result,
               const Eigen::Matrix4d& base_point);
  void PubCircle(const Eigen::Matrix4d& base_point);
  void PubMap();
 private:
  XYMapVec xy_map;
  XYMapVec laser_points;
  bool map_flag;
  bool laser_flag;
  double limit;
  Eigen::Matrix4d map_transform;

  ros::Publisher line_pub;
  ros::Publisher circle_pub;
  ros::Publisher marker_pub;
  ros::Publisher left_marker_pub;
  ros::Publisher right_marker_pub;
  ros::Publisher laser_marker_pub;
};

WallFollow::WallFollow(ros::NodeHandle n, const std::string add) {
  YAML::Node config = YAML::LoadFile(add);
  xy_map.clear(); 
  map_flag = false; 
  laser_flag = false;
  limit = config["limit"].as<double>();
  map_transform = Eigen::Matrix4d::Identity();
  line_pub = n.advertise<nav_msgs::Path>("path", 100);
  circle_pub = n.advertise<nav_msgs::Path>("circle", 100);
  marker_pub = n.advertise<visualization_msgs::Marker>("map_points", 100);
  left_marker_pub = n.advertise<visualization_msgs::Marker>("left_points", 100);
  right_marker_pub = n.advertise<visualization_msgs::Marker>("right_points", 100);
  laser_marker_pub = n.advertise<visualization_msgs::Marker>("laser_points", 100);
}

void WallFollow::GetMapCallback(const nav_msgs::OccupancyGrid& map) {
  double reso = map.info.resolution;
  int width = map.info.width;
  int height = map.info.height;
  map_transform(0,3) = map.info.origin.position.x;
  map_transform(1,3) = map.info.origin.position.y;

  for (size_t i = 0; i < width; i++) {
    for (size_t j = 0; j < height; j++) {
      //int n = i * width + j;
      int n = i + j * height;
      if (100 == map.data[n]) {
        Eigen::Vector4d v_tmp;
        v_tmp << (i + 0.5) * reso, (j + 0.5) * reso, 0, 1;
        Eigen::Vector4d v = map_transform * v_tmp;

        XYMap xy_map_tmp;
        xy_map_tmp.x = v(0);
        xy_map_tmp.y = v(1);
        xy_map.push_back(xy_map_tmp);
      }
    }
  }
  
  map_flag = true;

}

void WallFollow::GetOdometryCallback(const nav_msgs::Odometry& pose) {

  if (map_flag && laser_flag) {
    double qua_x = pose.pose.pose.orientation.x;
    double qua_y = pose.pose.pose.orientation.y;
    double qua_z = pose.pose.pose.orientation.z;
    double qua_w = pose.pose.pose.orientation.w;
    
    Eigen::Quaternion<double> q(qua_w, qua_x, qua_y, qua_z);
    Eigen::Matrix3d rot = q.matrix();

    // std::cout << "===============" << std::endl;
    // std::cout << "quater is : " << qua_x << "  " 
    //                             << qua_y << "  " 
    //                             << qua_z << "  " 
    //                             << qua_w << std::endl;
    // std::cout << "rotation matrix : " << std::endl
    //           << rot << std::endl;

    Eigen::Matrix4d base_point = Eigen::Matrix4d::Zero();
    base_point.block(0, 0, 3, 3) = rot;
    base_point(0, 3) = pose.pose.pose.position.x;
    base_point(1, 3) = pose.pose.pose.position.y;
    base_point(2, 3) = 0;
    base_point(3, 3) = 1.0;
    
    XYMapVec laser_points_base;
    for (size_t i = 0; i < laser_points.size(); i++) {
      Eigen::Vector4d v_tmp;
      v_tmp << laser_points[i].x, laser_points[i].y, 0, 1;
      Eigen::Vector4d v = base_point * v_tmp;
      laser_points_base.push_back(XYMap(v(0), v(1)));
    }
    //=============pub laser points start================
    visualization_msgs::Marker laser_marker;

    laser_marker.header.stamp = ros::Time::now();
    laser_marker.header.frame_id = "map";
    laser_marker.ns = "laser_marker";
    laser_marker.id = 3;
    laser_marker.type = visualization_msgs::Marker::POINTS;
    laser_marker.action = visualization_msgs::Marker::ADD;

    laser_marker.pose.orientation.w = 1.0;
    laser_marker.scale.x = 0.1;
    laser_marker.scale.y = 0.1;
    laser_marker.scale.z = 0.1;

    laser_marker.color.b = 1.0;
    laser_marker.color.a = 1.0;

    for (size_t i = 0; i < laser_points_base.size(); i++) {
      geometry_msgs::Point p;
      p.x = laser_points_base[i].x;
      p.y = laser_points_base[i].y;
      p.z = 0.0;
      laser_marker.points.push_back(p);
    }
    //=============pub laser points end================

    Eigen::Matrix4d car_point = base_point.inverse();
    
    XYMapVec local_map_right;
    XYMapVec local_map_left; 
    Eigen::Vector4d t(0.0, 0.0, 0.0, 1.0);

    // ===================================
    visualization_msgs::Marker left_marker;
    visualization_msgs::Marker right_marker;

    left_marker.header.stamp = 
    right_marker.header.stamp = ros::Time::now();
    left_marker.header.frame_id = 
    right_marker.header.frame_id = "map";
    left_marker.ns = 
    right_marker.ns = "map_marker";
    left_marker.id = 1;
    right_marker.id = 2;
    left_marker.type = 
    right_marker.type = visualization_msgs::Marker::POINTS;
    left_marker.action = 
    right_marker.action = visualization_msgs::Marker::ADD;

    left_marker.pose.orientation.w = 
    right_marker.pose.orientation.w = 1.0;
    left_marker.scale.x = 
    right_marker.scale.x = 0.1;
    left_marker.scale.y =
    right_marker.scale.y = 0.1;
    left_marker.scale.z = 
    right_marker.scale.z = 0.1;

    left_marker.color.r = 1.0;
    left_marker.color.a = 1.0;
    right_marker.color.b = 1.0;
    right_marker.color.a = 1.0;
    // ===================================


    for (size_t i = 0; i < xy_map.size(); i++) {
      t(0) = xy_map[i].x;
      t(1) = xy_map[i].y;

      Eigen::Vector4d t_transformed =  car_point * t;
      double dis = sqrt(pow(t_transformed(0),2) + pow(t_transformed(1),2));

      
      if (t_transformed(1) < 0 && dis <= limit) {
        // XYMap local_map_tmp;
        // local_map_tmp.x = t_transformed(0);
        // local_map_tmp.y = t_transformed(1);
        // local_map_right.push_back(local_map_tmp);

        XYMap local_map_tmp;
        local_map_tmp.x = xy_map[i].x;
        local_map_tmp.y = xy_map[i].y;
        local_map_right.push_back(local_map_tmp);

        geometry_msgs::Point p;
        p.x = xy_map[i].x;
        p.y = xy_map[i].y;
        p.z = 0;

        right_marker.points.push_back(p);
      } else if (t_transformed(1) > 0 && dis <= limit) {
        // XYMap local_map_tmp;
        // local_map_tmp.x = t_transformed(0);
        // local_map_tmp.y = t_transformed(1);
        // local_map_left.push_back(local_map_tmp);

        XYMap local_map_tmp;
        local_map_tmp.x = xy_map[i].x;
        local_map_tmp.y = xy_map[i].y;
        local_map_left.push_back(local_map_tmp);

        geometry_msgs::Point p;
        p.x = xy_map[i].x;
        p.y = xy_map[i].y;
        p.z = 0;

        left_marker.points.push_back(p);
      }
    }

    Eigen::Vector2d result_right = LinearFitting(local_map_right);
    Eigen::Vector2d result_left = LinearFitting(local_map_left);

    Eigen::Vector2d position;
    position << pose.pose.pose.position.x, 
                pose.pose.pose.position.y;

    Eigen::Vector2d result = GetLine(result_right, result_left, position);
    
    bool right_flag = !(result_right(0) == 0 && result_right(1) == 0);
    bool left_flag = !(result_left(0) == 0 && result_left(1) == 0);

    PubLine(result, car_point);
    PubCircle(base_point);
    PubMap();
    left_marker_pub.publish(left_marker);
    right_marker_pub.publish(right_marker);
    laser_marker_pub.publish(laser_marker); 
  }

}

// void WallFollow::GetOdometryCallback(const nav_msgs::Odometry& pose) {

//   if (map_flag && laser_flag) {
//     Eigen::Vector4d quat(pose.pose.pose.orientation.x,
//                          pose.pose.pose.orientation.y,
//                          pose.pose.pose.orientation.z,
//                          pose.pose.pose.orientation.w);
    
//     Eigen::Quaternion<double> q(quat(3), quat(0), quat(1), quat(2));
//     Eigen::Matrix3d rot = q.matrix();

//     // std::cout << "===============" << std::endl;
//     // std::cout << "quater is : " << qua_x << "  " 
//     //                             << qua_y << "  " 
//     //                             << qua_z << "  " 
//     //                             << qua_w << std::endl;
//     // std::cout << "rotation matrix : " << std::endl
//     //           << rot << std::endl;

//     Eigen::Matrix4d base_point = Eigen::Matrix4d::Identity();
//     base_point.block(0, 0, 3, 3) = rot;
//     base_point(0, 3) = pose.pose.pose.position.x;
//     base_point(1, 3) = pose.pose.pose.position.y;
//     base_point(2, 3) = 0;
    
//     XYMapVec laser_points_base_left;
//     XYMapVec laser_points_base_right;
//     for (size_t i = 0; i < laser_points.size(); i++) {

//       if (laser_points[i].y > 0) {

//         Eigen::Vector4d v_tmp;
//         v_tmp << laser_points[i].x, laser_points[i].y, 0, 1;
//         Eigen::Vector4d v = base_point * v_tmp;
//         laser_points_base_left.push_back(XYMap(v(0), v(1)));

//       } else if (laser_points[i].y < 0) {

//         Eigen::Vector4d v_tmp;
//         v_tmp << laser_points[i].x, laser_points[i].y, 0, 1;
//         Eigen::Vector4d v = base_point * v_tmp;
//         laser_points_base_right.push_back(XYMap(v(0), v(1)));

//       }
//     }
    
//     std::cout << "base left size : " << laser_points_base_left.size() << std::endl;
//     std::cout << "base right size : " << laser_points_base_right.size() << std::endl; 
//     XYMapVec laser_left_cut = LineCut(laser_points_base_left);
//     XYMapVec laser_right_cut = LineCut(laser_points_base_right);
    
//     std::cout << "finish line cut" << std::endl;
//     Eigen::Vector2d line_left, line_right;
//     if (laser_left_cut.size() > 0) {
//       line_left = LinearFitting(laser_left_cut);
//     } else {
//       line_left << 0, 0;
//     }
//     if (laser_right_cut.size() > 0) {
//       line_right = LinearFitting(laser_right_cut);
//     } else {
//       line_right << 0, 0;
//     } // attention! k = 0 and b = 0 can also represent a straight line
//     std::cout << "finish linear fitting" << std::endl;
//     Eigen::Vector2d position(pose.pose.pose.position.x,
//                              pose.pose.pose.position.y);
//     Eigen::Vector2d line = GetLine(line_left, line_right, position);

//     Eigen::Matrix4d car_point = base_point.inverse();
    
//     std::cout << "start publish line and circle" << std::endl;
//     PubLine(line, car_point);
//     PubCircle(base_point);
//     PubMap();
//   }

// }

void WallFollow::GetScanCallback(const sensor_msgs::LaserScan& scan) {
  
  // get coordinates of laser points w.r.t base_link frame
  laser_points.clear();
  for (size_t i = 0; i < scan.ranges.size(); i++) {

    double range = scan.ranges[i];
    if (range <= scan.range_max && range <= limit) {

      XYMap laser_points_tmp;
      laser_points_tmp.x = range * cos(i * 0.0175);
      laser_points_tmp.y = range * sin(i * 0.0175);
      laser_points.push_back(laser_points_tmp);

    }

  }
  laser_flag = true; 

}

void WallFollow::PubMap() {
  visualization_msgs::Marker map_marker;

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "map";
  marker.ns = "map_marker";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 1.0;
  marker.color.a = 1;

  
  for (size_t i = 0; i < xy_map.size(); i++) {    
    
    geometry_msgs::Point p;

    p.x = xy_map[i].x;
    p.y = xy_map[i].y;
    p.z = 0.0;

    marker.points.push_back(p);
  }
  marker_pub.publish(marker);

}

XYMapVec WallFollow::LineCut(const XYMapVec& vec) {

  XYMapVec result_vec;
  result_vec.clear();
  std::cout << vec.size() << std::endl;
  if (vec.size() > 2) {
    for (size_t i = 0; i < (vec.size() - 2); i++) {
      Eigen::Vector2d v1(vec[i+1].x - vec[i].x,
                         vec[i+1].y - vec[i].y);
      Eigen::Vector2d v2(vec[i+2].x - vec[i+1].x,
                         vec[i+2].y - vec[i+1].y);
      double nv1 = sqrt(pow(v1(0), 2) + pow(v1(1), 2));
      double nv2 = sqrt(pow(v2(0), 2) + pow(v2(1), 2));
      double angle_rad = fabs(v1.dot(v2) / (nv1 * nv2));
    
      std::cout << angle_rad << std::endl;
      if (angle_rad <= 0.175) {
        XYMap result_tmp;
        result_tmp.x = vec[i].x;
        result_tmp.y = vec[i].y;
        result_vec.push_back(result_tmp);
      } else if (result_vec.size() > 10) {
        break;
      }
    }
  }
  std::cout << "finish cut in function" << std::endl;
  return result_vec;

}


void WallFollow::PubCircle(const Eigen::Matrix4d& base_point) {
  int circle_reso = 20;
  nav_msgs::Path circle;
  circle.header.frame_id = "map";
  circle.header.stamp = ros::Time::now();

  for (size_t i = 0; i <= circle_reso; i++) {
    geometry_msgs::PoseStamped p;
    double angle = i * (2 * PI) / circle_reso;
    Eigen::Vector4d vec_tmp;
    vec_tmp(0) = limit * cos(angle);
    vec_tmp(1) = limit * sin(angle);
    vec_tmp(2) = 0;
    vec_tmp(3) = 1;

    Eigen::Vector4d vec = base_point * vec_tmp;
    p.pose.position.x = vec(0);
    p.pose.position.y = vec(1);
    p.pose.position.z = 0;

    circle.poses.push_back(p);
  }
  circle_pub.publish(circle);
}

void WallFollow::PubLine(const Eigen::VectorXd& result,
                         const Eigen::Matrix4d& car_point) {
  
  // Eigen::Vector4d p_tmp;

  // p_tmp(0) = 0;
  // p_tmp(1) = result(0) * p_tmp(0) + result(1);
  // p_tmp(2) = 0;
  // p_tmp(3) = 1;
  // Eigen::Vector4d start = (car_point * map_transform).inverse() * p_tmp;

  // p_tmp(0) = 20;
  // p_tmp(1) = result(0) * p_tmp(1) + result(1);
  // p_tmp(2) = 0;
  // p_tmp(3) = 1;
  // Eigen::Vector4d end = (car_point * map_transform).inverse() * p_tmp;

  nav_msgs::Path line;
  line.header.seq++;
  line.header.stamp = ros::Time::now();
  line.header.frame_id = "map";

  geometry_msgs::PoseStamped p1, p2;
  p1.pose.position.x = 0;
  p1.pose.position.y = result(0) * p1.pose.position.x + result(1);
  p1.pose.position.z = 0;
  
  p2.pose.position.x = -5;
  p2.pose.position.y = result(0) * p2.pose.position.x + result(1);
  p2.pose.position.z = 0;

  line.poses.push_back(p1);
  line.poses.push_back(p2);

  line_pub.publish(line);

}

Eigen::Vector2d WallFollow::GetLine(const Eigen::Vector2d& right,
                                    const Eigen::Vector2d& left,
                                    const Eigen::Vector2d& position) {

  double k_r = right(0);
  double b_r = right(1);
  double k_l = left(0);
  double b_l = left(1);

  double x = position(0);
  double y = position(1);

  double l_right = fabs(k_r * x - y + b_r) / sqrt(k_r * k_r + 1);
  double l_left = fabs(k_l * x - y + b_l) / sqrt(k_l * k_l + 1);

  Eigen::Vector2d result;
  if (l_right > l_left) {
    result = left;
  } else {
    result = right;
  }
  

  return result;
}

Eigen::Vector2d WallFollow::LinearFitting(const XYMapVec& local_map) {

 
  Eigen::Vector2d result;

  if (local_map.size() > 1) {
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(local_map.size(), 2);
    Eigen::VectorXd v(local_map.size());
    for (size_t i = 0; i < local_map.size(); i++) {
      m(i, 0) = local_map[i].x;
      m(i, 1) = 1;
      v(i) = local_map[i].y;
    }
    result = m.jacobiSvd(ComputeThinU | ComputeThinV).solve(v);
  } else {
    result << 0, 0;
  }

  return result;
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "wall_follow");
  ros::NodeHandle n;
  std::string add = "/home/george/slam_ws/src/wall_following/data/config.yaml";
  WallFollow follow(n, add);
  ros::Subscriber map_sub = n.subscribe("map", 100, &WallFollow::GetMapCallback, &follow);
  ros::Subscriber odom_sub = n.subscribe("odom", 100, &WallFollow::GetOdometryCallback, &follow);
  ros::Subscriber scan_sub = n.subscribe("scan", 100, &WallFollow::GetScanCallback, &follow);
  ros::spin();

  return 0;
}
