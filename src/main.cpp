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

#define PI 3.1415926

using Eigen::ComputeThinU;
using Eigen::ComputeThinV;

// enum PointState {
//   OCCUPIED,
//   FREE,
//   UNKNOWN;
// };

struct XYMap {
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
  Eigen::Vector2d LinearFitting(const XYMapVec& local_map);
  Eigen::Vector2d GetLine(const Eigen::Vector2d& right,
                          const Eigen::Vector2d& left,
                          const Eigen::Vector2d& position);
  void PubLine(const Eigen::VectorXd& result,
               const Eigen::Matrix4d& base_point);
  void PubCircle(const Eigen::Matrix4d& base_point);
 
 private:
  XYMapVec xy_map;
  bool map_flag;
  double limit;
  Eigen::Matrix4d map_transform;

  ros::Publisher line_pub;
  ros::Publisher circle_pub;
};

WallFollow::WallFollow(ros::NodeHandle n, const std::string add) {
  YAML::Node config = YAML::LoadFile(add);
  xy_map.clear(); 
  map_flag = false; 
  limit = config["limit"].as<double>();
  map_transform = Eigen::Matrix4d::Identity();
  line_pub = n.advertise<nav_msgs::Path>("path", 100);
  circle_pub = n.advertise<nav_msgs::Path>("circle", 100);
}

void WallFollow::GetMapCallback(const nav_msgs::OccupancyGrid& map) {
  double reso = map.info.resolution;
  int width = map.info.width;
  int height = map.info.height;
  map_transform(0,3) = map.info.origin.position.x;
  map_transform(1,3) = map.info.origin.position.y;


  for (size_t i = 0; i < width; i++) {
    for (size_t j = 0; j < height; j++) {
      int n = i * width + j;
      if (100 == map.data[n]) {
        XYMap xy_map_tmp;
        xy_map_tmp.x = (i + 0.5) * reso;
        xy_map_tmp.y = (j + 0.5) * reso;
        // xy_map_tmp.state = OCCUPIED;
        xy_map.push_back(xy_map_tmp);
      }
    }
  }
  
  map_flag = true;

}

void WallFollow::GetOdometryCallback(const nav_msgs::Odometry& pose) {

  if (map_flag) {
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

    // std::cout << "base matrix is : " << std::endl << base_point << std::endl;
 
    Eigen::Matrix4d car_point = base_point.inverse();
    // std::cout << "car matrix is : " << std::endl << car_point << std::endl;
    // std::cout << "====================" << std::endl;
    
    XYMapVec local_map_right;
    XYMapVec local_map_left; 
    Eigen::Vector4d t(0.0, 0.0, 0.0, 1.0);



    for (size_t i = 0; i < xy_map.size(); i++) {
      t(0) = xy_map[i].x;
      t(1) = xy_map[i].y;

      Eigen::Vector4d t_transformed =  car_point * map_transform * t;
      double dis = sqrt(pow(t_transformed(0),2) + pow(t_transformed(1),2));

      
      if (t_transformed(1) < 0 && dis <= limit) {
        XYMap local_map_tmp;
        local_map_tmp.x = t_transformed(0);
        local_map_tmp.y = t_transformed(1);
        local_map_right.push_back(local_map_tmp);
      } else if (t_transformed(1) > 0 && dis <= limit) {
        XYMap local_map_tmp;
        local_map_tmp.x = t_transformed(0);
        local_map_tmp.y = t_transformed(1);
        local_map_left.push_back(local_map_tmp);
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
  }

}

void WallFollow::PubMap() {
  
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
  
  Eigen::Vector4d p_tmp;

  p_tmp(0) = 0;
  p_tmp(1) = result(0) * p_tmp(0) + result(1);
  p_tmp(2) = 0;
  p_tmp(3) = 1;
  Eigen::Vector4d start = (car_point * map_transform).inverse() * p_tmp;

  p_tmp(0) = 20;
  p_tmp(1) = result(0) * p_tmp(1) + result(1);
  p_tmp(2) = 0;
  p_tmp(3) = 1;
  Eigen::Vector4d end = (car_point * map_transform).inverse() * p_tmp;

  nav_msgs::Path line;
  line.header.seq++;
  line.header.stamp = ros::Time::now();
  line.header.frame_id = "map";

  geometry_msgs::PoseStamped p1, p2;
  p1.pose.position.x = start(0);
  p1.pose.position.y = start(1);
  p1.pose.position.z = 0;
  
  p2.pose.position.x = end(0);
  p2.pose.position.y = end(1);
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

  ros::spin();

  return 0;
}
