#include "wall_following/WallDetect.h"

using Eigen::ComputeThinU;
using Eigen::ComputeThinV;

using wall::WallDetect;
using wall::XYMap;
using wall::XYMapVec;
using wall::LineParam;

// a vertical line appears suddenly in the fitting process
// check whether the final line parameter is empty
// the problem about nan


WallDetect::WallDetect(ros::NodeHandle n) {

  xy_map.clear(); 
  map_size = 0;
  get_map = false;
  get_laser = false;
  nh = ros::NodeHandle("~");
  // map_transform = Eigen::Matrix4d::Identity();

  ParamInit();
  Setup(n);

}

void WallDetect::GetMapCallback(const nav_msgs::OccupancyGrid& map_msg) {

  if (abs(map_size - map_msg.data.size()) > update_map_dif) {
    double reso = map_msg.info.resolution;
    int width = map_msg.info.width;
    int height = map_msg.info.height;
    map_msg_transform(0,3) = map_msg.info.origin.position.x;
    map_msg_transform(1,3) = map_msg.info.origin.position.y;

    for (size_t i = 0; i < width; i++) { 
      for (size_t j = 0; j < height; j++) {
        //int n = i * width + j;
        int n = i + j * height;
        if (100 == map_msg.data[n]) {
          Eigen::Vector4d v_tmp;
          v_tmp << (i + 0.5) * reso, (j + 0.5) * reso, 0, 1;
          Eigen::Vector4d v = map_msg_transform * v_tmp;

          XYMap xy_map_tmp;
          xy_map_tmp.x = v(0);
          xy_map_tmp.y = v(1);
          xy_map.push_back(xy_map_tmp);
        }
      }
    }
  
    get_map = true;

  }
}


void WallDetect::FindWallCallback(const ros::TimerEvent&) {

  if (get_laser) {
    
    XYMapVec laser_points_map;
    std::vector<geometry_msgs::PointStamped> trans_input_vec;
    ros::Time stamp = ros::Time(0);
    for (size_t i = 0; i < laser_points.size(); i++) {
      geometry_msgs::PointStamped trans_input_tmp;
      trans_input_tmp.point.x = laser_points[i].x;
      trans_input_tmp.point.y = laser_points[i].y;
      trans_input_tmp.point.z = 0.0;
      trans_input_tmp.header.frame_id = laser_frame_id;
      trans_input_tmp.header.stamp = stamp;

      trans_input_vec.push_back(trans_input_tmp);
    }
    tf::TransformListener listener;
    // this represents pose of laser frame w.r.t. map frame
    tf::StampedTransform laser_to_map;
    try {
      listener.waitForTransform(map_frame_id ,
                                laser_frame_id, 
                                stamp, 
                                ros::Duration(1.0));
      for (size_t i = 0; i < trans_input_vec.size(); i++) {
        XYMap laser_points_map_tmp;
        geometry_msgs::PointStamped trans_output;
        listener.transformPoint(map_frame_id, 
                                trans_input_vec[i], 
                                trans_output);
        laser_points_map_tmp.x = trans_output.point.x;
        laser_points_map_tmp.y = trans_output.point.y;
        laser_points_map.push_back(laser_points_map_tmp);
      }
    } catch (tf::TransformException &ex) {
      ROS_ERROR("Get errors in wall detect while requesting transform from laser to map: %s", ex.what());
      ros::Duration(1.0).sleep();
    }     

    std::vector<XYMapVec> laser_cut = LineCut(laser_points_map);
    
    std::vector<LineParam> param_vec;
    if (laser_cut.size() > 0) {
      param_vec = LinearFit(laser_cut);
    } else {
      param_vec.push_back(LineParam(0, 0, false));
    } // attention! k = 0 and b = 0 can also represent a straight line
  
    tf::StampedTransform base_transform_map;
    try {
      listener.waitForTransform(map_frame_id, 
                                base_frame_id, 
                                stamp, 
                                ros::Duration(1.0));
      listener.lookupTransform(map_frame_id, 
                               base_frame_id, 
                               stamp, 
                               base_transform_map);
      ROS_INFO("Transform between base and map is completed");
    } catch (tf::TransformException &ex) {
      ROS_ERROR("Get errors in wall detect while requesting transform from base to map: %s", ex.what());
      ros::Duration(1.0).sleep();
    }
    Eigen::Vector2d position(base_transform_map.getOrigin().x(),
                             base_transform_map.getOrigin().y());
    
    LineParam line = GetLine(param_vec, position);
    std::cout << "k is : " << line.k << " and b is : " << line.b << std::endl;
    std::cout << "============================" << std::endl;
    
    PubLine(line);
    // PubCircle(base_point);
    PubMap();
  }

}

void WallDetect::GetScanCallback(const sensor_msgs::LaserScan& scan) {
  
  // get coordinates of laser points w.r.t base_link frame rather than world frame
  laser_points.clear();
  double delta_angle = scan.angle_increment;
  for (size_t i = 0; i < scan.ranges.size(); i++) {

    double range = scan.ranges[i];
    if (range <= scan.range_max && range <= limit) {

      XYMap laser_points_tmp;
      laser_points_tmp.x = range * cos(scan.angle_min + i * delta_angle);
      laser_points_tmp.y = range * sin(scan.angle_min + i * delta_angle);
      laser_points.push_back(laser_points_tmp);

    }

  }
  get_laser = true; 

}

void WallDetect::ParamInit() {

  if (!nh.getParam("limit", limit)) {
    limit = 10.0;
  }
  if (!nh.getParam("least_n", least_n)) {
    least_n = 30;
  }
  if (!nh.getParam("angle_limit", angle_limit)) {
    angle_limit = 0.088;
  }
  if (!nh.getParam("laser_frame_id", laser_frame_id)) {
    laser_frame_id = "base_scan";
  }
  if (!nh.getParam("base_frame_id", base_frame_id)) {
    base_frame_id = "base_link";
  }
  if (!nh.getParam("map_frame_id", map_frame_id)) {
    map_frame_id = "map";
  }
  if (!nh.getParam("update_map_dif", update_map_dif)) {
    update_map_dif = 20;
  }

}

void WallDetect::Setup(ros::NodeHandle n) {

  // ros::Subscriber map_sub = n.subscribe("map", 100, &WallDetect::GetMapCallback, &follow);
  // ros::Subscriber odom_sub = n.subscribe("odom", 100, &WallDetect::GetOdometryCallback, &follow);
  // ros::Subscriber scan_sub = n.subscribe("scan", 100, &WallDetect::GetScanCallback, &follow);


  wall_path_pub = n.advertise<nav_msgs::Path>("path", 100);
  circle_pub = n.advertise<nav_msgs::Path>("circle", 100);
  marker_pub = n.advertise<visualization_msgs::Marker>("map_points", 100);
  left_marker_pub = n.advertise<visualization_msgs::Marker>("left_points", 100);
  right_marker_pub = n.advertise<visualization_msgs::Marker>("right_points", 100);
  laser_marker_pub = n.advertise<visualization_msgs::Marker>("laser_points", 100);
  start_pub = n.advertise<visualization_msgs::Marker>("start_point", 100);

}

void WallDetect::Loop() {

  // for subscriber or timer

}

void WallDetect::PubMap() {
  visualization_msgs::Marker map_marker;

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "laser";
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

std::vector<XYMapVec> WallDetect::LineCut(const XYMapVec& vec) {

  XYMapVec result_vec_tmp;
  std::vector<XYMapVec> result_vec;
  result_vec_tmp.clear();
  result_vec.clear();
  std::cout << vec.size() << std::endl;

  int count = 0;
  angle_vec.clear();
  value_vec.clear();
  
  size_t index = 0;
  if (vec.size() > 2) {
    result_vec_tmp.push_back(XYMap(vec[0].x, vec[0].y));
    result_vec_tmp.push_back(XYMap(vec[1].x, vec[1].y));

    for (size_t i = 2; i < (vec.size() - 2); i++) {
      Eigen::Vector2d v1(vec[index+1].x - vec[index].x,
                         vec[index+1].y - vec[index].y);
      Eigen::Vector2d v2(vec[i].x - vec[index].x,
                         vec[i].y - vec[index].y);
      double nv1 = sqrt(pow(v1(0), 2) + pow(v1(1), 2));
      double nv2 = sqrt(pow(v2(0), 2) + pow(v2(1), 2));
      double angle_rad;

      if (0 == nv1 || 0 == nv2) {
        //  This equation is only for making "angle_rad" less than 
        //  "angle_limit", no other actual meaning
        angle_rad = angle_limit / 2;
      } else {
        double var = v1.dot(v2) / (nv1 * nv2);
        if (1 == var) {
          angle_rad = 0;
        } else {
          angle_rad = fabs(acos(var));
        }
      }

      XYMap result_tmp;
      result_tmp.x = vec[i].x;
      result_tmp.y = vec[i].y;

      if (angle_rad > angle_limit) {
        count++;
      }
      
      //================
      angle_vec.push_back(angle_rad);
      value_vec.push_back(v1.dot(v2) / (nv1 * nv2));

      if (std::isnan(angle_rad)) {
        angle_rad = 0;
      }
      //================
      if (angle_rad <= angle_limit || std::isnan(angle_rad)) {
        result_vec_tmp.push_back(result_tmp);
      } else if (result_vec_tmp.size() < least_n) {
        result_vec_tmp.clear();
        result_vec_tmp.push_back(result_tmp);
        index = i;      
      }
      if (angle_rad > angle_limit && result_vec_tmp.size() > least_n) {
        // std::cout << "i push into result_vec" << std::endl;
        result_vec.push_back(result_vec_tmp);
        // std::cout << "size now is : " << result_vec.size() << std::endl;
        result_vec_tmp.clear();
        result_vec_tmp.push_back(result_tmp);
        index = i;      
      }  
    }
  }
  std::cout << "count is : " << count << std::endl;
  if (result_vec_tmp.size() > least_n) {
    result_vec.push_back(result_vec_tmp);
  }
  
  return result_vec;

}


void WallDetect::PubCircle(const Eigen::Matrix4d& base_point) {
  int circle_reso = 20;
  nav_msgs::Path circle;
  circle.header.frame_id = "laser";
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

void WallDetect::PubLine(const LineParam& line) {

  nav_msgs::Path wall_path;
  wall_path.header.seq++;
  wall_path.header.stamp = ros::Time::now();
  wall_path.header.frame_id = base_frame_id;

  geometry_msgs::PoseStamped p1, p2;
  p1.pose.position.x = -10;
  p1.pose.position.y = line.k * p1.pose.position.x + line.b;
  p1.pose.position.z = 0;
  
  p2.pose.position.x = 10;
  p2.pose.position.y = line.k * p2.pose.position.x + line.b;
  p2.pose.position.z = 0;

  wall_path.poses.push_back(p1);
  wall_path.poses.push_back(p2);

  wall_path_pub.publish(wall_path);

}

LineParam WallDetect::GetLine(const std::vector<LineParam>& param_vec,
                              const Eigen::Vector2d& position) {

  double min_l = 1000000;
  const double x = position(0);
  const double y = position(1);
  LineParam line_final;

  for (size_t i = 0; i < param_vec.size(); i++) {
    if (param_vec[i].is_line && !std::isnan(param_vec[i].k)) {
      double k = param_vec[i].k;
      double b = param_vec[i].b; 
      double l = fabs(k * x - y + b) / sqrt(k * k + 1);

      if (l < min_l) {
        line_final.k = k;
        line_final.b = b;
        line_final.is_line = true;
        min_l = l;
      }
    }
  }
  
  return line_final;
}

std::vector<LineParam> WallDetect::LinearFit(const std::vector<XYMapVec>& cut) {

 
  std::vector<LineParam> param_vec;
  
  for (size_t i = 0; i < cut.size(); i++) {
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(cut[i].size(), 2);
    Eigen::VectorXd v(cut[i].size());

    for (size_t j = 0; j < cut[i].size(); j++) {
      m(j, 0) = cut[i][j].x;
      m(j, 1) = 1;
      v(j) = cut[i][j].y;
    }
    Eigen::Vector2d param = m.jacobiSvd(ComputeThinU | ComputeThinV).solve(v);
    LineParam param_vec_tmp(param(0), param(1), true);
    param_vec.push_back(param_vec_tmp);
  }

  return param_vec;
}

void WallDetect::PubStart(const Eigen::Matrix4d& base) {
  visualization_msgs::Marker s;
  s.header.frame_id = "laser";
  s.header.stamp = ros::Time::now();
  s.ns = "start point";
  s.id = 0;
  s.type = visualization_msgs::Marker::POINTS;
  s.action = visualization_msgs::Marker::ADD;

  Eigen::Vector4d v1_tmp, v2_tmp;
  v1_tmp << laser_points[0].x, laser_points[0].y, 0, 1;
  v2_tmp << laser_points[laser_points.size()-1].x,
            laser_points[laser_points.size()-1].y,
            0,
            1;
  Eigen::Vector4d v1 = base * v1_tmp;
  Eigen::Vector4d v2 = base * v2_tmp;

  geometry_msgs::Point p;
  p.x = v1(0);
  p.y = v1(1);
  s.points.push_back(p);
  p.x = v2(0);
  p.y = v2(1);
  s.points.push_back(p);
  s.color.g = 20.0;
  s.color.a = 1.0;
  s.scale.x = 0.5;
  s.scale.y = 0.5;
  s.scale.z = 0.5;

  start_pub.publish(s);
}