#include "wall_following/map_based/Detection.h"

using Eigen::ComputeThinU;
using Eigen::ComputeThinV;

using wall::Detection;
using wall::Point2d;
using wall::Point2dVec;
using wall::Point3d;
using wall::Point3dVec;
using wall::PointPolar;
using wall::PointPolarVec;
using wall::LineParam;
using wall::WallParam;
using wall::WallParamVec;


Detection::Detection() {
  
  map_size = 0;
  get_map = false;
  nh = ros::NodeHandle("~");
  ParamInit();
  ROS_INFO("object initialized!");

}


void Detection::Setup() {


}


void Detection::ParamInit() {

  if (!nh.getParam("update_map_dif", update_map_dif)) {
  	update_map_dif = 20;
  }
  if (!nh.getParam("map_frame_id", map_frame_id)) {
  	map_frame_id = "map";
  }
  if (!nh.getParam("base_frame_id", base_frame_id)) {
  	base_frame_id = "base_link";
  }
  if (!nh.getParam("skip_interval", skip_interval)) {
  	skip_interval = 0.04;
  }
  if (!nh.getParam("least_n", least_n)) {
    least_n = 20;
  }
  if (!nh.getParam("angle_limit", angle_limit)) {
    angle_limit = 0.088;
  }

}


void Detection::MapCallback(const nav_msgs::OccupancyGrid& map_msg) {

  int map_msg_size = 0;
  for (size_t i = 0; i < map_msg.data.size(); i++) {
    if (wall::PASSABLE == map_msg.data[i]) {
      map_msg_size++;
    }
  }
  
  ROS_INFO("map size is : %d", map_msg_size);
  int map_dif = abs(map_size - map_msg_size); 
  if (map_dif > update_map_dif) {
  	map_2d.clear();
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
    

    for (size_t i = 0; i < map_msg.data.size(); i++) {
      if (wall::PASSABLE == map_msg.data[i]) {
        Eigen::Vector4d v_tmp;
          // v_tmp << (i + 0.5) * reso, (j + 0.5) * reso, 0, 1;
        v_tmp << (i%width + 0.5) * reso, ((int)(i/width) + 0.5) * reso, 0, 1;
        Eigen::Vector4d v = map_msg_transform * v_tmp;

        Point2d map_2d_tmp;
        map_2d_tmp.x = v(0);
        map_2d_tmp.y = v(1);
        map_2d.push_back(map_2d_tmp);
      }
    }

    ROS_INFO("finish map allocattion");
    map_size = map_msg_size;
    get_map = true;
    ROS_INFO("wall detect node - map size difference is %d, map updated", 
             map_dif);

    PolarMap();
    MapPartition();
    LinearFit();
  }
}


void Detection::PoseCallback(const ros::TimerEvent&) {

  if (get_map) {
  	ros::Time stamp = ros::Time(0);
  	tf::StampedTransform base_transform_map;
  	tf::TransformListener listener;
    try {
      listener.waitForTransform(map_frame_id, 
                                base_frame_id, 
                                stamp, 
                                ros::Duration(1.0));
      listener.lookupTransform(map_frame_id, 
                               base_frame_id, 
                               stamp, 
                               base_transform_map);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("Get errors in wall detect while requesting transform from base to map: %s", ex.what());
      ros::Duration(1.0).sleep();
    } 
  }
}

void Detection::PolarMap() {
  
  map_polar.clear();
  // transform map data to polar coordinates
  for (size_t i = 0; i < map_2d.size(); i++) {
  	PointPolar map_polar_tmp;
  	double x = map_2d[i].x;
  	double y = map_2d[i].y;
  	map_polar_tmp.r = sqrt(x*x + y*y);
  	map_polar_tmp.theta = atan2(y, x);

  	map_polar.push_back(map_polar_tmp);
  }
  
  // sort sequence map points based on theta
  for (size_t i = 0; i < map_polar.size()-1; i++) {
  	for (size_t j = 0; j < map_polar.size()-1-i; j++) {
  	  if (map_polar[j].theta > map_polar[j+1].theta) {
  	  	PointPolar tmp = map_polar[j+1];
  	  	map_polar[j+1]= map_polar[j];
  	  	map_polar[j] = tmp;
  	  }
  	}
  }
  
  // skip some map points when they are too closed to each other
  // based on theta difference
  PointPolarVec polar_vec;
  polar_vec.push_back(map_polar[0]);
  for (size_t i = 1; i < map_polar.size(); i++) {
    double angle_dif 
      = fabs(map_polar[i].theta - polar_vec[polar_vec.size()-1].theta);
  	if (angle_dif > skip_interval) {
  	  polar_vec.push_back(map_polar[i]);
  	}
  }

  map_polar.clear();
  map_polar = polar_vec;
  
  map_2d.clear();
  for (size_t i = 0; i < map_polar.size(); i++) {
    Point2d map_tmp;
    map_tmp.x = map_polar[i].r * cos(map_polar[i].theta);
    map_tmp.y = map_polar[i].r * sin(map_polar[i].theta);

    map_2d.push_back(map_tmp);
  }

  std::ofstream out;
  out.open("/home/glh/slam_ws/src/wall_following/data/map.txt");
  if (out.is_open()) {
  	for (size_t i = 0; i < map_polar.size(); i++) {
  	  out << map_polar[i].theta << "  " << map_polar[i].r << std::endl;
  	}
  } else {
  	std::cout << "open failure !!!" << std::endl;
  }
  out.close();

}

void Detection::MapPartition() {

  Point2dVec group_tmp;
  group_tmp.clear();
  points_group.clear();
  // std::cout << map_2d.size() << std::endl;

  int count = 0;
  // angle_vec.clear();
  // value_vec.clear();
  
  size_t index = 0;
  if (map_2d.size() > 2) {
    group_tmp.push_back(Point2d(map_2d[0].x, map_2d[0].y));
    group_tmp.push_back(Point2d(map_2d[1].x, map_2d[1].y));

    for (size_t i = 2; i < (map_2d.size() - 2); i++) {
      Eigen::Vector2d v1(map_2d[index+1].x - map_2d[index].x,
                         map_2d[index+1].y - map_2d[index].y);
      Eigen::Vector2d v2(map_2d[i].x - map_2d[index].x,
                         map_2d[i].y - map_2d[index].y);
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

      Point2d result_tmp;
      result_tmp.x = map_2d[i].x;
      result_tmp.y = map_2d[i].y;

      if (angle_rad > angle_limit) {
        count++;
      }
      
      //================
      // angle_vec.push_back(angle_rad);
      // value_vec.push_back(v1.dot(v2) / (nv1 * nv2));

      if (std::isnan(angle_rad)) {
        angle_rad = 0;
      }
      //================
      if (angle_rad <= angle_limit || std::isnan(angle_rad)) {
        group_tmp.push_back(result_tmp);
      } else if (group_tmp.size() < least_n) {
        group_tmp.clear();
        group_tmp.push_back(result_tmp);
        index = i;      
      }
      if (angle_rad > angle_limit && group_tmp.size() > least_n) {
        // std::cout << "i push into points_group" << std::endl;
        points_group.push_back(group_tmp);
        // std::cout << "size now is : " << points_group.size() << std::endl;
        group_tmp.clear();
        group_tmp.push_back(result_tmp);
        index = i;      
      }  
    }
  }
  // std::cout << "count is : " << count << std::endl;
  if (group_tmp.size() > least_n) {
    points_group.push_back(group_tmp);
  }
  ROS_INFO("points group size is : %d", (int)points_group.size());

}

void Detection::LinearFit() {
  
  walls.clear();
  std::cout << points_group.size() << std::endl;
  std::ofstream out;
  out.open("/home/glh/slam_ws/src/wall_following/data/line_param.txt");
  for (size_t i = 0; i < points_group.size(); i++) {
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(points_group[i].size(), 2);
    Eigen::VectorXd v(points_group[i].size());
    
    double min_x = 1000000.0;
    double max_x = -1000000.0;
    for (size_t j = 0; j < points_group[i].size(); j++) {
      m(j, 0) = points_group[i][j].x;
      m(j, 1) = 1;
      v(j) = points_group[i][j].y;

      if (points_group[i][j].x > max_x) {
        max_x = points_group[i][j].x;
      }
      if (points_group[i][j].x < min_x) {
        min_x = points_group[i][j].x;
      }
    }
    Eigen::Vector2d param = m.jacobiSvd(ComputeThinU | ComputeThinV).solve(v);

    WallParam wall_tmp;
    wall_tmp.param.k = param(0);
    wall_tmp.param.b = param(1);
    wall_tmp.param.is_line = true;
    wall_tmp.start.x = min_x;
    wall_tmp.start.y = param(0) * min_x + param(1);
    wall_tmp.end.x = max_x;
    wall_tmp.end.y = param(0) * max_x + param(1);    
    walls.push_back(wall_tmp);

    out << param(0) << "  " << param(1) << "  " << min_x << "  " << max_x << std::endl;
  }
  out.close();
  ROS_INFO("size of walls is : %d", (int)walls.size());

}