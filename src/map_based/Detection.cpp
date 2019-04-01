#include "wall_following/map_based/Detection.h"

using wall::Detection;
using wall::Point2d;
using wall::Point2dVec;
using wall::Point3d;
using wall::Point3dVec;
using wall::PointPolar;
using wall::PointPolarVec;


Detection::Detection() {
  
  map_size = 0;
  get_map = false;
  nh = ros::NodeHandle("~");
  ParamInit();

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
  if (!nh.getParam("angle_interval", angle_interval)) {
  	angle_interval = 0.04;
  }
  

}


void Detection::MapCallback(const nav_msgs::OccupancyGrid& map_msg) {

  int map_msg_size = 0;
  for (size_t i = 0; i < map_msg.data.size(); i++) {
    if (wall::PASSABLE == map_msg.data[i]) {
      map_msg_size++;
    }
  }
  
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

    for (size_t i = 0; i < width; i++) { 
      for (size_t j = 0; j < height; j++) {
        // int n = i * width + j;
        int n = i + j * height;
        if (wall::PASSABLE == map_msg.data[n]) {
          Eigen::Vector4d v_tmp;
          v_tmp << (i + 0.5) * reso, (j + 0.5) * reso, 0, 1;
          Eigen::Vector4d v = map_msg_transform * v_tmp;

          Point2d map_2d_tmp;
          map_2d_tmp.x = v(0);
          map_2d_tmp.y = v(1);
          map_2d.push_back(map_2d_tmp);
        }
      }
    }
    map_size = map_msg_size;
    get_map = true;
    ROS_INFO("wall detect node - map size difference is %d, map updated", 
             map_dif);

    PolarMap();
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
  
  // sort map points based on theta
  for (size_t i = 0; i < map_polar.size()-1; i++) {
  	for (size_t j = 0; j < map_polar.size()-1-i; j++) {
  	  if (map_polar[j].theta > map_polar[j+1].theta) {
  	  	PointPolar tmp = map_polar[j+1];
  	  	map_polar[j+1]= map_polar[j];
  	  	map_polar[j] = tmp;
  	  }
  	}
  }
  
  PointPolarVec polar_vec;
  polar_vec.push_back(map_polar[0]);
  for (size_t i = 1; i < map_polar.size(); i++) {
    double angle_dif 
      = fabs(map_polar[i].theta - polar_vec[polar_vec.size()-1].theta);
  	if (angle_dif > angle_interval) {
  	  polar_vec.push_back(map_polar[i]);
  	}
  }

  map_polar.clear();
  map_polar = polar_vec;

  std::ofstream out;
  out.open("/home/george/slam_ws/src/wall_following/data/map.txt");
  if (out.is_open()) {
  	for (size_t i = 0; i < map_polar.size(); i++) {
  	  out << map_polar[i].theta << "  " << map_polar[i].r << std::endl;
  	}
  } else {
  	std::cout << "open failure !!!" << std::endl;
  }
  out.close();

}