#include "wall_following/MoveSide.h"

using wall::RoboPosi;
using wall::RoboOrie;
using wall::RoboPose;
using wall::MoveSide;

#define PI 3.1415926

MoveSide::MoveSide(ros::NodeHandle n) {
  get_pose = false;
  send_first_goal = false;
  nh = ros::NodeHandle("~");
  goal_n = ros::NodeHandle("move_base_simple");

  ParamInit();
  Setup(n);
}

void MoveSide::ParamInit() {

  if (!nh.getParam("laser_frame_id", laser_frame_id)) {
    laser_frame_id = "base_scan";
  }
  if (!nh.getParam("base_frame_id", base_frame_id)) {
    base_frame_id = "base_link";
  }
  if (!nh.getParam("map_frame_id", map_frame_id)) {
    map_frame_id = "map";
  }
  if (!nh.getParam("to_wall_dis", to_wall_dis)) {
    to_wall_dis = 0.2;
  }

}

void MoveSide::Setup(ros::NodeHandle n) {
  first_goal_pub = goal_n.advertise<geometry_msgs::PoseStamped>("goal", 10);
  first_wall_pub = n.advertise<nav_msgs::Path>("first_wall", 10);
  to_wall_vec_pub = n.advertise<nav_msgs::Path>("to_wall", 10);
  goal_marker_pub = n.advertise<visualization_msgs::Marker>("goal_marker", 10);
}

void MoveSide::GetWallCallback(const nav_msgs::Path& wall) {
  
  if (send_first_goal) {
    to_wall_vec_pub.publish(to_wall_vec);
    first_wall_pub.publish(show_wall);
  	return ;
  }  
  
  show_wall = wall;

  double k, b;
  k = (wall.poses[0].pose.position.y - wall.poses[1].pose.position.y) / 
      (wall.poses[0].pose.position.x - wall.poses[1].pose.position.x);
  b = wall.poses[0].pose.position.y - k * wall.poses[0].pose.position.x;
 
  RoboPosi first_goal;
  Eigen::Quaterniond q;
  double k_pose, b_pose;
  if (k != 0) {
  	k_pose = -1 / k;
  	b_pose = global_pose.position.y - k_pose * global_pose.position.x;

  	RoboPosi inter_point;
  	inter_point.x = (b_pose - b) / (k - k_pose);
  	inter_point.y = k * inter_point.x + b;

    first_goal = FirstGoal(inter_point, k_pose, b_pose);
  } else {
  	RoboPosi inter_point;
  	inter_point.x = global_pose.position.x;
  	inter_point.y = wall.poses[0].pose.position.y;

  	first_goal.x = inter_point.x;
  	if (global_pose.position.y > inter_point.y) {
  	  first_goal.y = inter_point.y + to_wall_dis;
  	} else {
  	  first_goal.y = inter_point.y - to_wall_dis;
  	}
  }
  double yaw = FindYaw(wall);
  ROS_ERROR("yaw angle is : %.4f", yaw/PI*180);
  q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  // Eigen::Matrix3d rot;
  // rot = q.toRotationMatrix();

  // publish the goal point
  geometry_msgs::PoseStamped goal_msg;
  goal_msg.header.frame_id = map_frame_id;
  goal_msg.header.stamp = ros::Time::now();

  goal_msg.pose.position.x = first_goal.x;
  goal_msg.pose.position.y = first_goal.y;
  goal_msg.pose.position.z = 0.0;
  goal_msg.pose.orientation.x = q.x();
  goal_msg.pose.orientation.y = q.y();
  goal_msg.pose.orientation.z = q.z();
  goal_msg.pose.orientation.w = q.w();

  first_goal_pub.publish(goal_msg);
  PubGoal(goal_msg);
  send_first_goal = true;
}

void MoveSide::GetPoseCallback(const ros::TimerEvent&) {
  
  ros::Time stamp = ros::Time(0);
  tf::TransformListener listener;
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
  } catch (tf::TransformException &ex) {
    ROS_ERROR("Get errors in move side while requesting transform from base to map: %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  RoboPosi position(base_transform_map.getOrigin().x(),
  	                base_transform_map.getOrigin().y(),
  	                0);
  tf::Quaternion tf_quat = base_transform_map.getRotation();
  RoboOrie orientation(tf_quat.getW(),
  	                   tf_quat.getX(),
  	                   tf_quat.getY(),
  	                   tf_quat.getZ());

  global_pose = RoboPose(position, orientation);
  // ROS_INFO("Position of robot is : x = %.2f, y = %.2f, z = %.2f", 
  //          global_pose.position.x, 
  //          global_pose.position.y, 
  //          global_pose.position.z); 
  // ROS_INFO("q_x = %.2f, q_y = %.2f, q_z = %.2f, q_w = %.2f",
  //          global_pose.orientation.x(), global_pose.orientation.y(), 
  //          global_pose.orientation.z(), global_pose.orientation.w());
  // std::cout << "Rotation Matrix : " << std::endl;
  // std::cout << global_pose.orientation.toRotationMatrix() << std::endl;
  
  get_pose = true;
}

RoboPosi MoveSide::FirstGoal(const RoboPosi& inter_point, 
	                     const double k_pose, 
	                     const double b_pose) {

  RoboPosi goal;
  
  RoboPosi pose_point = global_pose.position;
  // double a, b, c;
  // a = k_pose * k_pose + 1;
  // b = 2 * (b_pose - inter_point.y) * k_pose - 2 * inter_point.x;
  // c = inter_point.x * inter_point.x 
  //     + pow((b_pose - inter_point.y), 2) 
  //     - pow(to_wall_dis, 2);

  // double root_1, root_2;
  // double delta;
  // delta = b * b - 4 * a * c;
  // if (delta >= 0) {
  // 	root_1 = (- b + sqrt(b * b - 4 * a * c)) / (2 * a);
  //   root_2 = (- b - sqrt(b * b - 4 * a * c)) / (2 * a);

  //   Eigen::Vector2d vec_ref, vec_1, vec_2;
  //   vec_ref << inter_point.x - pose_point.x, 
  //              inter_point.y - pose_point.y;
  //   vec_1 << inter_point.x - root_1,
  //            inter_point.y - (k_pose * root_1 + b_pose);
  //   vec_2 << inter_point.x - root_2,
  //            inter_point.y - (k_pose * root_2 + b_pose);

  //   if (vec_ref.dot(vec_1) > 0.0) {
  //     goal.x = vec_1(0);
  //     goal.y = vec_1(1);
  //   } else if (vec_ref.dot(vec_2) > 0.0) {
  //     goal.x = vec_2(0);
  // 	  goal.y = vec_2(1);
  //   }

  // } else {
  // 	ROS_ERROR("no solution for first goal");
  // } 
  

  // get the unit vector of reference vector
  Eigen::Vector2d vec_ref(inter_point.x - pose_point.x, 
                          inter_point.y - pose_point.y);
  double ref_norm = sqrt(pow(vec_ref(0), 2) + pow(vec_ref(1), 2));
  Eigen::Vector2d unit_ref = vec_ref / ref_norm;
  goal.x = inter_point.x - unit_ref(0) * to_wall_dis;
  goal.y = k_pose * goal.x + b_pose;
  


  to_wall_vec.header.frame_id = map_frame_id;
  to_wall_vec.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped p;
  p.pose.position.x = inter_point.x;
  p.pose.position.y = inter_point.y;
  p.pose.orientation.w = 1.0;
  to_wall_vec.poses.push_back(p);

  p.pose.position.x = goal.x;
  p.pose.position.y = goal.y;
  p.pose.orientation.w = 1.0;
  to_wall_vec.poses.push_back(p); 


  return goal;

}

double MoveSide::FindYaw(const nav_msgs::Path& wall) {

  double l_0 = FindDist(wall.poses[0].pose.position.x,
  	                    wall.poses[0].pose.position.y,
  	                    global_pose.position.x,
  	                    global_pose.position.y);
  double l_1 = FindDist(wall.poses[1].pose.position.x,
  	                    wall.poses[1].pose.position.y,
  	                    global_pose.position.x,
  	                    global_pose.position.y);
  RoboPosi start, end;
  if (l_0 >= l_1) {
  	start.x = wall.poses[1].pose.position.x;
  	start.y = wall.poses[1].pose.position.y;

  	end.x = wall.poses[0].pose.position.x;
  	end.y = wall.poses[0].pose.position.y;
  } else {
  	start.x = wall.poses[0].pose.position.x;
  	start.y = wall.poses[0].pose.position.y;

  	end.x = wall.poses[1].pose.position.x;
  	end.y = wall.poses[1].pose.position.x;
  }

  Eigen::Vector2d v1(end.x - start.x, end.y - start.y);
  Eigen::Vector2d v2(1, 0);
  double nv1 = sqrt(pow(v1(0), 2) + pow(v1(1), 2));
  double nv2 = sqrt(pow(v2(0), 2) + pow(v2(1), 2));
  
  ROS_ERROR("The vector is [x: %.2f, y: %.2f]", v1(0), v1(1));
  if ((nv1 * nv2) == 0) {
  	ROS_ERROR("Get error from move side -> norm of vector is zero");
  }
  double yaw = acos(v1.dot(v2) / (nv1 * nv2));
  if (v1(1) < v2(1)) {
  	yaw = -yaw;
  }

  return yaw;
}

double MoveSide::FindDist(double x1, double y1, double x2, double y2) {
  
  double l;
  l = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));

  return l;
}

void MoveSide::PubGoal(const geometry_msgs::PoseStamped& pose) {

  visualization_msgs::Marker marker;
  marker.header.frame_id = map_frame_id;
  marker.header.stamp = ros::Time::now();
  
  marker.ns = "goal_point";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 
  marker.scale.y =
  marker.scale.z = 0.5;

  marker.color.b = 1.0;
  marker.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = pose.pose.position.x;
  p.y = pose.pose.position.y;
  p.z = pose.pose.position.z;

  marker.points.push_back(p);

  goal_marker_pub.publish(marker);


}
