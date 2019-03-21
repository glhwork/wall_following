#ifndef MOVESIDE_H
#define MOVESIDE_H

#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "nav_msgs/Path.h"

namespace wall {

class MoveSide {
 public:
  MoveSide();
  ~MoveSide() {}
  void GetWallCallback(const nav_msgs::Path& line);
 private:

	
};  // class MoveSide

}  // namespace wall

#endif