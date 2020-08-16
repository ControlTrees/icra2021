#pragma once

#include <visualization_msgs/Marker.h>
#include <ros/node_handle.h>

void log_to_file(std::ofstream & ofs, ros::NodeHandle & n, double car_x, int i, double cost, double time);

std::string filename(const std::string & name, double p_obstacle);
