#pragma once

#include <visualization_msgs/Marker.h>
#include <ros/node_handle.h>

void log_to_file(std::ofstream & ofs, ros::NodeHandle & n, double car_x, int i, double cost);

std::string filename(ros::NodeHandle & n);

visualization_msgs::Marker create_obstacle_marker(double x, double y, double sx, double sy, double sz, double alpha, int id);
visualization_msgs::Marker create_collision_marker(double x, double y, double sx, double sy, double sz, double alpha, int id);
