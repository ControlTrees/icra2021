#include <control_tree/ros/obstacle_common.h>

#include <fstream>

#include <ros/package.h>
#include <ros/console.h>

#include <chrono>

visualization_msgs::Marker create_center_line(double map_x)
{
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.id = std::hash<std::string>()("center_line");
    marker.type = visualization_msgs::Marker::LINE_STRIP;//;visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.scale.x = 0.1; // thickness
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.5;
    marker.color.a = 0.5;

    geometry_msgs::Point p;
    p.x = map_x - 100;
    p.y = 0;
    p.z = 0;

    marker.points.push_back(p);

    p.x = map_x + 300;
    p.y = 0;
    p.z = 0;

    marker.points.push_back(p);

    return marker;
}

void log_to_file(std::ofstream & ofs, ros::NodeHandle & n, double car_x, int i, double cost, double time)
{
    int n_obstacles, n_non_obstacles;
    n.getParam("/n_obstacles", n_obstacles);
    n.getParam("/n_non_obstacles", n_non_obstacles);

    double p_obstacle;
    n.getParam("p_obstacle", p_obstacle);

    std::stringstream ss;

    ss << "N:" << n_non_obstacles + n_non_obstacles << " Avg dist between obstacles:" << car_x / (n_non_obstacles + n_non_obstacles) << " Probability:" << p_obstacle << " Avg cost:" << cost << " time(ms):" << time * 1000 << " " << i << " iterations";

    ROS_INFO_STREAM(ss.str());
    ofs << ss.str() << std::endl;
}

std::string filename(const std::string & name, double p_obstacle)
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/camille/Phd/Paper/RSS/plots/gen_obstacles/data-" << std::to_string(p_obstacle) << "-" << name << "-" << t << ".txt";
    return ss.str();
}
