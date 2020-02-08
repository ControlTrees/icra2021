#include <control_tree/ros/pedestrian_common.h>

#include <fstream>

#include <ros/package.h>
#include <ros/console.h>

#include <chrono>

visualization_msgs::Marker create_road_border(double map_x)
{
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.id = std::hash<std::string>()("center_line");
    marker.type = visualization_msgs::Marker::LINE_LIST;//;visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.scale.x = 0.1; // thickness
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.5;
    marker.color.a = 0.8;

    geometry_msgs::Point p;
    p.x = map_x - 100;
    p.y = -1.75;
    p.z = 0;

    marker.points.push_back(p);

    p.x = map_x + 300;
    p.y = -1.75;
    p.z = 0;

    marker.points.push_back(p);

    p.x = map_x - 100;
    p.y = 1.75;
    p.z = 0;

    marker.points.push_back(p);

    p.x = map_x + 300;
    p.y = 1.75;
    p.z = 0;

    marker.points.push_back(p);

    return marker;
}

void log_to_file(std::ofstream & ofs, ros::NodeHandle & n, double car_x, int i, double cost, double velocity, double time)
{
    int n_crossings, n_non_crossings;
    n.getParam("/n_crossings", n_crossings);
    n.getParam("/n_non_crossings", n_non_crossings);

    double p_crossing;
    n.getParam("p_crossing", p_crossing);

    std::stringstream ss;

    ss << "N:" << n_non_crossings + n_crossings << " Avg dist between pedestrians:" << car_x / (n_non_crossings + n_crossings) << " Probability:" << p_crossing << " TREE QP avg cost:" << cost << " velocity(m/s):" << velocity << " time(ms):" << time * 1000 << " " << i << " iterations";

    ROS_INFO_STREAM(ss.str());
    ofs << ss.str() << std::endl;
}

std::string filename(const std::string & name, double p_crossing, int n_pedestrians, int n_branches)
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/camille/Phd/Paper/RSS/plots/gen/data-" << std::to_string(p_crossing) << "-" << name << "-" << std::to_string(n_pedestrians) << "-" << std::to_string(n_branches) << "-" << t << ".txt";
    return ss.str();
}
