#include <control_tree/ros/obstacle_common.h>

#include <fstream>

#include <ros/package.h>
#include <ros/console.h>

#include <chrono>

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
