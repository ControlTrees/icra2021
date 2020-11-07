#include <nodes/pedestrian_common.h>

#include <fstream>

#include <ros/package.h>
#include <ros/console.h>

#include <chrono>

void log_to_file(std::ofstream & ofs, ros::NodeHandle & n, double car_x, int i, double cost, double velocity, double time)
{
    int n_crossings, n_non_crossings;
    n.getParam("/n_crossings", n_crossings);
    n.getParam("/n_non_crossings", n_non_crossings);

    double p_crossing;
    n.getParam("p_crossing", p_crossing);

    std::stringstream ss;

    ss << "N:" << n_crossings << " M:" << n_non_crossings << " M+N:" << n_crossings + n_non_crossings << " ~p:" << double(n_crossings) / (n_crossings + n_non_crossings) << " Avg dist between pedestrians:" << car_x / (n_non_crossings + n_crossings) << " Probability:" << p_crossing << " TREE QP avg cost:" << cost << " velocity(m/s):" << velocity << " time(ms):" << time * 1000 << " " << i << " iterations";

    ROS_INFO_STREAM(ss.str());
    ofs << ss.str() << std::endl;
}

std::string filename(const std::string & name, double p_crossing, int n_pedestrians, int n_branches)
{
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/camille/Phd/Paper/ICRA-2021/plots/gen/data-" << std::to_string(p_crossing) << "-" << name << "-" << std::to_string(n_pedestrians) << "-" << std::to_string(n_branches) << "-" << t << ".txt";
    return ss.str();
}
