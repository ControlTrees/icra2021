#include <control_tree/ros/obstacle_common.h>
#include <control_tree/core/utility.h>

#include <fstream>

#include <ros/package.h>
#include <ros/console.h>

#include <chrono>

void log_to_file(std::ofstream & ofs, ros::NodeHandle & n, double car_x, int i, double cost)
{
    int n_obstacles;
    int n_non_obstacles;
    double p_obstacle;
    double planning_time = 0;

    n.getParam("/n_total_obstacles", n_obstacles);
    n.getParam("/n_total_non_obstacles", n_non_obstacles);
    n.getParam("/p_obstacle", p_obstacle);
    n.getParam("/planning_time", planning_time);

    std::stringstream ss;

    ss << "N:" << n_obstacles << " M:" << n_non_obstacles << " M+N:"<< n_obstacles + n_non_obstacles << " ~p:" << double(n_obstacles) / (n_non_obstacles+n_obstacles) << " Travelled dist:" << car_x << " Probability:" << p_obstacle << " Avg cost:" << cost << " time(ms):" << planning_time * 1000 << " " << i << " iterations";

    ROS_INFO_STREAM(ss.str());
    ofs << ss.str() << std::endl;
}

std::string filename(ros::NodeHandle & n)
{
    double p_obstacle = 0.1;
    int n_obstacles = 1;
    bool tree = true;

    n.getParam("tree", tree);
    n.getParam("p_obstacle", p_obstacle);
    n.getParam("n_obstacles", n_obstacles);
    n.getParam("tree", tree);

    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/home/camille/Phd/Paper/ICRA-2021/plots/gen_obstacles/data-" << std::to_string(p_obstacle) << "-" << (tree ? "tree" : "linear") << "-" << n_obstacles << "-" << t << ".txt";
    return ss.str();
}

visualization_msgs::Marker create_obstacle_marker(double x, double y, double sx, double sy, double sz, double alpha, int id)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.id = std::hash<std::string>()("obstacle_" + std::to_string(id));
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.5 * sz;
    marker.scale.x = sx; // diameter
    marker.scale.y = sy;
    marker.scale.z = sz;
    marker.pose.orientation = get_quaternion_from_yaw(x);
    marker.color.a = alpha;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
}

//visualization_msgs::Marker create_ellipsoid_collision_marker(double x, double y, double sx, double sy, double sz, double alpha, int id)
//{
//    visualization_msgs::Marker marker;

//    const double m = 0.5; // margin
//    double d = sx * sx / (8 * m) - sy / 2 + m / 2;
//    double diameter = 2 * (d + sy / 2 + m);

//    marker.header.frame_id = "map";
//    marker.id = std::hash<std::string>()("collision_" + std::to_string(id));
//    marker.type = visualization_msgs::Marker::CYLINDER;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose.position.x = x;
//    marker.pose.position.y = y + (y > 0 ? d : -d);
//    marker.pose.position.z = 0.5 * 0.01;
//    marker.scale.x = diameter; // diameter
//    marker.scale.y = diameter;
//    marker.scale.z = 0.01;
//    marker.color.a = alpha > 0.01 ? alpha : 0.0;
//    marker.color.r = 1.0;
//    marker.color.g = 0.0;
//    marker.color.b = 0.0;

//    //std::cout << "diameter:" << diameter << std::endl;

//    return marker;
//}

visualization_msgs::Marker create_collision_marker(double x, double y, double sx, double sy, double sz, double alpha, int id)
{
    visualization_msgs::Marker marker;

    const double m = 0.5; // margin
    double d = sx * sx / (8 * m) - sy / 2 + m / 2;
    double diameter = 2 * (d + sy / 2 + m);

    marker.header.frame_id = "map";
    marker.id = std::hash<std::string>()("collision_" + std::to_string(id));
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y + (y > 0 ? d : -d);
    marker.pose.position.z = 0.5 * 0.01;
    marker.scale.x = diameter; // diameter
    marker.scale.y = diameter;
    marker.scale.z = 0.01;
    marker.color.a = alpha > 0.01 ? alpha : 0.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    return marker;
}

//nav_msgs::Path transform(const nav_msgs::Path & trajectory, double tx)
//{
//    nav_msgs::Path transformed;
//    transformed.header = trajectory.header;

//    for(auto i = 0; i < trajectory.poses.size(); ++i)
//    {
//        const auto pose = trajectory.poses[i];
//        const auto x = pose.pose.position.x;
//        const auto y = pose.pose.position.y;
//        const auto yaw = get_yaw_from_quaternion(pose.pose.orientation);

//        geometry_msgs::PoseStamped transformed_pose;
//        //transformed_pose.header = pose.header;
//        transformed_pose.pose.position.x = x + tx * cos(yaw);
//        transformed_pose.pose.position.y = y + tx * sin(yaw);
//        transformed_pose.pose.orientation = pose.pose.orientation;

//        transformed.poses.push_back(transformed_pose);
//    }

//    return transformed;
//}

//std::vector<nav_msgs::Path> transform(const std::vector<nav_msgs::Path> & trajectories, double tx)
//{
//    std::vector<nav_msgs::Path> transformed;
//    transformed.reserve(trajectories.size());

//    for(const auto & trajectory: trajectories)
//    {
//        transformed.push_back(transform(trajectory, tx));
//    }

//    return transformed;
//}
