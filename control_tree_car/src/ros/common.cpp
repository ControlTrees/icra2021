#include <control_tree/ros/common.h>

#include <fstream>

#include <ros/package.h>
#include <ros/console.h>

#include <chrono>

namespace
{
visualization_msgs::Marker create_center_line(double map_x)
{
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.id = std::hash<std::string>()("centerline");
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

visualization_msgs::Marker create_road_border(double map_x)
{
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.id = std::hash<std::string>()("borders");
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
}

RoadModelBuilder& RoadModelBuilder::add_center_line()
{
    markers.markers.push_back(create_center_line(map_x));
    return *this;
}

RoadModelBuilder& RoadModelBuilder::add_road_border()
{
    markers.markers.push_back(create_road_border(map_x));
    return *this;
}


