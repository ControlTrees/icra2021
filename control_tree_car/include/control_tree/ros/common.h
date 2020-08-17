#pragma once

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <ros/node_handle.h>

struct RoadModelBuilder
{
    RoadModelBuilder(double map_x, double road_width)
        : map_x(map_x)
        , road_width(road_width)
    {

    }

    RoadModelBuilder& add_center_line();

    RoadModelBuilder& add_road_border();

    visualization_msgs::MarkerArray build() { return markers; }

    visualization_msgs::MarkerArray markers;
    double map_x;
    double road_width;
};
