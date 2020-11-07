#pragma once

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

double rand_01();
double rand_m11();
double draw_p(double median_p);
bool draw_bool(double average_p);

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
