#pragma once

#include <common/utility.h>
#include <common/control_tree.h>

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include "gnuplot-iostream.h"


struct PlotAxis
{
    std::string name;
    std::string range;
};

void plot(const std::vector<nav_msgs::Path> & trajectories,
          const visualization_msgs::MarkerArray::Ptr & obstacles,
          const PlotAxis & axis);

class TrajectoryPlot
{
public:
    TrajectoryPlot(int n_branches, const std::string & name, const std::string & yrange)
        : n_branches_(n_branches)
        , name_(name)
        , yrange_(yrange)
    {
    }

    void update(const std::vector<nav_msgs::Path> & trajectories, const visualization_msgs::MarkerArray::Ptr & obstacles);

private:
    Gnuplot gp_;
    int n_branches_;
    std::string name_;
    std::string yrange_;
};
