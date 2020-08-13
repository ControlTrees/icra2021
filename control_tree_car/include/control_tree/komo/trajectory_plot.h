#pragma once

#include <control_tree/core/utility.h>
#include <control_tree/qp/control_tree.h>

#include "nav_msgs/Path.h"

#include "gnuplot-iostream.h"


struct PlotAxis
{
    std::string name;
    std::string range;
};

void plot(const std::vector<nav_msgs::Path> & trajectories,
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

    void update(const std::vector<nav_msgs::Path> & trajectories);

private:
    Gnuplot gp_;
    int n_branches_;
    std::string name_;
    std::string yrange_;
};
