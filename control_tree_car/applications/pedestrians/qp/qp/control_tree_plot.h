#pragma once

#include <common/control_tree.h>

#include "gnuplot-iostream.h"

struct PlotAxis
{
    std::string name;
    std::string range;
};

void plot(const std::function<double(int i)> & value_provider,
          const std::vector<IntA> & varss,
          const std::vector<Arr> & scales,
          const PlotAxis & axis);

void save(const std::function<double(int i)> & value_provider,
          double dt,
          const std::vector<IntA> & varss,
          const std::vector<Arr> & scales, std::ostream & file);

void save(const std::function<double(int i)> & value_provider,
          double dt,
          const IntA & vars,
          std::ostream & file);


class TreePlot
{
public:
    TreePlot(int n_branches, const std::string & name, const std::string & yrange)
        : n_branches_(n_branches)
        , name_(name)
        , yrange_(yrange)
    {
    }

    void update(const std::vector<IntA> & varss,
                const std::vector<Arr> & scales,
                const std::function<double(int i)> & value_provider);

private:
    Gnuplot gp_;
    int n_branches_;
    std::string name_;
    std::string yrange_;
};
