#pragma once

#include <control_tree/qp/control_tree.h>

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
                const std::function<double(int i)> & value_provider)
    {
        auto color_code = [](int i, double p)->std::string
        {
            std::stringstream ss;
            ss << "#";
            ss << std::hex << int((1.0 - p) * 255);
            ss << "00";
            ss << "00";
            ss << "FF";

            return ss.str();
        };

        auto & gp = gp_;

        gp << "set title '" << name_ << "'\n";
        gp << "set xrange [0:16]\nset yrange " << yrange_ << "\n";
        gp << "plot ";

        for(auto i = 0; i < varss.size(); ++i)
        {
            gp << "'-' with lines title '" << name_ << "-" << i << "'";
            gp << " lc rgb '"<< color_code(i, scales[i].back()) <<"'";

            if(i < varss.size() - 1)
                gp << ",";
        }

        gp << "\n";


        // data
        std::vector<std::vector<std::pair<double, double> > > xy_pts_all;

        for(auto vars : varss)
        {
            std::vector<std::pair<double, double> > xy_pts;
            for(int i=0; i<vars.size(); i+=1)
            {
                double x = i;
                double y = value_provider(vars[i]);
                xy_pts.push_back(std::make_pair(x, y));
            }
            xy_pts_all.push_back(xy_pts);
        }


        for(auto xy_pts : xy_pts_all)
        {
            gp.send1d(xy_pts);
        }
    }

private:
    Gnuplot gp_;
    int n_branches_;
    std::string name_;
    std::string yrange_;
};
