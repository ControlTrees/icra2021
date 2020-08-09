#include <control_tree/qp/control_tree_plot.h>

void plot(const std::function<double(int i)> & value_provider,
          const std::vector<IntA> & varss,
          const std::vector<Arr> & scaless,
          const PlotAxis & axis)
{
    TreePlot plotter(varss.size(), axis.name, axis.range);
    plotter.update(varss, scaless, value_provider);
}

void save(const std::function<double(int i)> & value_provider,
          double dt,
          const std::vector<IntA> & varss,
          const std::vector<Arr> & scales,
          std::ostream & file)
{
    for(const auto & vars : varss)
    {
        save(value_provider, dt, vars, file);
    }

    file << std::endl;
}


void save(const std::function<double(int i)> & value_provider,
          double dt,
          const IntA & vars, std::ostream & file)
{
    for(int i=0; i<vars.size(); i+=1)
    {
        auto x = i * dt;
        auto y = value_provider(vars[i]);
        file << "  " << x << " " << y << std::endl;
    }

    file << std::endl;
    file << std::endl;
}
