#include <control_tree/komo/trajectory_plot.h>

void plot(const std::vector<nav_msgs::Path> & trajectories,
          const PlotAxis & axis)
{
    TrajectoryPlot plotter(trajectories.size(), axis.name, axis.range);
    plotter.update(trajectories);
}

void TrajectoryPlot::update(const std::vector<nav_msgs::Path> & trajectories)
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
    gp << "set xrange [-10:50]\nset yrange " << yrange_ << "\n";
    gp << "plot ";

    for(auto i = 0; i < trajectories.size(); ++i)
    {
        gp << "'-' with lines title '" << name_ << "-" << i << "'";
        gp << " lc rgb '"<< color_code(i, 1.0) <<"'";

        if(i < trajectories.size() - 1)
            gp << ",";
    }

    gp << "\n";


    // data
    std::vector<std::vector<std::pair<double, double> > > xy_pts_all;

    for(const auto& traj : trajectories)
    {
        std::vector<std::pair<double, double> > xy_pts;
        for(int i=0; i<traj.poses.size(); i+=1)
        {
            const auto& x = traj.poses[i].pose.position.x;
            const auto& y = traj.poses[i].pose.position.y;
            xy_pts.push_back(std::make_pair(x, y));
        }
        xy_pts_all.push_back(xy_pts);
    }


    for(auto xy_pts : xy_pts_all)
    {
        gp.send1d(xy_pts);
    }
}
