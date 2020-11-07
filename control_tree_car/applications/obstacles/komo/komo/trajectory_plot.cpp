#include <komo/trajectory_plot.h>

double radius(const visualization_msgs::MarkerArray::Ptr & obstacles)
{
    if(obstacles->markers.size() < 2)
        return 0.0;

    const auto&m = obstacles->markers[1];

    std::cout << m.scale.x << std::endl;

    return m.scale.x / 2;;
}

void plot(const std::vector<nav_msgs::Path> & trajectories,
          const visualization_msgs::MarkerArray::Ptr & obstacles,
          const PlotAxis & axis)
{
    TrajectoryPlot plotter(trajectories.size(), axis.name, axis.range);
    plotter.update(trajectories, obstacles);
}

void TrajectoryPlot::update(const std::vector<nav_msgs::Path> & trajectories, const visualization_msgs::MarkerArray::Ptr & obstacles)
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

    gp << "set size ratio -1" << "\n";
    gp << "set title '" << name_ << "'\n";
    gp << "set xrange [-10:50]\nset yrange " << yrange_ << "\n";
    gp << "plot ";

    for(auto i = 0; i < trajectories.size(); ++i)
    {
        gp << "'-' with lines title '" << name_ << "-" << i << "'";
        gp << " lc rgb '"<< color_code(i, 1.0) <<"'";

        gp << ",";
    }

    gp << "'-' with circles" << "\n";

    // send trajs
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

    // send obstacles
    std::vector<std::tuple<double, double, double> > xyr_pts;
    for(auto i = 0; i < obstacles->markers.size() / 2; ++i)
    {
      const auto&m = obstacles->markers[2 * i + 1];

      /// position and geometry
      auto x = m.pose.position.x;
      auto y = m.pose.position.y;
      auto r = m.scale.x / 2;

      //std::cout << "radius:" << r << std::endl;

      xyr_pts.push_back(std::make_tuple(x, y, r));
    }

    gp.send1d(xyr_pts);
}
