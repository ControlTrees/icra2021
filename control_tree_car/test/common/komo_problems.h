#include <control_tree/core/behavior_manager.h>

#include <control_tree/komo/obstacle_avoidance_linear.h>
#include <control_tree/komo/obstacle_avoidance_tree.h>
#include <control_tree/komo/obstacle_avoidance_dec.h>
#include <control_tree/komo/trajectory_plot.h>

#include <gtest/gtest.h>

nav_msgs::Odometry::Ptr create_odo(double x, double y, double vx);
std_msgs::Float32::Ptr create_desired_speed(double v);
visualization_msgs::MarkerArray::Ptr create_obstacles(double x, double y, double alpha);

struct Scenario
{
  nav_msgs::Odometry::Ptr odo;
  std_msgs::Float32::Ptr desired_velocity;
  visualization_msgs::MarkerArray::Ptr obstacles;
};

Scenario create_scenario_1();
Scenario create_scenario_2();

template<typename T>
void plan_impl(const Scenario & scenario, BehaviorManager& manager, T& behavior, bool _plot, bool _plot_debug)
{
  manager.odometry_callback(scenario.odo);
  behavior->desired_speed_callback(scenario.desired_velocity);
  behavior->obstacle_callback(scenario.obstacles);

  auto plotter = [&]()
  {
      static int n = 0;

      PlotAxis axis{std::to_string(n) + " x/y", "[-4:3]"};
      plot(manager.get_trajectories(), axis);

      ++n;
  };

  if(_plot_debug)
  {
    behavior->set_optim_callback(plotter);
  }

  manager.plan();

  if(_plot)
  {
    plotter();
  }
}

class KomoLinearTest : public ::testing::Test
{
public:
  void SetUp()
  {
    ros::Time::init();

    behavior = std::make_shared<ObstacleAvoidanceLinear>(manager, 4);

    manager.register_behavior("collision_avoidance", behavior);
    manager.set_current_behavior("collision_avoidance");
  }

  void plan(const Scenario & scenario, bool plot, bool plot_debug = false)
  {
    plan_impl(scenario, manager, behavior, plot, plot_debug);
  }

  BehaviorManager manager;
  std::shared_ptr<ObstacleAvoidanceLinear> behavior;
};

class KomoJointTest : public ::testing::Test
{
public:
  void SetUp()
  {
    ros::Time::init();

    behavior = std::make_shared<ObstacleAvoidanceTree>(manager, 4);

    manager.register_behavior("collision_avoidance", behavior);
    manager.set_current_behavior("collision_avoidance");
  }

  void plan(const Scenario & scenario, bool plot, bool plot_debug = false)
  {
    plan_impl(scenario, manager, behavior, plot, plot_debug);
  }

  BehaviorManager manager;
  std::shared_ptr<ObstacleAvoidanceTree> behavior;
};

class KomoDecTest : public ::testing::Test
{
public:
  void SetUp()
  {
    ros::Time::init();

    behavior = std::make_shared<ObstacleAvoidanceDec>(manager, 4);

    manager.register_behavior("collision_avoidance", behavior);
    manager.set_current_behavior("collision_avoidance");
  }

  void plan(const Scenario & scenario, bool plot, bool plot_debug = false)
  {
    plan_impl(scenario, manager, behavior, plot, plot_debug);
  }

  BehaviorManager manager;
  std::shared_ptr<ObstacleAvoidanceDec> behavior;
};
