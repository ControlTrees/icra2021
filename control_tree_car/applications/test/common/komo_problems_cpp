#include "komo_problems.h"

nav_msgs::Odometry::Ptr create_odo(double x, double y, double vx)
{
  nav_msgs::Odometry::Ptr odo(new nav_msgs::Odometry());
  odo->pose.pose.position.x = x;
  odo->pose.pose.position.y = y;
  odo->pose.pose.position.z = 0;

  odo->pose.pose.orientation.w = 1;
  odo->pose.pose.orientation.x = 0;
  odo->pose.pose.orientation.y = 0;
  odo->pose.pose.orientation.z = 0;

  odo->twist.twist.linear.x = vx;
  odo->twist.twist.linear.y = 0;
  odo->twist.twist.linear.z = 0;

  return odo;
}

std_msgs::Float32::Ptr create_desired_speed(double v)
{
  std_msgs::Float32::Ptr desired_speed(new std_msgs::Float32());
  desired_speed->data = 10;

  return desired_speed;
}

struct ObstaclesBuilder
{
  ObstaclesBuilder()
    : obstacles(new visualization_msgs::MarkerArray())
  {

  }

  ObstaclesBuilder& add(double x, double y, double alpha)
  {
    visualization_msgs::Marker obstacle;

    obstacle.pose.position.x = x;
    obstacle.pose.position.y = y;
    obstacle.scale.x = 2;
    obstacle.color.a = alpha;

    obstacles->markers.push_back(obstacle); // obstacle
    obstacles->markers.push_back(obstacle); // obstacle belief

    return *this;
  }

  visualization_msgs::MarkerArray::Ptr build() const { return obstacles;}

  visualization_msgs::MarkerArray::Ptr obstacles;
};

Scenario create_scenario_1(double p)
{
  const auto odo = create_odo(0, 0, 10);
  const auto desired_velocity = create_desired_speed(10);
  const auto obstacles = ObstaclesBuilder().add(20, 1, p).build();

  return {odo, desired_velocity, obstacles};
}

Scenario create_scenario_2()
{
  const auto odo = create_odo(0, 0, 10);
  const auto desired_velocity = create_desired_speed(10);
  const auto obstacles = ObstaclesBuilder().add(15, 1, 0.02).add(22.5, 0.1, 0.02).build();

  return {odo, desired_velocity, obstacles};
}

Scenario create_scenario_3()
{
  const auto odo = create_odo(0, 0, 10);
  const auto desired_velocity = create_desired_speed(10);
  const auto obstacles = ObstaclesBuilder().add(20, 1, 0.5).add(25, -1, 0.3).add(100, 1.0, 0.1).build();

  return {odo, desired_velocity, obstacles};
}
