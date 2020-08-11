#include <chrono>

#include <control_tree/komo/obstacle_avoidance_tree.h>
#include <control_tree/core/behavior_manager.h>

#include <gtest/gtest.h>

TEST(KOMO, Foo)
{
  ros::Time::init();

  BehaviorManager manager;

  auto ca = std::make_shared<ObstacleAvoidanceTree>(manager, 4);

  manager.register_behavior("collision_avoidance", ca);
  manager.set_current_behavior("collision_avoidance");

  /// SET MESSAGES
  // create odo
  nav_msgs::Odometry::Ptr odo(new nav_msgs::Odometry());
  odo->pose.pose.position.x = 0;
  odo->pose.pose.position.y = 0;
  odo->pose.pose.position.z = 0;

  odo->pose.pose.orientation.w = 1;
  odo->pose.pose.orientation.x = 0;
  odo->pose.pose.orientation.y = 0;
  odo->pose.pose.orientation.z = 0;

  odo->twist.twist.linear.x = 0;
  odo->twist.twist.linear.y = 0;
  odo->twist.twist.linear.z = 0;

  manager.odometry_callback(odo);

  // create desired speed
  std_msgs::Float32::Ptr desired_speed(new std_msgs::Float32());
  desired_speed->data = 10;

  ca->desired_speed_callback(desired_speed);

  // create obstacles
  visualization_msgs::Marker::Ptr obstacles(new visualization_msgs::Marker());

  obstacles->pose.position.x = 50;
  obstacles->color.a = 0.5;

  ca->obstacle_callback(obstacles);

  ///PLAN
  manager.plan();

  ///CHECK RESULTS
  const auto trajectories = manager.get_trajectories();
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

