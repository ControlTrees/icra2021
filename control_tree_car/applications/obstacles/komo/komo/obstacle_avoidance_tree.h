#pragma once

#include <common/behavior_base.h>

#include <math.h>
#include <boost/bind.hpp>
#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <KOMO/komo.h>

#include <tree_builder.h>
#include <car_kinematic.h>
#include <velocity.h>
#include <axis_bound.h>
#include <circular_obstacle.h>

#include <visualization_msgs/MarkerArray.h>

class ObstacleAvoidanceTree : public BehaviorBase
{
public:
    ObstacleAvoidanceTree(BehaviorManager&, int steps_per_phase);

    void desired_speed_callback(const std_msgs::Float32::ConstPtr& msg);

    void obstacle_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);

    TimeCostPair plan();

    std::vector<nav_msgs::Path> get_trajectories();

    void set_optim_callback(const std::function<void()>& _) {}

private:
    void update_tree(double p);

private:
    // params
    rai::KinematicWorld kin_;
    const uint steps_;

    // target: params than can be adapted
    double v_desired_;
    double existence_probability_;
    std::vector<Obstacle> obstacles_;

    // functional
    std::shared_ptr<Car3CirclesCircularObstacle> circular_obstacle_;

    // objectives
    Objective * acc_;
    Objective * ax_;
    Objective * vel_;
    Objective * car_kin_;
    Objective * collision_avoidance_;

    // state;
    mp::TreeBuilder tree_;
    intA vars_branch_1_order_0_;
    intA vars_branch_1_order_1_;
    intA vars_branch_1_order_2_;
    intA vars_branch_2_order_0_;
    intA vars_branch_2_order_1_;
    intA vars_branch_2_order_2_;
    intA vars_all_order_0_;
    intA vars_all_order_1_;
    intA vars_all_order_2_;

    arr scales_all_;

    std::shared_ptr<KOMO> komo_;
};


