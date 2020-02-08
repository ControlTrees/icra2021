#pragma once

#include <control_tree/core/behavior_base.h>

#include <math.h>
#include <boost/bind.hpp>
#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <visualization_msgs/Marker.h>

#include <KOMO/komo.h>

#include <car_kinematic.h>
#include <velocity.h>
#include <axis_bound.h>
#include <circular_obstacle.h>

#include <control_tree/komo/velocity_axis.h>


class ObstacleAvoidanceLinear : public BehaviorBase
{
public:
    ObstacleAvoidanceLinear(BehaviorManager&, int steps_per_phase);

    void desired_speed_callback(const std_msgs::Float32::ConstPtr& msg);

    void obstacle_callback(const visualization_msgs::Marker::ConstPtr& msg);

    TimeCostPair plan();

    std::vector<nav_msgs::Path> get_trajectories();

private:
    // params
    rai::KinematicWorld kin_;
    const uint steps_;

    // target: params than can be adapted
    double v_desired_;
    double existence_probability_;
    arr obstacle_position_;

    // functional
    std::shared_ptr<Car3CirclesCircularObstacle> circular_obstacle_;

    // objectives
    Objective * acc_;
    Objective * ax_;
    Objective * vel_;
    Objective * car_kin_;
    Objective * collision_avoidance_;

    std::shared_ptr<KOMO> komo_;
};
