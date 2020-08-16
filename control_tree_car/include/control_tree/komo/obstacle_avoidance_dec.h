#pragma once

#include <control_tree/core/behavior_base.h>

#include <math.h>
#include <boost/bind.hpp>
#include <chrono>
#include <memory>
#include <functional>

#include <Eigen/Dense>

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <KOMO/komo.h>
#include <komo_wrapper.h>

#include <tree_builder.h>
#include <car_kinematic.h>
#include <velocity.h>
#include <axis_bound.h>
#include <circular_obstacle.h>

#include <Optimization/decentralized_optimizer.h>
#include <Optimization/decentralized_lagrangian.h>

struct Obstacle
{
  arr position;
  double p;
};

class ObstacleAvoidanceDec : public BehaviorBase
{
public:
    ObstacleAvoidanceDec(BehaviorManager&, int n_obstacles, int steps_per_phase);

    void desired_speed_callback(const std_msgs::Float32::ConstPtr& msg);

    void obstacle_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);

    TimeCostPair plan();

    std::vector<nav_msgs::Path> get_trajectories();

    void set_optim_callback(const std::function<void()>& callback) { options_.callback = callback; }

    static uint n_branches(uint n_obstacles) { return pow(2.0, n_obstacles); }
private:
    void init_tree();
    void update_groundings();
    void init_optimization_variable();

private:
    // params
    const uint n_obstacles_; // number of obstacles
    const uint n_branches_; // number of branches
    rai::KinematicWorld kin_;
    const uint steps_;

    // target: params than can be adapted
    double v_desired_;
    std::vector<Obstacle> obstacles_;

    // objectives
    struct Objectives
    {
      Objective * acc_{0};
      Objective * ax_{0};
      Objective * vel_{0};
      Objective * car_kin_{0};
      std::shared_ptr<Car3CirclesCircularObstacle> circular_obstacle_;
      Objective * collision_avoidance_{0};

      void apply_scales(const arr& scales);
    };

    std::vector<Objectives> objectivess_;

    // state;
    mp::TreeBuilder komo_tree_;

    std::vector<std::shared_ptr<KOMO>> komos_;
    std::vector<std::shared_ptr<KOMO::Conv_MotionProblem_GraphProblem>> converters_;
    std::vector<std::shared_ptr<ConstrainedProblem>> constrained_problems_;
    DecOptConfig options_;

    arr x_;
    std::vector<arr> xmasks_;
    std::vector<intA> vars_; // uncompressed vars (dual of xmasks)

    intA vars_branch_order_0_; // comressed var (hence all the same)
    intA vars_branch_order_1_;
    intA vars_branch_order_2_;
};

std::vector<double> fuse_probabilities(const std::vector<Obstacle>&, std::vector<std::vector<bool>> &);
void convert(uint n_branches, mp::TreeBuilder& komo_tree_);

