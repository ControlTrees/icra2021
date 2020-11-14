#pragma once

#include <common/behavior_base.h>

#include <math.h>
#include <boost/bind.hpp>
#include <chrono>
#include <memory>
#include <functional>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <KOMO/komo.h>
#include <komo/utility_komo.h>

#include <tree_builder.h>
#include <circular_obstacle.h>
#include <komo/komo_factory.h>

#include <Optimization/decentralized_optimizer.h>
#include <Optimization/decentralized_lagrangian.h>

class ObstacleAvoidanceDec : public BehaviorBase
{
public:
    ObstacleAvoidanceDec(BehaviorManager&, int n_obstacles, bool tree, double road_width, double v_desired, int steps_per_phase);

    void desired_speed_callback(const std_msgs::Float32::ConstPtr& msg);

    void obstacle_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);

    TimeCostPair plan();

    std::vector<nav_msgs::Path> get_trajectories();

    void set_optim_callback(const std::function<void()>& callback) { options_.callback = callback; }

    static uint n_branches(uint n_obstacles, bool tree) { return tree ? pow(2.0, n_obstacles) : 1; }
private:
    void init_tree();
    void update_groundings();
    void init_optimization_variable();

private:
    // params
    const uint n_obstacles_; // number of obstacles
    const uint n_branches_; // number of branches
    const bool tree_;
    const double road_width_;
    const uint horizon_; // planning horizon number of phases ~ [s]
    const uint steps_;

    // target: params than can be adapted
    double v_desired_;
    std::vector<Obstacle> obstacles_;

    // komo
    KomoFactory komo_factory_;

    // objectives
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
    std::vector<double> belief_state_;

    intA vars_branch_order_0_; // comressed var (hence all the same)
    intA vars_branch_order_1_;
    intA vars_branch_order_2_;
};

std::vector<double> fuse_probabilities(const std::vector<Obstacle>&, bool tree, std::vector<std::vector<bool>> &);
void convert(uint n_branches, uint horizon, mp::TreeBuilder& komo_tree_);

