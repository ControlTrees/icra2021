#pragma once

#include <control_tree/core/behavior_base.h>

#include <math.h>
#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include <control_tree/core/control_tree.h>

#include <visualization_msgs/MarkerArray.h>

#include <control_tree/qp/MPC_model.h>
#include <control_tree/qp/control_tree_plot.h>
#include <control_tree/qp/QP_tree_problem_CGAL.h>
#include <control_tree/qp/QP_tree_problem_OSQP.h>
#include <control_tree/qp/QP_tree_problem_DecQP.h>

//typedef QP_tree_problem_OSQP QP_tree_problem_solver_type;
//typedef QP_tree_problem_JointQP QP_tree_problem_solver_type;
typedef QP_tree_problem_DecQP QP_tree_problem_solver_type;

struct Stopline
{
    double x;
    double p;
};

class StopLineQPTree : public BehaviorBase
{
public:
    StopLineQPTree(BehaviorManager&, int n_pedestrians, int steps_per_phase);

    void desired_speed_callback(const std_msgs::Float32::ConstPtr& msg);

    void stopline_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);

    TimeCostPair plan();

    std::vector<nav_msgs::Path> get_trajectories();

private:
    void create_tree();
    bool valid(const VectorXd & U, const VectorXd & X) const;

private:
    // params
    const int n_branches_;
    const uint steps_;
    double u_min_{-8.0};
    double u_max_{2.0};

    // target: params than can be adapted
    double v_desired_;
    std::vector<Stopline> stoplines_;

    // tree
    std::shared_ptr<TreePb> tree_;
    MPC_model model_;
    QP_tree_problem_solver_type solver_;

    // results
    Vector2d x0_;
    VectorXd U_sol_;
    VectorXd X_sol_;

    // state;
    bool optimization_run_;
    bool optimization_error_;

    // plot
    TreePlot acc_plotter_;
    TreePlot vel_plotter_;
};

VectorXd emergency_brake(const double v, const TreePb &, int steps_per_phase, double u);
std::vector<double> fuse_probabilities(const std::vector<Stopline> &, int n); // n = n_branches -1

