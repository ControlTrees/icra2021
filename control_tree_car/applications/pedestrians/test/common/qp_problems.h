#pragma once

#include <common/control_tree.h>
#include <qp/MPC_model.h>
#include <qp/QP_constraints.h>
#include <qp/QP_tree_solver_base.h>
#include <qp/control_tree_plot.h>

#include <gtest/gtest.h>

struct QP_problem
{
    MPC_model model;
    TreePb tree;
    Constraints k;
    Vector2d x0;
    Vector2d xd;
};

QP_problem create_2_branches_2_steps_constrained(double p=0.6);
QP_problem create_2_branches_4_steps_constrained(double p=0.6);
QP_problem create_3_branches_4_steps_constrained(double p=0.6);
QP_problem create_4_branches_4_steps_constrained(double p=0.6);
QP_problem create_10_branches_4_steps_constrained();
QP_problem create_20_branches_4_steps_constrained();
QP_problem create_N_branches_4_steps_constrained(int N);
QP_problem create_2_stages_branching(double p=0.6);
// paper plots
QP_problem create_paper_1_branch_4_steps_constrained(double p=0.6);
QP_problem create_paper_4_branches_4_steps_constrained(double p=0.6);
QP_problem replicate_simulation_1();

class QPTest : public ::testing::Test
{
public:
    double execution_time_ms{0};

 protected:
    VectorXd plan_OSQP(const QP_problem &pb, bool plot = false, const std::string & filename = "");
    VectorXd plan_JointQP(const QP_problem &pb, bool plot = false, const std::string & filename = "");
    VectorXd plan_DecQP(const QP_problem &pb, bool plot = false, const std::string & filename = "");

    void plot_XU(const VectorXd& X, const VectorXd& U, const QP_problem &pb) const;
    void save_XU(const VectorXd& X, const VectorXd& U, const QP_problem &pb, const std::string & filename) const;

    PlotAxis acc_axis{"acceleration", "[-4:3]"};
    PlotAxis vel_axis{"velocity", "[0:10]"};
    PlotAxis x_axis{"x", "[0:50]"};

    double u_min{-6.0};
    double u_max{2.0};
};

