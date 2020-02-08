#include <control_tree/qp/QP_tree_problem_CGAL.h>

#include <chrono>

#include <control_tree/qp/control_tree.h>

#include <gtest/gtest.h>

#include <control_tree/qp/control_tree_plot.h>

#include "common/qp_problems.h"

using namespace std;

typedef QP_tree_problem_CGAL QP_solver_type;

TEST_F(QPTest, test_2_branches_2_steps_constrained)
{
    auto pb = create_2_branches_2_steps_constrained();

    plan_CGAL(pb, false);
}

TEST_F(QPTest, test_2_branches_4_steps_constrained)
{
    auto pb = create_2_branches_4_steps_constrained();

    plan_CGAL(pb, false);
}

TEST_F(QPTest, test_3_branches_4_steps_constrained)
{
    auto pb = create_3_branches_4_steps_constrained();

    plan_CGAL(pb, false);
}

TEST_F(QPTest, test_4_branches_4_steps_constrained)
{
    auto pb = create_4_branches_4_steps_constrained();

    plan_CGAL(pb, true);
}

TEST(QP_tree_problem, test_stop_late)
{
    // o.x:336.262 o.v:9.40064 stop_position_(0):347.743 existence_probability_:0.18843
    // o.x:502.03 o.v:4.20352 stop_position_(0):508.049 existence_probability_:0.101338 v_desired_:18
    // o.x:269.387 o.v:11.8263 stop_position_(0):291.77 existence_probability_:0.18843 v_desired_:15
    const auto x0 = 0;
    const auto v = 10;
    const auto xmax = 10;
    const auto existence_probability = 0.8;
    const auto v_desired = 15;
    const auto steps_per_phase = 4;

    auto tree = TreePb::refined(Tree2Branches(existence_probability), steps_per_phase);

    MPC_model model(1.0 / steps_per_phase, 1.0, 15.0);
    Constraints k(tree.n_steps, tree.varss);
    k.add_constraint(0, Vector2d(15, 20), Vector2d(1, 1));
    k.add_constraint(1, Vector2d(100, 20), Vector2d(1, 1));
    QP_solver_type solver(model, -6.0, 2.0);

    // INITIAL STATES
    Vector2d X0;
    X0 << x0, xmax;

    Vector2d xd;
    xd << 0, v_desired;

    // SOLVE
    auto start = std::chrono::high_resolution_clock::now();

    auto U = solver.solve(X0, xd, k, tree.n_steps, tree.varss, tree.scaless);

    auto end = std::chrono::high_resolution_clock::now();
    float execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    std::cout << "Execution time (ms):" << execution_time_us / 1000 << std::endl;
    std::cout << "U:\n" << U << std::endl;

    if(U.rows())
    {
        auto X = model.predict_trajectory(X0, U, tree.varss);

        std::cout << "X:\n" << X << std::endl;

        // plot
        plot([&U](int i){return U(i);}, tree.varss, tree.scaless, {"acceleration", "[-7:2]"});
        plot([&X](int i){return X(2*i+1);}, tree.varss, tree.scaless, {"velocity", "[0:20]"});
        plot([&X](int i){return X(2*i);}, tree.varss, tree.scaless, {"x", "[0:50]"});
    }
    else
    {
        std::cout << "Infeasible:/\n" << std::endl;
    }
}

TEST(QP_tree_problem, test_constrained_tree_2_branches_infeasible)
{
    auto n_steps_per_phase = 4;

    auto tree = TreePb::refined(Tree2Branches(0.6), n_steps_per_phase);

    MPC_model model(1.0 / n_steps_per_phase, 1.0, 3.0);
    Constraints k(tree.n_steps, tree.varss);
    k.add_constraint(0, Vector2d(-1, 0), Vector2d(1, 0));
    QP_solver_type solver(model, -3.0, 2.0);

    // INITIAL STATES
    Vector2d X0;
    X0 << 0, 5.0;

    Vector2d xd;
    xd << 0, 8.0;

    // SOLVE

    auto start = std::chrono::high_resolution_clock::now();

    auto U = solver.solve(X0, xd, k, tree.n_steps, tree.varss, tree.scaless);

    auto end = std::chrono::high_resolution_clock::now();
    float execution_time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    std::cout << "Execution time (ms):" << execution_time_us / 1000 << std::endl;

    EXPECT_EQ(U.rows(), 0);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

