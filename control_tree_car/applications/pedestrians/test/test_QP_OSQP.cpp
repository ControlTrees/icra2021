#include <qp/QP_tree_problem_OSQP.h>

#include <chrono>

#include <common/control_tree.h>
#include <qp/control_tree_plot.h>
#include "common/qp_problems.h"

#include <gtest/gtest.h>


TEST_F(QPTest, test_2_branches_2_steps_constrained)
{
    auto pb = create_2_branches_2_steps_constrained();

    plan_OSQP(pb, false);
}

TEST_F(QPTest, test_2_branches_4_steps_constrained)
{
    auto pb = create_2_branches_4_steps_constrained();

    plan_OSQP(pb, false);
}

TEST_F(QPTest, test_3_branches_4_steps_constrained)
{
    auto pb = create_3_branches_4_steps_constrained();

    plan_OSQP(pb, false);
}

TEST_F(QPTest, test_4_branches_4_steps_constrained)
{
    auto pb = create_4_branches_4_steps_constrained();

    plan_OSQP(pb, false);
}

TEST_F(QPTest, test_2_stages_branching)
{
    auto pb = create_2_stages_branching();

    plan_OSQP(pb, false);
}

// plots for paper
TEST_F(QPTest, test_paper_4_branches_4_steps_constrained)
{
    auto pb = create_paper_4_branches_4_steps_constrained(0.1);

    vel_axis.range = "[0:15]";
    acc_axis.range = "[-8:3]";

    plan_OSQP(pb, false, "/home/camille/Phd/Paper/ICRA-2021/plots/4_branches.dat");
}

TEST_F(QPTest, test_paper_1_branch_4_steps)
{
    auto pb = create_paper_1_branch_4_steps_constrained();

    vel_axis.range = "[0:15]";
    acc_axis.range = "[-8:3]";

    auto U = plan_OSQP(pb, false, "/home/camille/Phd/Paper/RSS/plots/1_branch.dat");
}

TEST_F(QPTest, test_paper_multi_probabilities)
{
    std::vector<double> ps{0.025, 0.05, 0.1, 0.25, 0.5, 0.75, 1.0};

    for(const auto p: ps)
    {
        auto pb = create_paper_4_branches_4_steps_constrained(p);

        vel_axis.range = "[0:15]";
        acc_axis.range = "[-8:3]";

        std::stringstream ss;
        ss << "/home/camille/Phd/Paper/ICRA-2021/plots/4_branches_" << p << ".dat";
        
        plan_OSQP(pb, false, ss.str());
    }
}


TEST_F(QPTest, test_10_branches)
{
    auto pb = create_10_branches_4_steps_constrained();

    plan_OSQP(pb, false);
}

TEST_F(QPTest, test_20_branches)
{
    auto pb = create_20_branches_4_steps_constrained();

    plan_OSQP(pb, false);
}

TEST_F(QPTest, test_20_branches_)
{
    std::ofstream file("/home/camille/Phd/Paper/ICRA-2021/plots/execution_time_osqp_20.dat");
    file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;
    for(auto i = 1; i <= 20; ++i)
    {
        auto pb = create_N_branches_4_steps_constrained(i);
        plan_OSQP(pb, false);
        file << "  " << i << " " << execution_time_ms << std::endl;
    }
}

//TEST_F(QPTest, test_N_branches)
//{
//    std::ofstream file("/home/camille/Phd/Paper/ICRA-2021/plots/execution_time_osqp_100.dat");
//    file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;
//    for(auto i = 1; i <= 100; ++i)
//    {
//        auto pb = create_N_branches_4_steps_constrained(i);
//        plan_OSQP(pb, false);
//        file << "  " << i << " " << execution_time_ms << std::endl;
//    }
//}

TEST_F(QPTest, test_paper_sparsity)
{
    auto to_file = [](const MatrixXd & m, const std::string & filepath)
    {
        std::ofstream of(filepath);

        for(auto i = 0; i < m.rows(); ++i)
        {
            for(auto j = 0; j < m.cols(); ++j)
            {
                of << m(i, j) << " ";
            }

            of << std::endl;
        }
    };

    {
    auto pb = create_N_branches_4_steps_constrained(3);

    QP_tree_problem_OSQP solver(pb.model, u_min, u_max);
    solver.solve(pb.x0, pb.xd, pb.k, pb.tree.n_steps, pb.tree.varss, pb.tree.scaless);

    to_file(solver.get_H(), "/home/camille/Phd/Paper/RSS/matrices/H_tree_3.txt");
    to_file(solver.get_KA(), "/home/camille/Phd/Paper/RSS/matrices/K_tree_3.txt");
    }

    {
    auto pb = create_N_branches_4_steps_constrained(1);

    QP_tree_problem_OSQP solver(pb.model, u_min, u_max);
    solver.solve(pb.x0, pb.xd, pb.k, pb.tree.n_steps, pb.tree.varss, pb.tree.scaless);

    to_file(solver.get_H(), "/home/camille/Phd/Paper/RSS/matrices/H_tree_1.txt");
    to_file(solver.get_KA(), "/home/camille/Phd/Paper/RSS/matrices/K_tree_1.txt");
    }
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

