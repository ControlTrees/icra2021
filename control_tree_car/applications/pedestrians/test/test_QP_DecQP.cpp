//#include <control_tree/qp/QP_tree_problem_DecQP.h>

#include <chrono>

#include <qp/control_tree_plot.h>
#include <qp/QP_tree_problem_DecQP.h>
#include "common/qp_problems.h"

#include <common/control_tree.h>

#include <gtest/gtest.h>

TEST_F(QPTest, JOINT_test_2_branches_4_steps_constrained)
{
    auto pb = create_2_branches_4_steps_constrained();

    plan_JointQP(pb, false);
}

TEST_F(QPTest, JOINT_test_paper_1_branch_4_steps)
{
    auto pb = create_paper_1_branch_4_steps_constrained();

    vel_axis.range = "[0:15]";
    acc_axis.range = "[-8:3]";

    auto U = plan_JointQP(pb, false, "/tmp/1_branch.dat");//, "/tmp/1_branch.dat");

    //auto U = plan_JointQP(pb, false, "/home/camille/Phd/Paper/ICRA-2021/plots/1_branch.dat");//, "/tmp/1_branch.dat");
}

TEST_F(QPTest, DEC_paper_4_branches_4_steps_constrained)
{
    auto pb = create_paper_4_branches_4_steps_constrained(0.1);

    vel_axis.range = "[0:15]";
    acc_axis.range = "[-8:3]";

    plan_DecQP(pb, false, "/tmp/4_branches.dat");

    //plan_DecQP(pb, false, "/home/camille/Phd/Paper/ICRA-2021/plots/4_branches.dat");
}

TEST_F(QPTest, DEC_test_paper_1_branch_4_steps)
{
    auto pb = create_paper_1_branch_4_steps_constrained();

    vel_axis.range = "[0:15]";
    acc_axis.range = "[-8:3]";

    plan_DecQP(pb, false);//, "/tmp/1_branch.dat");
    // plan_DecQP(pb, false);//, "/home/camille/Phd/Paper/ICRA-2021/plots/1_branch.dat");
}

TEST_F(QPTest, DEC_test_paper_multi_probabilities)
{
    std::vector<double> ps{0.025, 0.05, 0.1, 0.25, 0.5, 0.75, 1.0};

    for(const auto p: ps)
    {
        auto pb = create_paper_4_branches_4_steps_constrained(p);

        vel_axis.range = "[0:15]";
        acc_axis.range = "[-8:3]";

        std::stringstream ss;
        //ss << "/home/camille/Phd/Paper/ICRA-2021/plots/4_branches_" << p << ".dat";
        ss << "/tmp/4_branches_" << p << ".dat";

        plan_DecQP(pb, false, ss.str());
    }
}

TEST_F(QPTest, DEC_test_2_branches_4_steps_constrained)
{
  auto pb = create_2_branches_4_steps_constrained(0.1);

  plan_DecQP(pb, false);
}

TEST_F(QPTest, test_3_branches_4_steps_constrained)
{
    auto pb = create_3_branches_4_steps_constrained();

    plan_DecQP(pb, false);
}

TEST_F(QPTest, test_4_branches_4_steps_constrained)
{
    auto pb = create_4_branches_4_steps_constrained();

    plan_DecQP(pb, false);
}

TEST_F(QPTest, test_replicate_simulation_1)
{
    auto pb = replicate_simulation_1();

    plan_DecQP(pb, true);
}

TEST_F(QPTest, test_20_branches_)
{
    std::ofstream file("/tmp/execution_time_dec_qp_20.dat");

    //std::ofstream file("/home/camille/Phd/Paper/ICRA-2021/plots/execution_time_dec_qp_20.dat");
    file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;
    for(auto i = 1; i <= 20; ++i)
    {
        auto pb = create_N_branches_4_steps_constrained(i);
        plan_DecQP(pb, false);
        file << "  " << i << " " << execution_time_ms << std::endl;
    }
}

//TEST_F(QPTest, test_N_branches)
//{
//    std::ofstream file("/home/camille/Phd/Paper/ICRA-2021/plots/execution_time_dec_qp_100_.dat");
//    file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;
//    for(auto i = 1; i <= 100; ++i)
//    {
//        auto pb = create_N_branches_4_steps_constrained(i);
//        plan_DecQP(pb, false);
//        file << "  " << i << " " << execution_time_ms << std::endl;
//    }
///}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

