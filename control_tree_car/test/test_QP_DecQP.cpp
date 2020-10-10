//#include <control_tree/qp/QP_tree_problem_DecQP.h>

#include <chrono>

#include <control_tree/qp/control_tree_plot.h>
#include <control_tree/qp/QP_tree_problem_DecQP.h>
#include "common/qp_problems.h"

#include <control_tree/core/control_tree.h>

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

    auto U = plan_JointQP(pb, false);//, "/tmp/1_branch.dat");
}

TEST_F(QPTest, DEC_test_paper_1_branch_4_steps)
{
    auto pb = create_paper_1_branch_4_steps_constrained();

    vel_axis.range = "[0:15]";
    acc_axis.range = "[-8:3]";

    auto U = plan_DecQP(pb, false);//, "/tmp/1_branch.dat");
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

TEST_F(QPTest, test_25_branches_)
{
    std::ofstream file("/home/camille/Phd/Paper/ICRA-2021/plots/execution_time_dec_qp_25.dat");
    file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;
    for(auto i = 1; i <= 25; ++i)
    {
        auto pb = create_N_branches_4_steps_constrained(i);
        plan_DecQP(pb, false);
        file << "  " << i << " " << execution_time_ms << std::endl;
    }
}

//TEST_F(QPTest, test_N_branches)
//{
//    std::ofstream file("/home/camille/Phd/Paper/ICRA-2021/plots/execution_time_dec_qp_100.dat");
//    file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;
//    for(auto i = 1; i <= 100; ++i)
//    {
//        auto pb = create_N_branches_4_steps_constrained(i);
//        plan_DecQP(pb, false);
//        file << "  " << i << " " << execution_time_ms << std::endl;
//    }
//}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

