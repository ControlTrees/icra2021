//#include <control_tree/qp/QP_tree_problem_DecQP.h>

#include <chrono>
#include <gtest/gtest.h>

#include <control_tree/qp/control_tree.h>
#include <control_tree/qp/control_tree_plot.h>
#include "common/qp_problems.h"


TEST_F(QPTest, test_2_branches_4_steps_constrained)
{
    auto pb = create_2_branches_4_steps_constrained();

    plan_DecQP(pb, true);
}

TEST_F(QPTest, test_paper_1_branch_4_steps)
{
    auto pb = create_paper_1_branch_4_steps_constrained();

    vel_axis.range = "[0:15]";
    acc_axis.range = "[-8:3]";

    auto U = plan_DecQP(pb, true);//, "/tmp/1_branch.dat");
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

