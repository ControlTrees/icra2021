//#include <control_tree/qp/QP_tree_problem_DecQP.h>

#include <chrono>

#include <control_tree/qp/control_tree.h>
#include <control_tree/qp/control_tree_plot.h>
#include <control_tree/qp/QP_tree_problem_DecQP.h>
#include "common/qp_problems.h"

#include <gtest/gtest.h>

TEST(VarsMasks, Compress)
{
  IntA var0 {0, 1, 2, 3};
  IntA var1 {0, 4, 5, 6};

  std::vector<IntA> varss{var0, var1};

  IntA var, global_to_branch;
  auto masks = get_compressed_masks(7, 2, varss, var, global_to_branch);

  EXPECT_EQ(var, var0);
  EXPECT_EQ(masks[0], arr({1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_EQ(masks[1], arr({1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));
  EXPECT_EQ(global_to_branch, IntA({0, 1, 2, 3, 1, 2, 3}));
}

TEST(Constraints, Compress)
{
  Tree2Branches tree(0.2);

  Constraints k(tree.n_steps, tree.varss);

  k.add_constraint(0, Vector2d(10, 10), Vector2d(1, 1), {3});
  k.add_constraint(1, Vector2d(10, 10), Vector2d(1, 1), {-1});

  IntA var, global_to_branch;
  auto masks = get_compressed_masks(7, 1, tree.varss, var, global_to_branch);

  auto ks = get_compressed_constraints(k, var, global_to_branch);

  EXPECT_EQ(2, ks.size());
}

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
  auto pb = create_2_branches_4_steps_constrained();

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

TEST_F(QPTest, test_N_branches)
{
    std::ofstream file("/tmp/execution_time.dat");
    file << "#" << " " << "n" << " " << "execution time (ms)" << std::endl;
    for(auto i = 30; i <= 30; ++i)
    {
        auto pb = create_N_branches_4_steps_constrained(i);
        plan_DecQP(pb, false);
        file << "  " << i << " " << execution_time_ms << std::endl;
    }
}


////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

