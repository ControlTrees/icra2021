#include <common/control_tree.h>
#include <qp/control_tree_plot.h>
#include <qp/QP_tree_problem_DecQP.h>
#include "common/qp_problems.h"

#include <chrono>
#include <gtest/gtest.h>

namespace
{
    bool close(const Arr& a, const Arr& b)
    {
        if(a.size() != b.size())
            return false;

        bool ret = true;
        for(auto i = 0; i < a.size(); ++i)
        {
            ret = ret && fabs(a[i] - b[i]) < 0.0001;
        }

        return ret;
    }

    bool close(const std::vector<Arr> & a, const std::vector<Arr> & b)
    {
        if(a.size() != b.size())
            return false;

        bool ret = true;
        for(auto i = 0; i < a.size(); ++i)
        {
            ret = ret && close(a[i], b[i]);
        }

        return ret;
    }
}

TEST(Tree, TreeNBranches)
{
  {
  Tree2Branches tree(0.4);
  TreeNBranches treen(std::vector<double>({0.4}));

  EXPECT_EQ(tree.varss, treen.varss);
  EXPECT_EQ(tree.scaless, treen.scaless);
  EXPECT_EQ(tree.n_steps, treen.n_steps);
  }

  {
  Tree4Branches tree(0.3, 0.35, 0.2);
  TreeNBranches treen(std::vector<double>({0.3, 0.35, 0.2}));

  EXPECT_EQ(tree.varss, treen.varss);
  EXPECT_TRUE(close(tree.scaless, treen.scaless));
  EXPECT_EQ(tree.n_steps, treen.n_steps);
  }
}

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
  auto masks = get_compressed_masks(9, 1, tree.varss, var, global_to_branch);

  auto ks = get_compressed_constraints(k, var, global_to_branch);

  EXPECT_EQ(2, ks.size());
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

