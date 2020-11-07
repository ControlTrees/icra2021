#include <qp/MPC_model.h>

#include <chrono>

#include <common/control_tree.h>
#include <qp/control_tree_plot.h>

#include <gtest/gtest.h>

using namespace std;

static double p = 0.6;

TEST(MPC_model_tests, cost_zero)
{
    MPC_model model_(0.1, 1.0, 3.0);

    auto cost = model_.cost({0, 0}, 0, {0, 0});

    EXPECT_EQ(0.0, cost);
}


TEST(MPC_model_tests, cost_control)
{
    MPC_model model_(0.1, 1.0, 3.0);

    auto cost = model_.cost({0, 0}, 1.0, {0, 0});

    EXPECT_EQ(3.0, cost);
}

TEST(MPC_model_tests, cost_state)
{
    MPC_model model_(0.1, 1.0, 3.0);

    auto cost = model_.cost({0, 0}, 0.0, {0, 1});

    EXPECT_EQ(1.0, cost);
}

TEST(MPC_model_tests, test_R_simple_tree)
{
    Tree2Branches tree(p);

    MPC_model model_(0.1, 1.0, 3.0);

    auto R_bar = model_.get_R_bar(tree.n_steps, tree.varss, tree.scaless);

    std::cout << "R_bar:\n" << R_bar << std::endl;

    EXPECT_EQ(R_bar(0,0), model_.R(0));
    EXPECT_EQ(R_bar(1,1), p * model_.R(0));
    EXPECT_EQ(R_bar(5,5), (1-p) * model_.R(0));
}

TEST(MPC_model_tests, test_Q_simple_tree)
{
    Tree2Branches tree(p);

    MPC_model model_(0.1, 1.0, 3.0);

    auto Q_bar = model_.get_Q_bar(tree.n_steps, tree.varss, tree.scaless);

    std::cout << "Q_bar:\n" << Q_bar << std::endl;

    EXPECT_EQ(Q_bar(1,1), model_.Q(1, 1));
    EXPECT_EQ(Q_bar(2*1+1,2*1+1), p * model_.Q(1, 1));
    EXPECT_EQ(Q_bar(2*5+1,2*5+1), (1-p) * model_.Q(1, 1));
}

TEST(MPC_model_tests, test_T_simple_tree)
{
    Tree2Branches tree(p);

    MPC_model model_(0.1, 1.0, 3.0);

    auto T = model_.get_T(tree.n_steps, tree.varss);

    std::cout << "T:\n" << T << std::endl;

    EXPECT_EQ(T.block(0, 0, 2, 2), model_.A);
    EXPECT_EQ(T.block(2, 0, 2, 2), model_.A.pow(2)); // 1->2
    EXPECT_EQ(T.block(2*2, 0, 2, 2), model_.A.pow(3)); // 2->3
    EXPECT_EQ(T.block(2*3, 0, 2, 2), model_.A.pow(4)); // 3->4
    EXPECT_EQ(T.block(2*4, 0, 2, 2), model_.A.pow(5)); // 4->5

    EXPECT_EQ(T.block(2*4+2*1, 0, 2, 2), model_.A.pow(2)); // 1->5
    EXPECT_EQ(T.block(2*4+2*2, 0, 2, 2), model_.A.pow(3)); // 5->6
    EXPECT_EQ(T.block(2*4+2*3, 0, 2, 2), model_.A.pow(4)); // 6->7
    EXPECT_EQ(T.block(2*4+2*4, 0, 2, 2), model_.A.pow(5)); // 7->8
}

TEST(MPC_model_tests, test_T_simple_tree_N_steps_per_phase)
{
    //auto tb = build_simple_path_builder();

    Tree2Branches2Steps tree(p);

    MPC_model model_(0.1, 1.0, 3.0);

    auto T = model_.get_T(tree.n_steps, tree.varss);

    std::cout << "T:\n" << T << std::endl;

    EXPECT_EQ(T.block(0, 0, 2, 2), model_.A);
    EXPECT_EQ(T.block(2, 0, 2, 2), model_.A.pow(2)); // 1->2
    EXPECT_EQ(T.block(2*2, 0, 2, 2), model_.A.pow(3)); // 2->3
    EXPECT_EQ(T.block(2*3, 0, 2, 2), model_.A.pow(4)); // 3->4

//    EXPECT_EQ(T.block(2*3+2*1, 0, 2, 2), model_.A.pow(2)); // 1->5
//    EXPECT_EQ(T.block(2*3+2*2, 0, 2, 2), model_.A.pow(3)); // 5->6
//    EXPECT_EQ(T.block(2*3+2*3, 0, 2, 2), model_.A.pow(4)); // 6->7
}

TEST(MPC_model_tests, test_S_simple_tree)
{
    //auto tb = build_simple_path_builder();

    Tree2Branches tree(p);

    MPC_model model_(0.1, 1.0, 3.0);

    auto S = model_.get_S(tree.n_steps, tree.varss);

    std::cout << "S:\n" << S << std::endl;

    EXPECT_EQ(S.block(0, 0, 2, 1), model_.B); // 0->1

    EXPECT_EQ(S.block(2, 0, 2, 1), model_.A.pow(1) * model_.B); // 1->2
    EXPECT_EQ(S.block(2, 1, 2, 1), model_.B); // 1->2

    EXPECT_EQ(S.block(2*2, 0, 2, 1), model_.A.pow(2) * model_.B); // 2->3
    EXPECT_EQ(S.block(2*2, 1, 2, 1), model_.A.pow(1) * model_.B); // 2->3
    EXPECT_EQ(S.block(2*2, 2, 2, 1), model_.A.pow(0) * model_.B); // 2->3

    EXPECT_EQ(S.block(2*3, 0, 2, 1), model_.A.pow(3) * model_.B); // 3->4
    EXPECT_EQ(S.block(2*3, 1, 2, 1), model_.A.pow(2) * model_.B); // 3->4
    EXPECT_EQ(S.block(2*3, 2, 2, 1), model_.A.pow(1) * model_.B); // 3->4
    EXPECT_EQ(S.block(2*3, 3, 2, 1), model_.A.pow(0) * model_.B); // 3->4

    EXPECT_EQ(S.block(2*4, 0, 2, 1), model_.A.pow(4) * model_.B); // 4->5
    EXPECT_EQ(S.block(2*4, 1, 2, 1), model_.A.pow(3) * model_.B); // 4->5
    EXPECT_EQ(S.block(2*4, 2, 2, 1), model_.A.pow(2) * model_.B); // 4->5
    EXPECT_EQ(S.block(2*4, 3, 2, 1), model_.A.pow(1) * model_.B); // 4->5
    EXPECT_EQ(S.block(2*4, 4, 2, 1), model_.A.pow(0) * model_.B); // 4->5

    EXPECT_EQ(S.block(2*5, 0, 2, 1), model_.A.pow(1) * model_.B); // 1->6
    EXPECT_EQ(S.block(2*5, 5, 2, 1), model_.B); // 1->6
}

TEST(TreeBb, refinement)
{
    auto tree = TreePb::refined(Tree2Branches(p), 2);

    auto tree_2 = Tree2Branches2Steps(p);

    EXPECT_EQ(tree.varss, tree_2.varss);
    EXPECT_EQ(tree.scaless, tree_2.scaless);
    EXPECT_EQ(tree.n_steps, tree_2.n_steps);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

