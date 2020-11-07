#include <common/control_tree.h>

#include <qp/QP_constraints.h>

#include <chrono>
#include <gtest/gtest.h>

using namespace std;

static double p = 0.6;

TEST(QP_constraints, test_constraints_construction)
{
    Tree2Branches tree(p);

    Constraints k(tree.n_steps, tree.varss);

    k.add_constraint(0, Vector2d(10, 0), Vector2d(1, 0));
    k.add_constraint(1, Vector2d(0, 10), Vector2d(0, 1));

    EXPECT_TRUE(k.validate());
}

TEST(QP_constraints, test_constraints_with_elements_specification)
{
    Tree2Branches tree(p);

    Constraints k(tree.n_steps, tree.varss);

    k.add_constraint(0, Vector2d(10, 10), Vector2d(1, 1), {4});
    k.add_constraint(1, Vector2d(10, 10), Vector2d(1, 1), {-1});

    EXPECT_TRUE(k.validate());

    auto Sextract = k.getSextract();

    // branch 1
    EXPECT_EQ(Sextract(0, 4*2+0), 1);
    EXPECT_EQ(Sextract(1, 4*2+1), 1);

    // branch 2
    EXPECT_EQ(Sextract(2, 8*2+0), 1);
    EXPECT_EQ(Sextract(3, 8*2+1), 1);

    std::cout << "Sextract:\n" << Sextract << std::endl;
}

TEST(QP_constraints, test_constraints_refinement)
{
    Tree2Branches tree(p);

    Constraints k(tree.n_steps, tree.varss);

    k.add_constraint(0, Vector2d(10, 10), Vector2d(1, 1)); // all
    k.add_constraint(1, Vector2d(10, 10), Vector2d(1, 1), {-1}); // last

    EXPECT_TRUE(k.validate());

    auto l = Constraints::refined(k, 2);

    EXPECT_TRUE(l.validate());
    EXPECT_EQ(l.varss.size(), k.varss.size());
    EXPECT_EQ(l.xmaxs.size(), k.xmaxs.size());

    for(auto i = 0; i < l.varss.size(); ++i)
    {
        EXPECT_EQ(l.varss[i].size(), 2 * k.varss[i].size());
    }

    for(auto i = 0; i < l.xmaxs.size(); ++i)
    {
        EXPECT_EQ(get<3>(l.xmaxs[i]).size(), 2 * get<3>(k.xmaxs[i]).size());
    }

    EXPECT_EQ(std::get<3>(l.xmaxs[0]), IntA({0, 1, 2, 3, 4, 5, 6, 7, 8, 9}));
    EXPECT_EQ(std::get<3>(l.xmaxs[1]), IntA({16, 17}));
}

TEST(QP_constraints, test_Sextract_if_no_constraints)
{
    Tree2Branches tree(p);

    Constraints k(tree.n_steps, tree.varss);

    k.n_steps = tree.n_steps;
    k.varss = tree.varss;
    k.add_constraint(0, Vector2d(10, 0), Vector2d(0, 0));
    k.add_constraint(1, Vector2d(0, 10), Vector2d(0, 0));

    auto Sextract = k.getSextract();

    EXPECT_EQ(Sextract.rows(), 0);

    std::cout << "Sextract:\n" << Sextract << std::endl;
}

TEST(QP_constraints, test_Sextract_construction)
{
    Tree2Branches tree(p);

    Constraints k(tree.n_steps, tree.varss);

    k.n_steps = tree.n_steps;
    k.varss = tree.varss;
    k.add_constraint(0, Vector2d(10, 0), Vector2d(1, 0));
    k.add_constraint(1, Vector2d(0, 10), Vector2d(0, 1));

    auto Sextract = k.getSextract();

    EXPECT_EQ(Sextract.rows(), tree.n_steps + 1); // first step has 2 constraints
    EXPECT_EQ(Sextract.cols(), 2 * tree.n_steps);

    EXPECT_EQ(Sextract(0, 0), 1); // first branch
    EXPECT_EQ(Sextract(1, 2), 1);
    EXPECT_EQ(Sextract(2, 4), 1);
    EXPECT_EQ(Sextract(3, 6), 1);
    EXPECT_EQ(Sextract(4, 8), 1);

    EXPECT_EQ(Sextract(5, 1), 1); // second branch
    EXPECT_EQ(Sextract(6, 11), 1);
    EXPECT_EQ(Sextract(7, 13), 1);
    EXPECT_EQ(Sextract(8, 15), 1);
    EXPECT_EQ(Sextract(9, 17), 1);

    std::cout << "Sextract:\n" << Sextract << std::endl;
}

TEST(QP_constraints, test_Xmax_construction)
{
    Tree2Branches tree(p);

    Constraints k(tree.n_steps, tree.varss);

    k.add_constraint(0, Vector2d(5, 0), Vector2d(1, 0));
    k.add_constraint(1, Vector2d(0, 10), Vector2d(0, 1));

    auto Xmax = k.getXmax();

    //std::cout << "Xmax:\n" << Xmax << std::endl;

    EXPECT_EQ(Xmax.rows(), 2 * tree.n_steps);

    EXPECT_EQ(Xmax(0), 5);
    EXPECT_EQ(Xmax(1), 10);

    EXPECT_EQ(Xmax(2), 5);
    EXPECT_EQ(Xmax(3), 1.);

    EXPECT_EQ(Xmax(5*2), 1.);
    EXPECT_EQ(Xmax(5*2+1), 10);
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

