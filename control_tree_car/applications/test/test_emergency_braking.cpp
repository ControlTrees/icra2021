#include <common/control_tree.h>

#include <qp/stopline_qp_tree.h>

#include <gtest/gtest.h>

using namespace std;

TEST(Emergency_brake_trajectory, control_vector_size)
{
    auto U = emergency_brake(15.0, 4, 4, -6.0);

    EXPECT_EQ(U.rows(), 16);
}

TEST(Emergency_brake_trajectory, null_speed)
{
    std::cout << "HERE" << std::endl;

    auto U = emergency_brake(0.0, 4, 4, -6.0);

    EXPECT_EQ(U, VectorXd::Zero(4 * 4));
}

TEST(Emergency_brake_trajectory, normal_speed)
{
    auto U = emergency_brake(10.0, 4, 4, -6.0);

    EXPECT_EQ(U[0], -6.0);

    //std::cout << U << std::endl;
}

TEST(Emergency_brake_trajectory_tree, normal_speed)
{
    auto tree = TreePb::refined(Tree2Branches(0.5), 2);

    auto U = emergency_brake(10.0, tree, 4, -6.0);

    EXPECT_EQ(U[0], -6.0);

    //std::cout << U << std::endl;
}

TEST(Emergency_brake_trajectory_tree, foo)
{
    auto tree = TreePb::refined(Tree2Branches(1.0), 4);

    auto U = emergency_brake(0.166056, tree, 4, -6.0);

    EXPECT_LE(U[0], 0);

    //std::cout << U << std::endl;
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

