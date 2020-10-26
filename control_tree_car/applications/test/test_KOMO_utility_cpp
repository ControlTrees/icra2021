#include <chrono>

#include <control_tree/komo/utility_komo.h>

#include <gtest/gtest.h>

TEST(ProjectOnTrajectory, NominalCase)
{
    std::vector<Pose2D> trajectory;
    trajectory.push_back({0, 0, 0});
    trajectory.push_back({1, 0, 0});
    trajectory.push_back({2, 0, 0});

    int index; double mu;
    auto proj = project_on_trajectory({0.5, 1, 0}, trajectory, index, mu);

    EXPECT_TRUE(near(proj,Pose2D({0.5, 0, 0})));
    EXPECT_EQ(index, 0);
    EXPECT_NEAR(mu, 0.5, 0.01);
}

TEST(ProjectOnTrajectory, BeforeStartPoint)
{
    std::vector<Pose2D> trajectory;
    trajectory.push_back({0, 0, 0});
    trajectory.push_back({1, 0, 0});
    trajectory.push_back({2, 0, 0});

    int index; double mu;
    auto proj = project_on_trajectory({-0.5, 1, 0}, trajectory, index, mu);

    EXPECT_TRUE(near(proj,Pose2D({-0.5, 0, 0})));
    EXPECT_EQ(index, 0);
    EXPECT_NEAR(mu, -0.5, 0.01);
}

TEST(TranslateTrajectory, TrajectorySlightlyAdvanced)
{
    std::vector<Pose2D> trajectory;
    // prefix
    trajectory.push_back({-2, 0, 0});
    trajectory.push_back({-1, 0, 0});
    // start
    trajectory.push_back({0, 0, 0});
    trajectory.push_back({1, 0, 0});
    trajectory.push_back({2, 0, 0});

    OdometryState projected_odo;
    projected_odo.x = 0.5;
    projected_odo.y = 0;
    projected_odo.yaw = 0;
    projected_odo.v = 1.0;
    projected_odo.omega = 0.0;

    translate_trajectory(projected_odo, 1, trajectory);

    EXPECT_TRUE(near(trajectory[0], Pose2D({-1.5, 0, 0})));
    EXPECT_TRUE(near(trajectory[1], Pose2D({-0.5, 0, 0})));
    EXPECT_TRUE(near(trajectory[2], Pose2D({0.5, 0, 0})));
    EXPECT_TRUE(near(trajectory[3], Pose2D({1.5, 0, 0})));
    EXPECT_TRUE(near(trajectory[4], Pose2D({2.5, 0, 0})));
}

TEST(TranslateTrajectorySecondStrategy, Straight)
{
    std::vector<Pose2D> trajectory;
    // prefix
    trajectory.push_back({-2, 0, 0});
    trajectory.push_back({-1, 0, 0});
    // start
    trajectory.push_back({0, 0, 0});
    trajectory.push_back({1, 0, 0});
    trajectory.push_back({2, 0, 0});

    slide_trajectory(1, 1, trajectory);

    EXPECT_TRUE(near(trajectory[0], Pose2D({-1, 0, 0})));
    EXPECT_TRUE(near(trajectory[1], Pose2D({0, 0, 0})));
    EXPECT_TRUE(near(trajectory[2], Pose2D({1, 0, 0})));
    EXPECT_TRUE(near(trajectory[3], Pose2D({2, 0, 0})));
    EXPECT_TRUE(near(trajectory[4], Pose2D({3, 0, 0})));
}

TEST(SlideTrajectorySecondStrategy, Curve)
{
    std::vector<Pose2D> trajectory;
    // prefix
    trajectory.push_back({-2, 0, 0});
    trajectory.push_back({-1, 0, 0});
    // start
    trajectory.push_back({0, 0, 0});
    trajectory.push_back({1, 1, 0});
    trajectory.push_back({2, 2, 0.01});

    slide_trajectory(2, 1, trajectory);

    EXPECT_TRUE(near(trajectory[0], Pose2D({0, 0, 0})));
    EXPECT_TRUE(near(trajectory[1], Pose2D({1, 1, 0})));
    EXPECT_TRUE(near(trajectory[2], Pose2D({2, 2, 0.01})));
    EXPECT_TRUE(near(trajectory[3], Pose2D({3, 3, 0.01})));
    EXPECT_TRUE(near(trajectory[4], Pose2D({4, 4, 0.01})));
}

TEST(SlideTrajectorySecondStrategySmoothMu, Curve)
{
    std::vector<Pose2D> trajectory;
    // prefix
    trajectory.push_back({-2, 0, 0});
    trajectory.push_back({-1, 0, 0});
    // start
    trajectory.push_back({0, 0, 0});
    trajectory.push_back({1, 1, 0});
    trajectory.push_back({2, 2, 0.01});

    slide_trajectory(0, 0.5, 1, trajectory);

    EXPECT_TRUE(near(trajectory[0], Pose2D({-1.5, 0, 0})));
    EXPECT_TRUE(near(trajectory[1], Pose2D({-0.5, 0, 0})));
    EXPECT_TRUE(near(trajectory[2], Pose2D({0.5, 0.5, 0})));
    EXPECT_TRUE(near(trajectory[3], Pose2D({1.5, 1.5, 0.005})));
    EXPECT_TRUE(near(trajectory[4], Pose2D({2.5, 2.5, 0.01})));
}

TEST(SlideTrajectorySecondStrategySmoothShift, Curve)
{
    std::vector<Pose2D> trajectory;
    // prefix
    trajectory.push_back({-2, 0, 0});
    trajectory.push_back({-1, 0, 0});
    // start
    trajectory.push_back({0, 0, 0});
    trajectory.push_back({1, 1, 0});
    trajectory.push_back({2, 2, 0.01});

    slide_trajectory(1, 0.0, 1, trajectory);

    EXPECT_TRUE(near(trajectory[0], Pose2D({-1, 0, 0})));
    EXPECT_TRUE(near(trajectory[1], Pose2D({0, 0, 0})));
    EXPECT_TRUE(near(trajectory[2], Pose2D({1, 1, 0})));
    EXPECT_TRUE(near(trajectory[3], Pose2D({2, 2, 0.01})));
    EXPECT_TRUE(near(trajectory[4], Pose2D({3, 3, 0.01})));
}

TEST(SlideTrajectorySecondStrategySmoothMu1EqualsShift, Curve)
{
    std::vector<Pose2D> trajectory;
    // prefix
    trajectory.push_back({-2, 0, 0});
    trajectory.push_back({-1, 0, 0});
    // start
    trajectory.push_back({0, 0, 0});
    trajectory.push_back({1, 1, 0});
    trajectory.push_back({2, 2, 0.01});

    slide_trajectory(0, 1.0, 1, trajectory);

    EXPECT_TRUE(near(trajectory[0], Pose2D({-1, 0, 0})));
    EXPECT_TRUE(near(trajectory[1], Pose2D({0, 0, 0})));
    EXPECT_TRUE(near(trajectory[2], Pose2D({1, 1, 0})));
    EXPECT_TRUE(near(trajectory[3], Pose2D({2, 2, 0.01})));
    EXPECT_TRUE(near(trajectory[4], Pose2D({3, 3, 0.01})));
}

////////////////////////////////
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

